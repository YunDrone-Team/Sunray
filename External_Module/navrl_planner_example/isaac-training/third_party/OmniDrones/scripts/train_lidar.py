import logging
import os

import hydra
import torch
import numpy as np
import pandas as pd
import wandb
import matplotlib.pyplot as plt

from tqdm import tqdm
from omegaconf import OmegaConf

from omni_drones import init_simulation_app
from torchrl.data import CompositeSpec, TensorSpec
from torchrl.envs.utils import set_exploration_type, ExplorationType
from omni_drones.utils.torchrl import SyncDataCollector
from omni_drones.utils.torchrl.transforms import (
    FromMultiDiscreteAction, 
    FromDiscreteAction,
    ravel_composite,
    VelController,
    AttitudeController,
    RateController,
)
from omni_drones.utils.wandb import init_wandb
from omni_drones.utils.torchrl import RenderCallback, EpisodeStats

from setproctitle import setproctitle
from torchrl.envs.transforms import TransformedEnv, InitTracker, Compose

import torch.nn as nn
import torch.nn.functional as F
import einops
from einops.layers.torch import Rearrange
from omni_drones.learning.ppo.ppo import PPOConfig, make_mlp, make_batch, Actor, IndependentNormal, GAE, ValueNorm1
from tensordict import TensorDict
from tensordict.nn import TensorDictSequential, TensorDictModule, TensorDictModuleBase
from torchrl.envs.transforms import CatTensors
from torchrl.modules import ProbabilisticActor


class PPOPolicy(TensorDictModuleBase):

    def __init__(self, cfg: PPOConfig, observation_spec: CompositeSpec, action_spec: CompositeSpec, reward_spec: TensorSpec, device):
        super().__init__()
        self.cfg = cfg
        self.device = device

        self.entropy_coef = 0.001
        self.clip_param = 0.1
        self.critic_loss_fn = nn.HuberLoss(delta=10)
        self.n_agents, self.action_dim = action_spec.shape[-2:]
        self.gae = GAE(0.99, 0.95)

        fake_input = observation_spec.zero()

        cnn = nn.Sequential(
            nn.LazyConv2d(out_channels=4, kernel_size=[5, 3], padding=[2, 1]), nn.ELU(), 
            nn.LazyConv2d(out_channels=16, kernel_size=[5, 3], stride=[2, 1], padding=[2, 1]), nn.ELU(),
            nn.LazyConv2d(out_channels=16, kernel_size=[5, 3], stride=[2, 2], padding=[2, 1]), nn.ELU(),
            Rearrange("n c w h -> n (c w h)"),
            nn.LazyLinear(128), nn.LayerNorm(128)
        )
        mlp = make_mlp([256, 256])
        
        # input encoder: encoder主要的作用是把所有的input （LIDAR + states）提取成feature
        self.encoder = TensorDictSequential(
            #  [("agents", "observation", "lidar")]表示连续使用三次key，这里是对LIDARinput进行encode
            TensorDictModule(cnn, [("agents", "observation", "lidar")], ["_cnn_feature"]),
            CatTensors(["_cnn_feature", ("agents", "observation", "state")], "_feature", del_keys=False), # 将LiDAR的feature和无人机的stateconcatenate
            TensorDictModule(mlp, ["_feature"], ["_feature"]),
        ).to(self.device)

        # Actor Network
        self.actor = ProbabilisticActor(
            TensorDictModule(Actor(self.action_dim), ["_feature"], ["loc", "scale"]), # actor maps feature from encoder to loc (mean) and scale (standard deviation)
            in_keys=["loc", "scale"],
            out_keys=[("agents", "action")], # 将output存在agent：action下
            distribution_class=IndependentNormal,
            return_log_prob=True
        ).to(self.device)

        # Critic Network
        self.critic = TensorDictModule(
            nn.LazyLinear(1), ["_feature"], ["state_value"] # 从feature到value
        ).to(self.device)

        self.encoder(fake_input)
        self.actor(fake_input)
        self.critic(fake_input)

        def init_(module):
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, 0.01)
                nn.init.constant_(module.bias, 0.)
        # 初始化weights
        self.actor.apply(init_)
        self.critic.apply(init_)
        
        self.encoder_opt = torch.optim.Adam(self.encoder.parameters(), lr=5e-4)
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=5e-4)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=5e-4)
        self.value_norm = ValueNorm1(1).to(self.device)

    def __call__(self, tensordict: TensorDict):
        self.encoder(tensordict)
        self.actor(tensordict)
        self.critic(tensordict)
        tensordict.exclude("loc", "scale", "_feature", inplace=True)
        return tensordict
    
    def train_op(self, tensordict: TensorDict):
        next_tensordict = tensordict["next"]
        with torch.no_grad():
            # 将下一个states所有的batch放入encoder，得到encoder的feature
            next_tensordict = torch.vmap(self.encoder)(next_tensordict) # vmap: https://pytorch.org/docs/stable/generated/torch.vmap.html for function over some dimension inputs
            # 通过encoder的feature获取S'的value
            next_values = self.critic(next_tensordict)["state_value"]
        # 下一个states的reward
        rewards = tensordict[("next", "agents", "reward")] # shape [num_env, train_every (每隔多少step训一次？), 1]
        # 下一个states是否为terminal state
        dones = tensordict[("next", "terminated")] # shape [num_env, 32, 1]

        # 当前的所有batchstate value （这里的value实际上是normalized value需要进一步的denormalize）
        values = tensordict["state_value"] # 在forward中记录 
        values = self.value_norm.denormalize(values)
        next_values = self.value_norm.denormalize(next_values)

        # GAE: Generalized Advantage Estimation
        adv, ret = self.gae(rewards, dones, values, next_values)
        adv_mean = adv.mean()
        adv_std = adv.std()
        adv = (adv - adv_mean) / adv_std.clip(1e-7)
        self.value_norm.update(ret)
        ret = self.value_norm.normalize(ret)

        # 加入advantage以及return
        tensordict.set("adv", adv)
        tensordict.set("ret", ret)

        # Training
        infos = []
        for epoch in range(self.cfg.ppo_epochs):
            batch = make_batch(tensordict, self.cfg.num_minibatches)
            for minibatch in batch:
                infos.append(self._update(minibatch))
        
        infos: TensorDict = torch.stack(infos).to_tensordict()
        infos = infos.apply(torch.mean, batch_size=[])
        # print({k: v.item() for k, v in infos.items()})
        return {k: v.item() for k, v in infos.items()}

    def _update(self, tensordict: TensorDict):
        # print("my print in _update: ", tensordict)
        self.encoder(tensordict) # 
        # 1. get action distribution from the ppo actor
        dist = self.actor.get_dist(tensordict) # (batch size, feature)-> (batch size, number_of_actions)
        #. convert it into log probability
        log_probs = dist.log_prob(tensordict[("agents", "action")])
        entropy = dist.entropy()

        # standard actor loss
        adv = tensordict["adv"] # calculated entropy
        ratio = torch.exp(log_probs - tensordict["sample_log_prob"]).unsqueeze(-1)
        surr1 = adv * ratio
        surr2 = adv * ratio.clamp(1.-self.clip_param, 1.+self.clip_param)
        policy_loss = - torch.mean(torch.min(surr1, surr2)) * self.action_dim # actor loss
        entropy_loss = - self.entropy_coef * torch.mean(entropy) # this term is to encourage exploration

        # standard cirtic loss
        b_values = tensordict["state_value"]
        b_returns = tensordict["ret"]
        values = self.critic(tensordict)["state_value"]
        values_clipped = b_values + (values - b_values).clamp(
            -self.clip_param, self.clip_param
        )
        value_loss_clipped = self.critic_loss_fn(b_returns, values_clipped)
        value_loss_original = self.critic_loss_fn(b_returns, values)
        value_loss = torch.max(value_loss_original, value_loss_clipped)

        loss = policy_loss + entropy_loss + value_loss
        self.encoder_opt.zero_grad()
        self.actor_opt.zero_grad()
        self.critic_opt.zero_grad()
        loss.backward()
        actor_grad_norm = nn.utils.clip_grad.clip_grad_norm_(self.actor.parameters(), 5)
        critic_grad_norm = nn.utils.clip_grad.clip_grad_norm_(self.critic.parameters(), 5)
        self.encoder_opt.step()
        self.actor_opt.step()
        self.critic_opt.step()
        explained_var = 1 - F.mse_loss(values, b_returns) / b_returns.var()
        return TensorDict({
            "policy_loss": policy_loss,
            "value_loss": value_loss,
            "entropy": entropy,
            "actor_grad_norm": actor_grad_norm,
            "critic_grad_norm": critic_grad_norm,
            "explained_var": explained_var
        }, [])


FILE_PATH = os.path.dirname(__file__)
# Config File: 
# 1: ./train.yaml  
# 2: ../cfg/train.yaml
# 3. ../cfg/task/Forest.yaml


@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg):
    # OmageConf使用python eval计算config中需要计算的部分
    OmegaConf.register_new_resolver("eval", eval)
    OmegaConf.resolve(cfg)
    OmegaConf.set_struct(cfg, False)
    
    simulation_app = init_simulation_app(cfg)
    run = init_wandb(cfg)
    setproctitle(run.name)

    print(OmegaConf.to_yaml(cfg))

    # 初始化simulation环境
    from omni_drones.envs.isaac_env import IsaacEnv # 会收集所有训练环境
    env_class = IsaacEnv.REGISTRY[cfg.task.name] # 训练的任务task的class: Forest class
    base_env = env_class(cfg, headless=cfg.headless) # 创建训练的环境

    # This transform populates the step/reset tensordict with a reset tracker entry that is set to True whenever reset() is called.
    transforms = [InitTracker()] # https://pytorch.org/rl/reference/generated/torchrl.envs.transforms.InitTracker.html

    # a CompositeSpec is by deafault processed by a entity-based encoder
    # ravel it to use a MLP encoder instead
    # 把环境的传入的input变成netowrk能使用的input (flatten + concat)
    
    if cfg.task.get("ravel_obs", False):
        transform = ravel_composite(base_env.observation_spec, ("agents", "observation"))
        transforms.append(transform)
        print("ravel obsrevation")
    if cfg.task.get("ravel_obs_central", False):
        transform = ravel_composite(base_env.observation_spec, ("agents", "observation_central"))
        transforms.append(transform)
        print("ravel observation central")
    if (
        cfg.task.get("flatten_intrinsics", True) # no flatten intrinsic so flatten it
        and ("agents", "intrinsics") in base_env.observation_spec.keys(True)
        and isinstance(base_env.observation_spec[("agents", "intrinsics")], CompositeSpec)
    ):
        # actually here
        transforms.append(ravel_composite(base_env.observation_spec, ("agents", "intrinsics"), start_dim=-1))
        print("flatten intrinsics")
    # if cfg.task.get("history", False):
    #     # transforms.append(History([("info", "drone_state"), ("info", "prev_action")]))
    #     transforms.append(History([("agents", "observation")]))

    # optionally discretize the action space or use a controller
    action_transform: str = cfg.task.get("action_transform", None)
    # print("check action tranform: ", action_transform)
    if action_transform is not None:
        if action_transform.startswith("multidiscrete"):
            nbins = int(action_transform.split(":")[1])
            transform = FromMultiDiscreteAction(nbins=nbins)
            transforms.append(transform)
        elif action_transform.startswith("discrete"):
            nbins = int(action_transform.split(":")[1])
            transform = FromDiscreteAction(nbins=nbins)
            transforms.append(transform)
        elif action_transform == "velocity":
            from omni_drones.controllers import LeePositionController
            controller = LeePositionController(9.81, base_env.drone.params).to(base_env.device)
            # transform = VelController(torch.vmap(controller))
            transform = VelController(controller)
            transforms.append(transform)
        elif action_transform == "rate":
            from omni_drones.controllers import RateController as _RateController
            controller = _RateController(9.81, base_env.drone.params).to(base_env.device)
            transform = RateController(controller)
            transforms.append(transform)
        elif action_transform == "attitude":
            from omni_drones.controllers import AttitudeController as _AttitudeController
            controller = _AttitudeController(9.81, base_env.drone.params).to(base_env.device)
            transform = AttitudeController(torch.vmap(torch.vmap(controller)))
            transforms.append(transform)
        elif not action_transform.lower() == "none":
            raise NotImplementedError(f"Unknown action transform: {action_transform}")
    # based env指的是原来的环境，env指的是变换后的环境
    env = TransformedEnv(base_env, Compose(*transforms)).train()
    env.set_seed(cfg.seed)

    # RL policy
    policy = PPOPolicy(
        cfg.algo, 
        env.observation_spec, 
        env.action_spec, 
        env.reward_spec, 
        device=base_env.device
    )

    frames_per_batch = env.num_envs * int(cfg.algo.train_every) # 环境数量（batch） x 每个环境每次训练的帧数（多少个time step？）
    total_frames = cfg.get("total_frames", -1) // frames_per_batch * frames_per_batch # 总共多少frame/
    max_iters = cfg.get("max_iters", -1)
    eval_interval = cfg.get("eval_interval", -1)
    save_interval = cfg.get("save_interval", -1)

    stats_keys = [
        k for k in base_env.observation_spec.keys(True, True) 
        if isinstance(k, tuple) and k[0]=="stats"
    ]
    episode_stats = EpisodeStats(stats_keys)
    collector = SyncDataCollector(
        env,
        policy=policy, # PPO Training policy
        frames_per_batch=frames_per_batch, # 
        total_frames=total_frames,
        device=cfg.sim.device,
        return_same_td=True,
    )

    @torch.no_grad()
    def evaluate(
        seed: int=0, 
        exploration_type: ExplorationType=ExplorationType.MODE
    ):

        base_env.enable_render(True)
        base_env.eval()
        env.eval()
        env.set_seed(seed)

        render_callback = RenderCallback(interval=2)
        
        with set_exploration_type(exploration_type):
            trajs = env.rollout(
                max_steps=base_env.max_episode_length,
                policy=policy,
                callback=render_callback,
                auto_reset=True,
                break_when_any_done=False,
                return_contiguous=False,
            )
        base_env.enable_render(not cfg.headless)
        env.reset()

        done = trajs.get(("next", "done"))
        first_done = torch.argmax(done.long(), dim=1).cpu()

        def take_first_episode(tensor: torch.Tensor):
            indices = first_done.reshape(first_done.shape+(1,)*(tensor.ndim-2))
            return torch.take_along_dim(tensor, indices, dim=1).reshape(-1)

        traj_stats = {
            k: take_first_episode(v)
            for k, v in trajs[("next", "stats")].cpu().items()
        }

        info = {
            "eval/stats." + k: torch.mean(v.float()).item() 
            for k, v in traj_stats.items()
        }

        # log video
        info["recording"] = wandb.Video(
            render_callback.get_video_array(axes="t c h w"), 
            fps=0.5 / (cfg.sim.dt * cfg.sim.substeps), 
            format="mp4"
        )
        
        # log distributions
        # df = pd.DataFrame(traj_stats)
        # table = wandb.Table(dataframe=df)
        # info["eval/return"] = wandb.plot.histogram(table, "return")
        # info["eval/episode_len"] = wandb.plot.histogram(table, "episode_len")

        return info

    pbar = tqdm(collector)
    env.train()
    for i, data in enumerate(pbar):
        info = {"env_frames": collector._frames, "rollout_fps": collector._fps}

        # print("My print: ", data.to_tensordict)
        # return

        episode_stats.add(data.to_tensordict())
        # print(data.to_tensordict())
        # return
        if len(episode_stats) >= base_env.num_envs:
            stats = {
                "train/" + (".".join(k) if isinstance(k, tuple) else k): torch.mean(v.float()).item() 
                for k, v in episode_stats.pop().items(True, True)
            }
            info.update(stats)
            print("my info: ", info)
        info.update(policy.train_op(data.to_tensordict()))

        if eval_interval > 0 and i % eval_interval == 0:
            logging.info(f"Eval at {collector._frames} steps.")
            info.update(evaluate())
            env.train()
            base_env.train()

        if save_interval > 0 and i % save_interval == 0:
            try:
                ckpt_path = os.path.join(run.dir, f"checkpoint_{collector._frames}.pt")
                torch.save(policy.state_dict(), ckpt_path)
                logging.info(f"Saved checkpoint to {str(ckpt_path)}")
            except AttributeError:
                logging.warning(f"Policy {policy} does not implement `.state_dict()`")

        run.log(info)
        print(OmegaConf.to_yaml({k: v for k, v in info.items() if isinstance(v, float)}))

        pbar.set_postfix({"rollout_fps": collector._fps, "frames": collector._frames})

        if max_iters > 0 and i >= max_iters - 1:
            break 
    
    logging.info(f"Final Eval at {collector._frames} steps.")
    info = {"env_frames": collector._frames}
    info.update(evaluate())
    run.log(info)

    try:
        ckpt_path = os.path.join(run.dir, "checkpoint_final.pt")
        torch.save(policy.state_dict(), ckpt_path)
        logging.info(f"Saved checkpoint to {str(ckpt_path)}")
    except AttributeError:
        logging.warning(f"Policy {policy} does not implement `.state_dict()`")
        

    wandb.finish()
    
    simulation_app.close()


if __name__ == "__main__":
    main()

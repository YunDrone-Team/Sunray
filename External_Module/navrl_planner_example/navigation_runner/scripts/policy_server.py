#!/usr/bin/env python3
import rospy
import hydra
import os
import torch
from torchrl.data import CompositeSpec, UnboundedContinuousTensorSpec
from ppo import PPO
import numpy as np
from navigation_runner.srv import GetPolicyInference
from torchrl.envs.utils import ExplorationType, set_exploration_type
from tensordict.tensordict import TensorDict

class policy_server:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lidar_hbeams = int(360/self.cfg.sensor.lidar_hres)
        self.raycast_vres = ((self.cfg.sensor.lidar_vfov[1] - self.cfg.sensor.lidar_vfov[0]))/(self.cfg.sensor.lidar_vbeams - 1) * np.pi/180.0
        self.raycast_hres = self.cfg.sensor.lidar_hres * np.pi/180.0
        self.policy = self.init_model()
        self.policy.eval()

    def init_model(self):
        observation_dim = 8
        num_dim_each_dyn_obs_state = 10
        observation_spec = CompositeSpec({
            "agents": CompositeSpec({
                "observation": CompositeSpec({
                    "state": UnboundedContinuousTensorSpec((observation_dim,), device=self.cfg.device), 
                    "lidar": UnboundedContinuousTensorSpec((1, self.lidar_hbeams, self.cfg.sensor.lidar_vbeams), device=self.cfg.device),
                    "direction": UnboundedContinuousTensorSpec((1, 3), device=self.cfg.device),
                    "dynamic_obstacle": UnboundedContinuousTensorSpec((1, self.cfg.algo.feature_extractor.dyn_obs_num, num_dim_each_dyn_obs_state), device=self.cfg.device),
                }),
            }).expand(1)
        }, shape=[1], device=self.cfg.device)

        action_dim = 3
        action_spec = CompositeSpec({
            "agents": CompositeSpec({
                "action": UnboundedContinuousTensorSpec((action_dim,), device=self.cfg.device), 
            })
        }).expand(1, action_dim).to(self.cfg.device)

        policy = PPO(self.cfg.algo, observation_spec, action_spec, self.cfg.device)


        checkpoint = "/home/zhefan/rl_ws/src/nav-ros/navigation_runner/scripts/ckpts/120obs-21000.pt"

        policy.load_state_dict(torch.load(checkpoint))
        print("[policy server]: model init success!")
        return policy
    
    def handle_inference(self, req):
        # Convert request data to input format for the model
        # Assume req contains observation data in the necessary format
        drone_state = torch.tensor(req.state, device=self.cfg.device).view(req.state_shape)
        lidar_scan = torch.tensor(req.lidar, device=self.cfg.device).view(req.lidar_shape)
        target_dir_2d = torch.tensor(req.direction, device=self.cfg.device).view(req.direction_shape)
        dyn_obs_states = torch.tensor(req.dynamic_obstacle, device=self.cfg.device).view(req.dynamic_obstacle_shape)
        
        obs = TensorDict({
            "agents": TensorDict({
                "observation": TensorDict({
                    "state": drone_state,
                    "lidar": lidar_scan,
                    "direction": target_dir_2d,
                    "dynamic_obstacle": dyn_obs_states
                })
            })
        })
        # Perform inference
        with set_exploration_type(ExplorationType.MEAN):
            output = self.policy(obs)  # Modify according to your model's inference method
        vel_world = output["agents", "action"]
        # Return the result
        # print(vel_world.cpu().detach().squeeze(0).squeeze(0).numpy().tolist())
        return {'action': vel_world.cpu().detach().squeeze(0).squeeze(0).numpy().tolist()}

    def start_server(self):
        rospy.Service('rl_navigation/GetPolicyInference', GetPolicyInference, self.handle_inference)


FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts/cfg")
@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg):
    rospy.init_node('policy_server')
    ps = policy_server(cfg)
    ps.start_server()
    print("[policy server]: ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
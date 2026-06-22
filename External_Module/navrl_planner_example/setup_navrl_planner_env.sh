#!/usr/bin/env bash
set -euo pipefail

# 中文使用指南
# ============
#
# 这个脚本用于配置 NavRL 规划器的部署/推理环境，不用于配置完整的
# Isaac Sim / Orbit 训练环境。脚本会创建或复用一个 Conda 环境，安装
# NavRL 推理所需的 Python 依赖、PyTorch、TensorDict、TorchRL，并可选
# 安装 ROS Noetic 相关系统依赖、编译 Sunray 的 navrl 构建组。
#
# 一、前置条件
# 1. 需要已经安装 Conda，并且当前 shell 中可以直接执行 conda。
# 2. 默认会检查 /opt/ros/noetic/setup.bash，因此机器上应安装 ROS Noetic。
#    如果只想安装 Python/Conda 依赖，可以添加 --no-ros-deps 跳过 apt 依赖安装。
# 3. 需要保留本目录下的 isaac-training/third_party/tensordict 和
#    isaac-training/third_party/rl 源码目录；脚本会以 editable 方式安装它们。
# 4. NavRL 策略权重不在仓库内。运行 planner 节点前，需要将权重文件放到：
#    navigation_runner/scripts/ckpts/navrl_checkpoint.pt
#
# 二、CPU 环境
# 默认就是 CPU 环境，会安装 PyTorch 2.0.1 CPU wheel：
#   ./setup_navrl_planner_env.sh
# 或显式指定：
#   ./setup_navrl_planner_env.sh --torch cpu
#
# 如果想单独创建一个 CPU 环境，推荐显式命名：
#   ./setup_navrl_planner_env.sh --env NavRL-cpu --torch cpu
#
# 三、CUDA 环境
# 当前脚本只支持 PyTorch CUDA 11.8 wheel，对应参数是 cu118：
#   ./setup_navrl_planner_env.sh --env NavRL-cu118 --torch cu118
#
# 注意：
# 1. --torch cu118 只表示安装 CUDA 11.8 版本的 PyTorch wheel，不会安装显卡驱动。
# 2. 机器仍需要具备可用 NVIDIA 驱动，并且驱动版本需要兼容 CUDA 11.8 wheel。
# 3. 当前脚本没有提供 cu121、cu126 等其他 CUDA wheel 后端。
# 4. 不建议在同一个 Conda 环境里反复切换 CPU/CUDA wheel；需要两套环境时，
#    请使用不同的 --env 名称分别创建。
#
# 四、常用命令
# 创建默认 CPU 环境：
#   ./setup_navrl_planner_env.sh
#
# 创建 CUDA 11.8 环境：
#   ./setup_navrl_planner_env.sh --env NavRL-cu118 --torch cu118
#
# 跳过 ROS/system apt 依赖安装：
#   ./setup_navrl_planner_env.sh --no-ros-deps
#
# 安装依赖后顺便编译 navrl 构建组：
#   ./setup_navrl_planner_env.sh --build
#
# 删除并重建已有 Conda 环境：
#   ./setup_navrl_planner_env.sh --recreate
#
# 组合示例：重建 CUDA 环境并编译 navrl：
#   ./setup_navrl_planner_env.sh --env NavRL-cu118 --torch cu118 --recreate --build
#
# 五、环境变量写法
# 所有主要参数也可以用环境变量传入，例如：
#   ENV_NAME=NavRL-cpu TORCH_BACKEND=cpu ./setup_navrl_planner_env.sh
#   ENV_NAME=NavRL-cu118 TORCH_BACKEND=cu118 BUILD_NAVRL=1 ./setup_navrl_planner_env.sh
#
# 六、安装完成后
# 后续使用环境：
#   conda activate NavRL
#
# 如果使用了自定义环境名，需要激活对应名称：
#   conda activate NavRL-cpu
#   conda activate NavRL-cu118
#
# 七、CPU 推理提醒
# 脚本可以安装 CPU 版本依赖，但 NavRL 的部分上游配置默认可能写成 cuda:0。
# 在 CPU 机器上运行 planner 前，需要确认运行配置中的 device 已改为 cpu，
# 否则即使环境安装成功，运行时也可能因为尝试访问 CUDA 设备而失败。
#
# Setup script for the NavRL planner deployment/inference environment.
# It intentionally does not install the full Isaac Sim / Orbit training stack.

ENV_NAME="${ENV_NAME:-NavRL}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"
TORCH_BACKEND="${TORCH_BACKEND:-cpu}" # cpu or cu118
INSTALL_ROS_DEPS="${INSTALL_ROS_DEPS:-1}"
BUILD_NAVRL="${BUILD_NAVRL:-0}"
RECREATE_ENV="${RECREATE_ENV:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
ISAAC_TRAINING_DIR="${SCRIPT_DIR}/isaac-training"
TENSOR_DICT_DIR="${ISAAC_TRAINING_DIR}/third_party/tensordict"
TORCHRL_DIR="${ISAAC_TRAINING_DIR}/third_party/rl"

log() {
  printf '[navrl-env] %s\n' "$*"
}

die() {
  printf '[navrl-env] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'USAGE'
Usage:
  ./setup_navrl_planner_env.sh [options]

Options:
  --env NAME          Conda env name. Default: NavRL
  --python VERSION   Python version. Default: 3.10
  --torch cpu|cu118  PyTorch wheel backend. Default: cpu
  --no-ros-deps      Skip apt installation of ROS/system dependencies
  --build            Run ./build.sh -y navrl after installing dependencies
  --recreate         Remove the existing Conda env before creating it
  -h, --help         Show this help

Environment variable equivalents:
  ENV_NAME, PYTHON_VERSION, TORCH_BACKEND, INSTALL_ROS_DEPS, BUILD_NAVRL,
  RECREATE_ENV

Notes:
  - This script targets NavRL deployment/inference, not Isaac Sim training.
  - The upstream deployment script pins Conda Python to 3.10.
  - The policy checkpoint is not included in this repository; place it under
    navigation_runner/scripts/ckpts/navrl_checkpoint.pt before running the
    planner node.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)
      ENV_NAME="${2:-}"
      [[ -n "${ENV_NAME}" ]] || die "--env requires a value"
      shift 2
      ;;
    --python)
      PYTHON_VERSION="${2:-}"
      [[ -n "${PYTHON_VERSION}" ]] || die "--python requires a value"
      shift 2
      ;;
    --torch)
      TORCH_BACKEND="${2:-}"
      [[ "${TORCH_BACKEND}" == "cpu" || "${TORCH_BACKEND}" == "cu118" ]] || die "--torch must be cpu or cu118"
      shift 2
      ;;
    --no-ros-deps)
      INSTALL_ROS_DEPS=0
      shift
      ;;
    --build)
      BUILD_NAVRL=1
      shift
      ;;
    --recreate)
      RECREATE_ENV=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
done

command -v conda >/dev/null 2>&1 || die "conda is not available in PATH"
[[ -d "${TENSOR_DICT_DIR}" ]] || die "Missing TensorDict source: ${TENSOR_DICT_DIR}"
[[ -d "${TORCHRL_DIR}" ]] || die "Missing TorchRL source: ${TORCHRL_DIR}"

eval "$(conda shell.bash hook)"

if [[ "${INSTALL_ROS_DEPS}" == "1" ]]; then
  if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
    die "ROS Noetic not found at /opt/ros/noetic/setup.bash; install ROS first or pass --no-ros-deps"
  fi

  log "Installing ROS/system dependencies with apt"
  sudo apt-get update
  sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    python3-catkin-tools \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-mavros-msgs \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-vision-msgs
fi

if [[ "${RECREATE_ENV}" == "1" ]] && conda env list | awk '{print $1}' | grep -Fxq "${ENV_NAME}"; then
  log "Removing existing Conda env: ${ENV_NAME}"
  conda env remove -n "${ENV_NAME}" -y
fi

if ! conda env list | awk '{print $1}' | grep -Fxq "${ENV_NAME}"; then
  log "Creating Conda env ${ENV_NAME} with Python ${PYTHON_VERSION}"
  conda create -n "${ENV_NAME}" "python=${PYTHON_VERSION}" -c conda-forge -y
else
  log "Reusing existing Conda env: ${ENV_NAME}"
fi

conda activate "${ENV_NAME}"
python -m pip install --upgrade pip
python -m pip install "setuptools<70" wheel packaging

log "Installing Python packages"
python -m pip install \
  numpy==1.26.4 \
  "pydantic!=1.7,!=1.7.1,!=1.7.2,!=1.7.3,!=1.8,!=1.8.1,<2.0.0,>=1.6.2" \
  imageio-ffmpeg==0.4.9 \
  moviepy==1.0.3 \
  hydra-core \
  einops \
  pyyaml \
  rospkg \
  matplotlib \
  opencv-python \
  ninja

case "${TORCH_BACKEND}" in
  cpu)
    log "Installing PyTorch 2.0.1 CPU wheels"
    python -m pip install \
      torch==2.0.1+cpu \
      torchvision==0.15.2+cpu \
      torchaudio==2.0.2+cpu \
      --index-url https://download.pytorch.org/whl/cpu
    ;;
  cu118)
    log "Installing PyTorch 2.0.1 CUDA 11.8 wheels"
    python -m pip install \
      torch==2.0.1 \
      torchvision==0.15.2 \
      torchaudio==2.0.2 \
      --index-url https://download.pytorch.org/whl/cu118
    ;;
esac

log "Installing local TensorDict and TorchRL sources in editable mode"
python -m pip uninstall -y tensordict torchrl >/dev/null 2>&1 || true
python -m pip install tomli
python -m pip install --no-build-isolation --no-deps -e "${TENSOR_DICT_DIR}"
python -m pip install --no-build-isolation --no-deps -e "${TORCHRL_DIR}"

if [[ -r /opt/ros/noetic/setup.bash ]]; then
  # Keep this env file explicit because Conda activation alone does not expose
  # ROS Noetic Python modules or generated catkin message/service packages.
  ENV_HOOK="${CONDA_PREFIX}/etc/conda/activate.d/navrl_ros_paths.sh"
  mkdir -p "$(dirname "${ENV_HOOK}")"
  cat > "${ENV_HOOK}" <<EOF
#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
if [[ -r "${REPO_ROOT}/devel/setup.bash" ]]; then
  source "${REPO_ROOT}/devel/setup.bash"
fi
export NAVRL_PLANNER_EXAMPLE_DIR="${SCRIPT_DIR}"
EOF
  chmod +x "${ENV_HOOK}"
  # shellcheck source=/dev/null
  source "${ENV_HOOK}"
fi

log "Verifying core Python imports"
python - <<'PY'
import sys
import torch
import tensordict
import torchrl
import hydra
import yaml
import rospkg

print("python:", sys.version.split()[0])
print("torch:", torch.__version__, "cuda_available:", torch.cuda.is_available())
print("tensordict:", getattr(tensordict, "__version__", "unknown"))
print("torchrl:", getattr(torchrl, "__version__", "unknown"))
print("hydra:", hydra.__version__)
print("yaml:", yaml.__version__)
PY

if [[ "${BUILD_NAVRL}" == "1" ]]; then
  log "Building Sunray navrl group"
  cd "${REPO_ROOT}"
  ./build.sh -y navrl
fi

CHECKPOINT="${SCRIPT_DIR}/navigation_runner/scripts/ckpts/navrl_checkpoint.pt"
if [[ ! -f "${CHECKPOINT}" ]]; then
  log "Policy checkpoint is still missing: ${CHECKPOINT}"
fi

log "Done"
log "Activate later with: conda activate ${ENV_NAME}"

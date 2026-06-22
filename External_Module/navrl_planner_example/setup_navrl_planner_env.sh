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
# 三、x86 CUDA 环境
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
# 四、Jetson Orin 环境
# Jetson 使用 aarch64 + JetPack/L4T 自带 CUDA 栈，不能安装 x86 PC 的 cu118 wheel。
# 需要从 NVIDIA Jetson PyTorch 兼容矩阵中选择匹配 JetPack 的 torch wheel：
#   ./setup_navrl_planner_env.sh \
#     --env NavRL-jetson \
#     --torch jetson \
#     --jetson-torch-url https://developer.download.nvidia.com/compute/redist/jp/v.../pytorch/torch-...-linux_aarch64.whl
#
# 脚本会从 wheel 文件名中的 cp38/cp310 标签推断或校验 Conda Python 版本。
# 如果你已经下载了 wheel，也可以传本地路径：
#   ./setup_navrl_planner_env.sh --torch jetson --jetson-torch-url /path/to/torch-...linux_aarch64.whl
#
# torchvision/torchaudio 在 Jetson 上也需要匹配 torch 和 JetPack。如果有可用 wheel，
# 可以通过 --jetson-torchvision-url / --jetson-torchaudio-url 传入；未传入时脚本会跳过，
# 避免 pip 自动装错版本并覆盖 Jetson PyTorch。
#
# 脚本默认会在安装目标 PyTorch 前清理已有 torch/torchvision/torchaudio 以及
# pip 安装的 PC CUDA runtime 包（例如 nvidia-*-cu11/cu12）。从 cu118 切到
# cpu/jetson 时建议保留这个默认行为。如果你明确想复用已有 PyTorch，可传：
#   ./setup_navrl_planner_env.sh --keep-existing-torch
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
# 创建 Jetson 环境：
#   ./setup_navrl_planner_env.sh --env NavRL-jetson --torch jetson --jetson-torch-url "$TORCH_INSTALL"
#
# 五、环境变量写法
# 所有主要参数也可以用环境变量传入，例如：
#   ENV_NAME=NavRL-cpu TORCH_BACKEND=cpu ./setup_navrl_planner_env.sh
#   ENV_NAME=NavRL-cu118 TORCH_BACKEND=cu118 BUILD_NAVRL=1 ./setup_navrl_planner_env.sh
#   ENV_NAME=NavRL-jetson TORCH_BACKEND=jetson JETSON_TORCH_URL="$TORCH_INSTALL" ./setup_navrl_planner_env.sh
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
if [[ -v PYTHON_VERSION ]]; then
  PYTHON_VERSION_EXPLICIT=1
else
  PYTHON_VERSION_EXPLICIT=0
fi
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"
TORCH_BACKEND="${TORCH_BACKEND:-cpu}" # cpu, cu118, or jetson
INSTALL_ROS_DEPS="${INSTALL_ROS_DEPS:-1}"
INSTALL_JETSON_DEPS="${INSTALL_JETSON_DEPS:-1}"
BUILD_NAVRL="${BUILD_NAVRL:-0}"
RECREATE_ENV="${RECREATE_ENV:-0}"
CLEAN_EXISTING_TORCH="${CLEAN_EXISTING_TORCH:-1}"
JETSON_TORCH_URL="${JETSON_TORCH_URL:-${TORCH_INSTALL:-}}"
JETSON_TORCHVISION_URL="${JETSON_TORCHVISION_URL:-}"
JETSON_TORCHAUDIO_URL="${JETSON_TORCHAUDIO_URL:-}"

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

detect_l4t_release() {
  if command -v dpkg-query >/dev/null 2>&1 && dpkg-query -W -f='${Version}' nvidia-l4t-core >/dev/null 2>&1; then
    dpkg-query -W -f='${Version}' nvidia-l4t-core
    return 0
  fi

  if [[ -r /etc/nv_tegra_release ]]; then
    sed -n 's/^# R\([0-9]\+\).*/R\1/p' /etc/nv_tegra_release | head -n1
    return 0
  fi

  echo "unknown"
}

jetpack_hint_from_l4t() {
  local l4t="$1"
  case "$l4t" in
    32.*|R32*) echo "JetPack 4.x" ;;
    35.*|R35*) echo "JetPack 5.x" ;;
    36.*|R36*) echo "JetPack 6.x" ;;
    38.*|R38*) echo "JetPack 7.x" ;;
    *) echo "unknown JetPack" ;;
  esac
}

python_abi_tag() {
  python - <<'PY'
import sys
print(f"cp{sys.version_info.major}{sys.version_info.minor}")
PY
}

python_version_from_abi() {
  local abi="$1"
  local digits="${abi#cp}"
  if [[ ! "$digits" =~ ^[0-9]+$ ]]; then
    return 1
  fi
  printf '%s.%s\n' "${digits:0:1}" "${digits:1}"
}

wheel_abi_tag() {
  local wheel_name="${1##*/}"
  if [[ "$wheel_name" =~ (cp[0-9]+)-cp[0-9]+-linux_aarch64\.whl ]]; then
    echo "${BASH_REMATCH[1]}"
  fi
}

validate_jetson_wheel() {
  local label="$1"
  local wheel="$2"
  local wheel_abi current_abi expected_python

  [[ -n "$wheel" ]] || die "${label} wheel URL/path is empty"
  [[ "$wheel" == *.whl ]] || die "${label} must point to a .whl file: $wheel"
  if [[ "$wheel" != *linux_aarch64.whl ]]; then
    log "Warning: ${label} wheel does not look like an aarch64 Jetson wheel: $wheel"
  fi

  wheel_abi="$(wheel_abi_tag "$wheel")"
  [[ -z "$wheel_abi" ]] && return 0

  current_abi="$(python_abi_tag)"
  if [[ "$wheel_abi" != "$current_abi" ]]; then
    expected_python="$(python_version_from_abi "$wheel_abi" || true)"
    die "${label} wheel ABI is ${wheel_abi}, but current Conda Python ABI is ${current_abi}. Recreate with --python ${expected_python:-matching-version}."
  fi
}

prepare_jetson_python_version() {
  local wheel_abi expected_python

  [[ "${TORCH_BACKEND}" == "jetson" ]] || return 0
  [[ -n "${JETSON_TORCH_URL}" ]] || return 0

  wheel_abi="$(wheel_abi_tag "${JETSON_TORCH_URL}")"
  [[ -z "$wheel_abi" ]] && return 0

  expected_python="$(python_version_from_abi "$wheel_abi" || true)"
  [[ -z "$expected_python" ]] && return 0

  if [[ "$PYTHON_VERSION_EXPLICIT" == "0" ]]; then
    if [[ "$PYTHON_VERSION" != "$expected_python" ]]; then
      log "Jetson torch wheel uses ${wheel_abi}; using Conda Python ${expected_python}"
      PYTHON_VERSION="$expected_python"
    fi
  elif [[ "$PYTHON_VERSION" != "$expected_python" ]]; then
    die "Jetson torch wheel uses ${wheel_abi}; --python ${PYTHON_VERSION} is incompatible. Use --python ${expected_python}."
  fi
}

check_jetson_request() {
  local arch l4t jetpack_hint

  [[ "${TORCH_BACKEND}" == "jetson" ]] || return 0

  arch="$(uname -m)"
  [[ "$arch" == "aarch64" ]] || die "--torch jetson requires aarch64, current arch is ${arch}. Use --torch cpu or --torch cu118 on x86_64."

  l4t="$(detect_l4t_release)"
  jetpack_hint="$(jetpack_hint_from_l4t "$l4t")"
  log "Detected Jetson platform: arch=${arch}, L4T=${l4t}, ${jetpack_hint}"

  if [[ -z "${JETSON_TORCH_URL}" ]]; then
    die "--torch jetson requires --jetson-torch-url or JETSON_TORCH_URL/TORCH_INSTALL. Choose the wheel from NVIDIA's Jetson PyTorch compatibility matrix for this JetPack."
  fi
}

install_jetson_system_deps() {
  [[ "${TORCH_BACKEND}" == "jetson" ]] || return 0
  [[ "${INSTALL_JETSON_DEPS}" == "1" ]] || return 0

  log "Installing Jetson PyTorch system dependencies with apt"
  sudo apt-get update
  sudo apt-get install -y \
    python3-pip \
    libopenblas-dev
}

cleanup_existing_torch_packages() {
  [[ "${CLEAN_EXISTING_TORCH}" == "1" ]] || {
    log "Keeping existing torch packages because --keep-existing-torch was requested"
    return 0
  }

  log "Cleaning existing PyTorch packages before installing ${TORCH_BACKEND} backend"
  python -m pip uninstall -y torch torchvision torchaudio >/dev/null 2>&1 || true

  local cuda_packages
  cuda_packages="$(
    python - <<'PY'
import importlib.metadata as md

names = []
for dist in md.distributions():
    name = (dist.metadata.get("Name") or "").strip()
    normalized = name.lower().replace("_", "-")
    if normalized.startswith("nvidia-") and ("-cu11" in normalized or "-cu12" in normalized or "-cu13" in normalized):
        names.append(name)

print(" ".join(sorted(set(names))))
PY
  )"

  if [[ -n "${cuda_packages}" ]]; then
    log "Removing pip CUDA runtime packages: ${cuda_packages}"
    # shellcheck disable=SC2086
    python -m pip uninstall -y ${cuda_packages} >/dev/null 2>&1 || true
  fi
}

usage() {
  cat <<'USAGE'
Usage:
  ./setup_navrl_planner_env.sh [options]

Options:
  --env NAME          Conda env name. Default: NavRL
  --python VERSION   Python version. Default: 3.10
  --torch cpu|cu118|jetson
                      PyTorch wheel backend. Default: cpu
  --jetson-torch-url URL_OR_PATH
                      NVIDIA Jetson torch wheel URL/path. Also accepts
                      JETSON_TORCH_URL or NVIDIA's TORCH_INSTALL env var.
  --jetson-torchvision-url URL_OR_PATH
                      Optional Jetson-compatible torchvision wheel URL/path.
  --jetson-torchaudio-url URL_OR_PATH
                      Optional Jetson-compatible torchaudio wheel URL/path.
  --no-ros-deps      Skip apt installation of ROS/system dependencies
  --no-jetson-deps   Skip apt installation of Jetson PyTorch system deps
  --keep-existing-torch
                      Do not uninstall existing torch/torchvision/torchaudio
                      or pip CUDA runtime packages before installing torch
  --build            Run ./build.sh -y navrl after installing dependencies
  --recreate         Remove the existing Conda env before creating it
  -h, --help         Show this help

Environment variable equivalents:
  ENV_NAME, PYTHON_VERSION, TORCH_BACKEND, INSTALL_ROS_DEPS, BUILD_NAVRL,
  RECREATE_ENV, CLEAN_EXISTING_TORCH, INSTALL_JETSON_DEPS, JETSON_TORCH_URL,
  JETSON_TORCHVISION_URL, JETSON_TORCHAUDIO_URL

Notes:
  - This script targets NavRL deployment/inference, not Isaac Sim training.
  - The upstream deployment script pins Conda Python to 3.10.
  - Jetson mode does not use x86 CUDA wheels. It installs the NVIDIA Jetson
    aarch64 torch wheel you provide and checks that its cpXX ABI matches the
    Conda Python version.
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
      PYTHON_VERSION_EXPLICIT=1
      shift 2
      ;;
    --torch)
      TORCH_BACKEND="${2:-}"
      [[ "${TORCH_BACKEND}" == "cpu" || "${TORCH_BACKEND}" == "cu118" || "${TORCH_BACKEND}" == "jetson" ]] || die "--torch must be cpu, cu118, or jetson"
      shift 2
      ;;
    --jetson-torch-url)
      JETSON_TORCH_URL="${2:-}"
      [[ -n "${JETSON_TORCH_URL}" ]] || die "--jetson-torch-url requires a value"
      shift 2
      ;;
    --jetson-torchvision-url)
      JETSON_TORCHVISION_URL="${2:-}"
      [[ -n "${JETSON_TORCHVISION_URL}" ]] || die "--jetson-torchvision-url requires a value"
      shift 2
      ;;
    --jetson-torchaudio-url)
      JETSON_TORCHAUDIO_URL="${2:-}"
      [[ -n "${JETSON_TORCHAUDIO_URL}" ]] || die "--jetson-torchaudio-url requires a value"
      shift 2
      ;;
    --no-ros-deps)
      INSTALL_ROS_DEPS=0
      shift
      ;;
    --no-jetson-deps)
      INSTALL_JETSON_DEPS=0
      shift
      ;;
    --keep-existing-torch)
      CLEAN_EXISTING_TORCH=0
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

check_jetson_request
prepare_jetson_python_version

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

install_jetson_system_deps

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

cleanup_existing_torch_packages

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
  jetson)
    log "Installing NVIDIA Jetson PyTorch wheel"
    validate_jetson_wheel "torch" "${JETSON_TORCH_URL}"
    python -m pip install --no-cache-dir "${JETSON_TORCH_URL}"

    if [[ -n "${JETSON_TORCHVISION_URL}" ]]; then
      log "Installing Jetson-compatible torchvision wheel"
      validate_jetson_wheel "torchvision" "${JETSON_TORCHVISION_URL}"
      python -m pip install --no-cache-dir --no-deps "${JETSON_TORCHVISION_URL}"
    else
      log "No Jetson torchvision wheel supplied; skipping torchvision to avoid replacing Jetson torch"
      log "If you use onboard_detector/yolo_detector, install a matching Jetson torchvision manually"
    fi

    if [[ -n "${JETSON_TORCHAUDIO_URL}" ]]; then
      log "Installing Jetson-compatible torchaudio wheel"
      validate_jetson_wheel "torchaudio" "${JETSON_TORCHAUDIO_URL}"
      python -m pip install --no-cache-dir --no-deps "${JETSON_TORCHAUDIO_URL}"
    else
      log "No Jetson torchaudio wheel supplied; skipping torchaudio"
    fi
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

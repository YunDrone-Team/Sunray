#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="${SCRIPT_DIR}"
WORKSPACE_ROOT="$(cd "${PACKAGE_ROOT}/../.." && pwd)"

SKIP_INSTALL=0
SKIP_BUILD=0

usage() {
  cat <<'EOF'
Usage: setup_sunray_test.sh [options]

Options:
  --skip-install   Do not install/check Python packages.
  --skip-build     Do not build the sunray_test package.
  -h, --help       Show this help message.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-install)
      SKIP_INSTALL=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

log() {
  echo "[sunray_test setup] $*"
}

python_module_exists() {
  local module_name="$1"
  python3 - "$module_name" <<'PY'
import importlib.util
import sys

module = sys.argv[1]
sys.exit(0 if importlib.util.find_spec(module) else 1)
PY
}

source_ros_distribution_environment() {
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    log "Sourcing /opt/ros/noetic/setup.bash"
    # shellcheck source=/dev/null
    source /opt/ros/noetic/setup.bash
  fi
}

install_apt_packages() {
  local apt_packages=("$@")
  if [[ ${#apt_packages[@]} -eq 0 ]]; then
    return 0
  fi

  if ! command -v apt-get >/dev/null 2>&1 || ! command -v sudo >/dev/null 2>&1; then
    return 1
  fi

  log "Installing apt dependencies: ${apt_packages[*]}"
  sudo apt-get update
  sudo apt-get install -y "${apt_packages[@]}"
}

ensure_pip() {
  if python3 -m pip --version >/dev/null 2>&1; then
    return
  fi

  log "python3-pip not found; trying to install it with apt."
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y python3-pip
  else
    echo "python3-pip is required, and sudo is not available." >&2
    echo "Please install it manually: apt-get install -y python3-pip" >&2
    exit 1
  fi
}

install_python_deps() {
  local apt_packages=()
  local pip_packages=()

  python_module_exists yaml || {
    apt_packages+=("python3-yaml")
    pip_packages+=("PyYAML")
  }
  python_module_exists numpy || {
    apt_packages+=("python3-numpy")
    pip_packages+=("numpy")
  }
  python_module_exists pandas || {
    apt_packages+=("python3-pandas")
    pip_packages+=("pandas")
  }
  python_module_exists rosbag || {
    apt_packages+=("python3-rosbag")
  }
  python_module_exists rospy || {
    apt_packages+=("python3-rospy")
  }
  python_module_exists roslib || {
    apt_packages+=("python3-roslib")
  }

  if [[ ${#pip_packages[@]} -eq 0 && ${#apt_packages[@]} -eq 0 ]]; then
    log "Python dependencies already available."
    return
  fi

  if install_apt_packages "${apt_packages[@]}"; then
    local missing_after_apt=()
    python_module_exists yaml || missing_after_apt+=("PyYAML")
    python_module_exists numpy || missing_after_apt+=("numpy")
    python_module_exists pandas || missing_after_apt+=("pandas")
    pip_packages=("${missing_after_apt[@]}")
    if ! python_module_exists rosbag || ! python_module_exists rospy || ! python_module_exists roslib; then
      echo "ROS Python modules are still missing after apt install." >&2
      echo "Please check ROS Noetic installation and source /opt/ros/noetic/setup.bash." >&2
      exit 1
    fi
    if [[ ${#pip_packages[@]} -eq 0 ]]; then
      log "Python dependencies installed via apt."
      return
    fi
  fi

  ensure_pip
  log "Installing Python dependencies: ${pip_packages[*]}"
  python3 -m pip install --user "${pip_packages[@]}"
}

source_ros_environment() {
  if command -v catkin_make >/dev/null 2>&1; then
    return
  fi

  source_ros_distribution_environment

  if ! command -v catkin_make >/dev/null 2>&1; then
    echo "catkin_make not found. Please install/source ROS Noetic first." >&2
    exit 1
  fi
}

build_sunray_test() {
  source_ros_environment
  log "Building sunray_test package"
  (
    cd "${WORKSPACE_ROOT}"
    catkin_make --source General_Module/sunray_test --build build/sunray_test
  )
  log "Build finished."
  log "Run this in your shell before rosrun: source ${WORKSPACE_ROOT}/devel/setup.bash"
}

log "Package root: ${PACKAGE_ROOT}"
log "Workspace root: ${WORKSPACE_ROOT}"

source_ros_distribution_environment

if [[ "${SKIP_INSTALL}" -eq 0 ]]; then
  install_python_deps
fi

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  build_sunray_test
fi

log "Done."

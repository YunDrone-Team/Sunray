#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

log() {
  echo "[sunray_test deps] $*"
}

resolve_config_file() {
  if [[ -n "${SUNRAY_TEST_RUNTIME_DEPS_CONFIG:-}" ]]; then
    echo "$SUNRAY_TEST_RUNTIME_DEPS_CONFIG"
    return
  fi

  local candidates=(
    "$PACKAGE_ROOT/config/runtime_deps.json"
    "$SCRIPT_DIR/../../share/sunray_test/config/runtime_deps.json"
  )

  if command -v rospack >/dev/null 2>&1; then
    local package_path
    package_path="$(rospack find sunray_test 2>/dev/null || true)"
    if [[ -n "$package_path" ]]; then
      candidates+=("$package_path/config/runtime_deps.json")
    fi
  fi

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "$candidate" ]]; then
      echo "$candidate"
      return
    fi
  done

  echo "Cannot locate runtime_deps.json." >&2
  exit 1
}

CONFIG_FILE="$(resolve_config_file)"

python_module_exists() {
  local module_name="$1"
  python3 - "$module_name" <<'PY'
import importlib.util
import sys

module = sys.argv[1]
sys.exit(0 if importlib.util.find_spec(module) else 1)
PY
}

config_value() {
  local path="$1"
  local default_value="${2:-}"
  python3 - "$CONFIG_FILE" "$path" "$default_value" <<'PY'
import json
import sys

config_path, dotted_path, default_value = sys.argv[1:4]
with open(config_path, encoding="utf-8") as f:
    data = json.load(f)

value = data
for key in dotted_path.split("."):
    if not isinstance(value, dict) or key not in value:
        print(default_value)
        sys.exit(0)
    value = value[key]

print(value if value is not None else default_value)
PY
}

python_dep_rows() {
  python3 - "$CONFIG_FILE" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)

for dep in data.get("python", {}).get("modules", []):
    print("\t".join([
        dep.get("module", ""),
        dep.get("apt", ""),
        dep.get("pip", ""),
    ]))
PY
}

system_dep_rows() {
  python3 - "$CONFIG_FILE" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)

for dep in data.get("system", {}).get("commands", []):
    print("\t".join([
        dep.get("command", ""),
        dep.get("apt", ""),
    ]))
PY
}

source_ros_distribution_environment() {
  local ros_distro="${ROS_DISTRO:-$(config_value ros.default_distro noetic)}"
  local setup_file="/opt/ros/$ros_distro/setup.bash"
  if [[ -f "$setup_file" ]]; then
    # shellcheck source=/dev/null
    source "$setup_file"
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

  log "安装 apt 依赖: ${apt_packages[*]}"
  sudo apt-get update
  sudo apt-get install -y "${apt_packages[@]}"
}

ensure_pip() {
  if python3 -m pip --version >/dev/null 2>&1; then
    return
  fi

  if command -v sudo >/dev/null 2>&1; then
    local pip_bootstrap_apt
    pip_bootstrap_apt="$(config_value python.pip_bootstrap_apt python3-pip)"
    log "安装 $pip_bootstrap_apt"
    sudo apt-get update
    sudo apt-get install -y "$pip_bootstrap_apt"
    return
  fi

  echo "缺少 python3-pip，且当前环境没有 sudo。" >&2
  echo "请手动安装: apt-get install -y python3-pip" >&2
  exit 1
}

install_python_deps() {
  local apt_packages=()
  local pip_packages=()
  local modules=()
  local pip_by_module=()
  local module_name apt_package pip_package

  while IFS=$'\t' read -r module_name apt_package pip_package; do
    [[ -z "$module_name" ]] && continue
    modules+=("$module_name")
    pip_by_module+=("$pip_package")
    if python_module_exists "$module_name"; then
      continue
    fi
    [[ -n "$apt_package" ]] && apt_packages+=("$apt_package")
    [[ -n "$pip_package" ]] && pip_packages+=("$pip_package")
  done < <(python_dep_rows)

  if [[ ${#apt_packages[@]} -eq 0 && ${#pip_packages[@]} -eq 0 ]]; then
    log "Python 依赖已满足。"
    return
  fi

  if install_apt_packages "${apt_packages[@]}"; then
    pip_packages=()
    local index
    for index in "${!modules[@]}"; do
      module_name="${modules[$index]}"
      pip_package="${pip_by_module[$index]}"
      if python_module_exists "$module_name"; then
        continue
      fi
      if [[ -n "$pip_package" ]]; then
        pip_packages+=("$pip_package")
      else
        echo "Python 模块仍然缺失: $module_name" >&2
        echo "请检查 ROS 环境或手动安装对应 apt 依赖。" >&2
        exit 1
      fi
    done

    if [[ ${#pip_packages[@]} -eq 0 ]]; then
      log "Python 依赖已通过 apt 安装。"
      return
    fi
  elif [[ ${#pip_packages[@]} -eq 0 ]]; then
    echo "缺少 Python 模块，且没有可用的 pip fallback。" >&2
    echo "请检查 sudo/apt 权限或 ROS 环境后重试。" >&2
    exit 1
  fi

  ensure_pip
  log "安装 Python 依赖: ${pip_packages[*]}"
  python3 -m pip install --user "${pip_packages[@]}"

  local missing_modules=()
  for module_name in "${modules[@]}"; do
    python_module_exists "$module_name" || missing_modules+=("$module_name")
  done
  if [[ ${#missing_modules[@]} -gt 0 ]]; then
    echo "Python 模块仍然缺失: ${missing_modules[*]}" >&2
    exit 1
  fi
}

install_system_deps() {
  local apt_packages=()
  local command_name apt_package

  while IFS=$'\t' read -r command_name apt_package; do
    [[ -z "$command_name" ]] && continue
    command -v "$command_name" >/dev/null 2>&1 && continue
    [[ -n "$apt_package" ]] && apt_packages+=("$apt_package")
  done < <(system_dep_rows)

  if [[ ${#apt_packages[@]} -eq 0 ]]; then
    log "系统依赖已满足。"
    return
  fi

  if ! install_apt_packages "${apt_packages[@]}"; then
    echo "系统依赖安装失败: ${apt_packages[*]}" >&2
    echo "请手动安装后重试。" >&2
    exit 1
  fi
}

source_ros_distribution_environment
install_python_deps
install_system_deps

#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from typing import Any, Dict, List, Set


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.core.runner import RunnerArgs, TestRunner
from sunray_test.core.suite_loader import load_platform_config, load_yaml


DEFAULT_PLATFORM = "sunray150_basic"


def parse_args():
    parser = argparse.ArgumentParser(description="Interactively choose and run a sunray test suite")
    parser.add_argument("--platform", default="")
    parser.add_argument("--env", default="exp")
    parser.add_argument("--suite", default="")
    parser.add_argument("--uav-id", type=int, default=1)
    parser.add_argument("--external-source", type=int, default=None)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--list", action="store_true", help="List available suites and exit")
    parser.add_argument("--skip-mid360-ip-check", action="store_true", help="Skip MID360 IP precheck")
    parser.add_argument("--apply-mid360-config", action="store_true", help="Update MID360_config.json when detected IP differs")
    parser.add_argument("--mid360-iface", default="", help="Override MID360 Ethernet interface")
    parser.add_argument("--mid360-config", default="", help="Override MID360_config.json path")
    parser.add_argument("--mid360-timeout", type=float, default=8.0, help="MID360 discovery timeout seconds")
    parser.add_argument("--mid360-no-sudo", action="store_true", help="Run tcpdump without sudo")
    return parser.parse_args()


def _suite_dir() -> str:
    return os.path.join(PACKAGE_ROOT, "config", "suites")


def _platform_dir() -> str:
    return os.path.join(PACKAGE_ROOT, "config", "platforms")


def _environment_dir() -> str:
    return os.path.join(PACKAGE_ROOT, "config", "environments")


def _load_suite_entries() -> List[Dict[str, str]]:
    entries: List[Dict[str, str]] = []
    suite_dir = _suite_dir()
    if not os.path.isdir(suite_dir):
        return entries

    for filename in sorted(os.listdir(suite_dir)):
        if not filename.endswith(".yaml"):
            continue
        path = os.path.join(suite_dir, filename)
        suite_name = os.path.splitext(filename)[0]
        suite_data = load_yaml(path)
        report = suite_data.get("report", {})
        display_name = str(report.get("title", "")).strip() or suite_name
        description = str(suite_data.get("description", "")).strip()
        step_count = len(suite_data.get("steps", []))
        entries.append(
            {
                "name": suite_name,
                "display_name": display_name,
                "description": description,
                "step_count": str(step_count),
            }
        )
    return entries


def _print_suite_list(entries: List[Dict[str, str]]) -> None:
    for entry in entries:
        suffix = f" | steps={entry['step_count']}"
        if entry["description"]:
            suffix += f" | {entry['description']}"
        print(f"{entry['name']}\t{entry['display_name']}{suffix}")


def _select_suite(entries: List[Dict[str, str]]) -> str:
    if not entries:
        raise RuntimeError(f"no suites found under {_suite_dir()}")

    print("可用 suites：", flush=True)
    for index, entry in enumerate(entries, start=1):
        label = f"{index}) {entry['name']}"
        if entry["display_name"] != entry["name"]:
            label += f"  ({entry['display_name']})"
        label += f"  [steps={entry['step_count']}]"
        if entry["description"]:
            label += f"  {entry['description']}"
        print(label, flush=True)

    while True:
        print("", flush=True)
        choice = input("请选择 suite 编号: ").strip()
        if not choice.isdigit():
            print("请输入有效编号。", flush=True)
            continue
        index = int(choice)
        if 1 <= index <= len(entries):
            return entries[index - 1]["name"]
        print("编号超出范围。", flush=True)


def _load_platform_topics(platform_name: str) -> Set[str]:
    platform = _load_platform_config(platform_name)
    return set((platform.get("topics") or {}).keys())


def _load_platform_config(platform_name: str) -> Dict[str, Any]:
    return load_platform_config(os.path.join(PACKAGE_ROOT, "config"), platform_name)


def _load_platform_capabilities(platform_name: str) -> Set[str]:
    platform = _load_platform_config(platform_name)
    return {
        str(name)
        for name, enabled in (platform.get("capabilities") or {}).items()
        if bool(enabled)
    }


def _load_environment_topic_keys(environment_name: str) -> Set[str]:
    path = os.path.join(_environment_dir(), f"{environment_name}.yaml")
    if not os.path.isfile(path):
        return set()
    environment = load_yaml(path)
    return set((environment.get("topic_overrides") or {}).keys())


def _suite_required_topic_keys(suite_name: str) -> Set[str]:
    path = os.path.join(_suite_dir(), f"{suite_name}.yaml")
    suite = load_yaml(path)
    required_topic_keys = set()
    for step in suite.get("steps", []):
        params = step.get("params", {}) if isinstance(step, dict) else {}
        topic_key = params.get("topic_key")
        if topic_key:
            required_topic_keys.add(str(topic_key))
    return required_topic_keys


def _suite_platform_requirements(suite_name: str) -> Dict[str, Set[str]]:
    path = os.path.join(_suite_dir(), f"{suite_name}.yaml")
    suite = load_yaml(path)
    requirements = suite.get("platform_requirements") or {}
    topics = {str(item) for item in requirements.get("topics", [])}
    capabilities = {str(item) for item in requirements.get("capabilities", [])}
    topics.update(_suite_required_topic_keys(suite_name))
    return {"topics": topics, "capabilities": capabilities}


def _load_platform_names() -> List[str]:
    platform_dir = _platform_dir()
    if not os.path.isdir(platform_dir):
        return []
    return [
        os.path.splitext(filename)[0]
        for filename in sorted(os.listdir(platform_dir))
        if filename.endswith(".yaml") and not filename.startswith("_")
    ]


def _platform_supports_suite(platform_name: str, environment_name: str, requirements: Dict[str, Set[str]]) -> bool:
    required_topic_keys = requirements.get("topics", set())
    required_capabilities = requirements.get("capabilities", set())
    available_topic_keys = _load_platform_topics(platform_name) | _load_environment_topic_keys(environment_name)
    available_capabilities = _load_platform_capabilities(platform_name)
    return (
        required_topic_keys.issubset(available_topic_keys)
        and required_capabilities.issubset(available_capabilities)
    )


def _resolve_platform_for_suite(suite_name: str, environment_name: str) -> str:
    requirements = _suite_platform_requirements(suite_name)
    required_topic_keys = requirements.get("topics", set())
    required_capabilities = requirements.get("capabilities", set())
    if not required_topic_keys and not required_capabilities:
        return DEFAULT_PLATFORM

    if _platform_supports_suite(DEFAULT_PLATFORM, environment_name, requirements):
        return DEFAULT_PLATFORM

    compatible_platforms = [
        platform_name
        for platform_name in _load_platform_names()
        if _platform_supports_suite(platform_name, environment_name, requirements)
    ]
    if len(compatible_platforms) == 1:
        selected_platform = compatible_platforms[0]
        print(
            f"[sunray_test] suite={suite_name} 需要 topics={sorted(required_topic_keys)} "
            f"capabilities={sorted(required_capabilities)}，"
            f"自动选择 platform={selected_platform}",
            flush=True,
        )
        return selected_platform
    if compatible_platforms:
        print("可用 platforms：", flush=True)
        for index, platform_name in enumerate(compatible_platforms, start=1):
            print(f"{index}) {platform_name}", flush=True)
        while True:
            print("", flush=True)
            choice = input("请选择 platform 编号: ").strip()
            if choice.isdigit() and 1 <= int(choice) <= len(compatible_platforms):
                return compatible_platforms[int(choice) - 1]
            print("请输入有效编号。", flush=True)

    print(
        f"[sunray_test] 未找到满足 suite={suite_name} requirements 的 platform，"
        f"topics={sorted(required_topic_keys)} capabilities={sorted(required_capabilities)}，"
        f"回退 platform={DEFAULT_PLATFORM}",
        flush=True,
    )
    return DEFAULT_PLATFORM


def _run_mid360_ip_check(args, platform_name: str) -> None:
    if args.skip_mid360_ip_check:
        return
    if args.environment.strip().lower() != "exp":
        return

    platform = _load_platform_config(platform_name)
    capabilities = platform.get("capabilities") or {}
    lidar_config = platform.get("lidar") or {}
    if not bool(capabilities.get("egoplanner", False)) and not lidar_config:
        return
    if lidar_config and not bool(lidar_config.get("mid360_auto_check", False)):
        return

    iface = args.mid360_iface.strip() or str(lidar_config.get("mid360_iface", "eth0"))
    config_path = os.path.expanduser(
        args.mid360_config.strip()
        or str(
            lidar_config.get(
                "mid360_config_path",
                "~/sunray_map/src/livox_ros_driver2/config/MID360_config.json",
            )
        )
    )

    command = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "livox_mid360_autoconfig.py"),
        "--iface",
        iface,
        "--timeout",
        str(args.mid360_timeout),
        "--config",
        config_path,
    ]
    if args.mid360_no_sudo:
        command.append("--no-sudo")
    if args.apply_mid360_config:
        command.append("--apply")
    else:
        command.append("--require-match")

    mode = "apply" if args.apply_mid360_config else "check"
    print(f"[sunray_test] MID360 IP {mode}: iface={iface} config={config_path}", flush=True)
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        print(
            "[sunray_test] MID360 IP 检查失败。"
            "如需自动写入检测到的雷达 IP，请加 --apply-mid360-config；"
            "仿真环境不会执行该检查。",
            flush=True,
        )
        raise SystemExit(completed.returncode)


def main():
    try:
        args = parse_args()
        entries = _load_suite_entries()

        if args.list:
            _print_suite_list(entries)
            return 0

        suite_name = args.suite.strip() or _select_suite(entries)
        platform_name = args.platform.strip() or _resolve_platform_for_suite(suite_name, args.environment)
        _run_mid360_ip_check(args, platform_name)
        runner = TestRunner(
            RunnerArgs(
                platform=platform_name,
                environment=args.environment,
                suite=suite_name,
                uav_id=args.uav_id,
                external_source=args.external_source,
                output_dir=args.output_dir,
                prompt_metadata=False,
            )
        )
        raise SystemExit(runner.run())
    except KeyboardInterrupt:
        print("\n[sunray_test] 收到 Ctrl+C，测试已中断。", flush=True)
        raise SystemExit(130)


if __name__ == "__main__":
    main()

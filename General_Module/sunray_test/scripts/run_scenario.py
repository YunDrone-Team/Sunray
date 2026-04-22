#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
import sys
import time
import shutil
from typing import Dict, List


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.core.context import package_root_from_file, workspace_root_from_package
from sunray_test.core.scenario_loader import list_scenarios, load_scenario
from sunray_test.core.suite_loader import load_config_triplet


def parse_args():
    parser = argparse.ArgumentParser(description="Launch a configured Sunray test scenario")
    parser.add_argument("--scenario", default="", help="Scenario name from config/scenarios/*.yaml")
    parser.add_argument("--list", action="store_true", help="List available scenarios")
    parser.add_argument("--platform", default="")
    parser.add_argument("--environment", default="")
    parser.add_argument("--suite", default="")
    parser.add_argument("--uav-id", type=int, default=None)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--sn", default="")
    parser.add_argument("--tester", default="")
    parser.add_argument("--no-prompt", action="store_true")
    return parser.parse_args()


def _detect_terminal() -> str:
    terminal = "gnome-terminal"
    if not shutil.which(terminal):
        raise RuntimeError("gnome-terminal not found")
    return terminal


def _compose_bash_command(command: str, delay_s: float, hold_open: bool) -> str:
    parts: List[str] = []
    if delay_s > 0:
        parts.append(f"sleep {delay_s}")
    if hold_open:
        parts.append(
            "{ "
            + command
            + '; status=$?; echo ""; echo "[sunray_test] tab exited with status ${status}. Press Ctrl+D to close."; exec bash; }'
        )
        return "; ".join(parts)
    parts.insert(0, "set -e")
    parts.append(command)
    return "; ".join(parts)


def _launch_window(terminal: str, window: Dict[str, object]) -> None:
    tabs = window.get("tabs", [])
    if not tabs:
        return

    command: List[str] = [terminal]
    for index, tab in enumerate(tabs):
        title = str(tab.get("title") or tab.get("name") or window.get("title") or "sunray_test")
        command.append("--window" if index == 0 else "--tab")
        command.extend(["--title", title, "--command"])
        command.extend(
            [
                shlex.join(
                    [
                        "bash",
                        "-lc",
                        _compose_bash_command(
                            str(tab["command"]),
                            float(tab.get("delay_s", 0.0)),
                            bool(tab.get("hold_open", True)),
                        ),
                    ]
                ),
            ]
        )

    subprocess.Popen(command)


def _runner_cli_args(runner: Dict[str, object], args) -> List[str]:
    cli_args: List[str] = [
        "--platform",
        str(runner["platform"]),
        "--environment",
        str(runner["environment"]),
        "--suite",
        str(runner["suite"]),
        "--uav-id",
        str(runner["uav_id"]),
    ]
    output_dir = args.output_dir or str(runner.get("output_dir", "")).strip()
    if output_dir:
        cli_args.extend(["--output-dir", output_dir])
    sn = args.sn or str(runner.get("sn", "")).strip()
    if sn:
        cli_args.extend(["--sn", sn])
    tester = args.tester or str(runner.get("tester", "")).strip()
    if tester:
        cli_args.extend(["--tester", tester])
    if args.no_prompt or bool(runner.get("no_prompt", False)):
        cli_args.append("--no-prompt")
    return cli_args


def main():
    args = parse_args()
    package_root = package_root_from_file()

    if args.list:
        for scenario in list_scenarios(package_root):
            print(
                "\t".join(
                    [
                        scenario["name"],
                        scenario.get("display_name", ""),
                        scenario.get("description", ""),
                    ]
                )
            )
        return 0

    if not args.scenario:
        raise SystemExit("--scenario is required unless --list is used")

    workspace_root = workspace_root_from_package(package_root)
    raw_name = args.scenario.strip()
    raw_scenario = load_scenario(
        package_root,
        raw_name,
        {
            "scenario": raw_name,
            "workspace_root": workspace_root,
            "package_root": package_root,
        },
    )
    runner = dict(raw_scenario["runner"])
    if args.platform:
        runner["platform"] = args.platform
    if args.environment:
        runner["environment"] = args.environment
    if args.suite:
        runner["suite"] = args.suite
    if args.uav_id is not None:
        runner["uav_id"] = args.uav_id

    variables = {
        "scenario": raw_scenario.get("name", raw_name),
        "display_name": raw_scenario.get("display_name", raw_name),
        "description": raw_scenario.get("description", ""),
        "platform": runner["platform"],
        "environment": runner["environment"],
        "suite": runner["suite"],
        "uav_id": runner["uav_id"],
        "uav_name": f"/uav{runner['uav_id']}",
        "workspace_root": workspace_root,
        "package_root": package_root,
    }
    scenario = load_scenario(package_root, raw_name, variables)
    runner = dict(scenario["runner"])
    runner_cli_args = _runner_cli_args(runner, args)
    variables["runner_cli"] = shlex.join(runner_cli_args)

    scenario = load_scenario(package_root, raw_name, variables)
    runner = dict(scenario["runner"])
    load_config_triplet(
        package_root=package_root,
        platform_name=str(runner["platform"]),
        environment_name=str(runner["environment"]),
        suite_name=str(runner["suite"]),
        uav_id=int(runner["uav_id"]),
    )
    terminal = _detect_terminal()

    for index, window in enumerate(scenario["windows"]):
        _launch_window(terminal, window)
        inter_window_delay_s = float(window.get("after_launch_delay_s", 0.2))
        if index < len(scenario["windows"]) - 1 and inter_window_delay_s > 0:
            time.sleep(inter_window_delay_s)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

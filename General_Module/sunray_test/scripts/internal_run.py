#!/usr/bin/env python3
import argparse
import os
import sys
from typing import Dict, List


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.core.runner import RunnerArgs, TestRunner
from sunray_test.core.suite_loader import load_yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Interactively choose and run a sunray test suite")
    parser.add_argument("--platform", default="sunray150_basic")
    parser.add_argument("--environment", default="sim")
    parser.add_argument("--suite", default="")
    parser.add_argument("--uav-id", type=int, default=1)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--list", action="store_true", help="List available suites and exit")
    return parser.parse_args()


def _suite_dir() -> str:
    return os.path.join(PACKAGE_ROOT, "config", "suites")


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


def main():
    try:
        args = parse_args()
        entries = _load_suite_entries()

        if args.list:
            _print_suite_list(entries)
            return 0

        suite_name = args.suite.strip() or _select_suite(entries)
        runner = TestRunner(
            RunnerArgs(
                platform=args.platform,
                environment=args.environment,
                suite=suite_name,
                uav_id=args.uav_id,
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

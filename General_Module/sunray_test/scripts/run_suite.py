#!/usr/bin/env python3
import argparse
import os
import sys


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.core.runner import RunnerArgs, TestRunner


def parse_args():
    parser = argparse.ArgumentParser(description="Run a sunray test suite")
    parser.add_argument("--platform", default="sunray150_basic")
    parser.add_argument("--environment", default="sim")
    parser.add_argument("--suite", default="basic_acceptance")
    parser.add_argument("--uav-id", type=int, default=1)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--sn", default="")
    parser.add_argument("--tester", default="")
    parser.add_argument("--no-prompt", action="store_true")
    return parser.parse_args()


def main():
    try:
        args = parse_args()
        runner = TestRunner(
            RunnerArgs(
                platform=args.platform,
                environment=args.environment,
                suite=args.suite,
                uav_id=args.uav_id,
                output_dir=args.output_dir,
                sn=args.sn,
                tester=args.tester,
                prompt_metadata=not args.no_prompt,
            )
        )
        raise SystemExit(runner.run())
    except KeyboardInterrupt:
        print("\n[sunray_test] 收到 Ctrl+C，测试已中断。", flush=True)
        raise SystemExit(130)


if __name__ == "__main__":
    main()

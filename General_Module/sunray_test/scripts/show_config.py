#!/usr/bin/env python3
import argparse
import json
import os
import sys

import yaml


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.core.suite_loader import show_effective_config


def parse_args():
    parser = argparse.ArgumentParser(description="Show merged and validated sunray test config")
    parser.add_argument("--platform", default="sunray150_basic")
    parser.add_argument("--environment", default="sim")
    parser.add_argument("--suite", default="basic_acceptance")
    parser.add_argument("--uav-id", type=int, default=1)
    parser.add_argument(
        "--section",
        default="all",
        choices=["all", "input", "platform", "environment", "suite", "defaults", "report", "topics", "recording", "missions"],
    )
    parser.add_argument("--format", default="yaml", choices=["yaml", "json"])
    return parser.parse_args()


def main():
    args = parse_args()
    effective = show_effective_config(
        package_root=PACKAGE_ROOT,
        platform_name=args.platform,
        environment_name=args.environment,
        suite_name=args.suite,
        uav_id=args.uav_id,
    )
    data = effective if args.section == "all" else effective[args.section]
    if args.format == "json":
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0
    print(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

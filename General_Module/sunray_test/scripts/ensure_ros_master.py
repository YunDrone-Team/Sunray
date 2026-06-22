#!/usr/bin/env python3
import os
import sys

import rosgraph


def master_is_online() -> bool:
    try:
        rosgraph.Master("/sunray_test_dashboard").getPid()
        return True
    except Exception:
        return False


def main() -> int:
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    if master_is_online():
        print(f"[sunray_test] ROS master already running: {master_uri}", flush=True)
        return 0

    print(f"[sunray_test] starting roscore: {master_uri}", flush=True)
    os.execvp("roscore", ["roscore"])
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

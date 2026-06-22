#!/usr/bin/env python3
"""Guard a V4L2 camera node startup before exec'ing the real ROS node."""

import argparse
import os
import shutil
import subprocess
import sys
import time
from typing import List, Sequence, Tuple


def _split_guard_and_command(argv: Sequence[str]) -> Tuple[List[str], List[str]]:
    if "--" not in argv:
        return list(argv), []
    split_index = list(argv).index("--")
    return list(argv[:split_index]), list(argv[split_index + 1 :])


def _run_quiet(
    command: Sequence[str],
    timeout_s: float,
    ignore_codes: Sequence[int] = (),
) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            list(command),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        print(f"[camera_guard] timeout: {' '.join(command)}", flush=True)
        return subprocess.CompletedProcess(list(command), 124, "", "timeout")


def _command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def _print_failure(label: str, result: subprocess.CompletedProcess, ignore_codes: Sequence[int]) -> None:
    if result.returncode == 0 or result.returncode in ignore_codes:
        return
    detail = (result.stderr or result.stdout or "").strip()
    if detail:
        detail = f": {detail.splitlines()[-1]}"
    print(f"[camera_guard] {label} failed rc={result.returncode}{detail}", flush=True)


def _kill_ros_nodes(node_names: Sequence[str], timeout_s: float) -> None:
    if not node_names or not _command_exists("rosnode"):
        return
    for node_name in node_names:
        if not node_name:
            continue
        result = _run_quiet(["rosnode", "kill", node_name], timeout_s, ignore_codes=(1,))
        _print_failure(f"rosnode kill {node_name}", result, ignore_codes=(1,))


def _release_devices(devices: Sequence[str], timeout_s: float, kill_delay_s: float) -> None:
    if not devices or not _command_exists("fuser"):
        return
    for device in devices:
        if not device or not os.path.exists(device):
            print(f"[camera_guard] device not found: {device}", flush=True)
            continue
        term = _run_quiet(["fuser", "-k", "-TERM", device], timeout_s, ignore_codes=(1,))
        _print_failure(f"fuser TERM {device}", term, ignore_codes=(1,))
        if term.returncode == 0 and kill_delay_s > 0:
            time.sleep(kill_delay_s)
        busy = _run_quiet(["fuser", device], timeout_s, ignore_codes=(1,))
        if busy.returncode == 0:
            kill = _run_quiet(["fuser", "-k", "-KILL", device], timeout_s, ignore_codes=(1,))
            _print_failure(f"fuser KILL {device}", kill, ignore_codes=(1,))


def _warmup_devices(devices: Sequence[str], frame_count: int, retries: int, retry_delay_s: float) -> None:
    if frame_count <= 0 or not devices or not _command_exists("v4l2-ctl"):
        return
    for device in devices:
        if not device or not os.path.exists(device):
            continue
        command = [
            "v4l2-ctl",
            "-d",
            device,
            "--stream-mmap=3",
            f"--stream-count={frame_count}",
            "--stream-to=/dev/null",
        ]
        for attempt in range(max(1, retries)):
            result = _run_quiet(command, timeout_s=max(3.0, frame_count / 5.0 + 2.0), ignore_codes=())
            if result.returncode == 0:
                break
            if attempt < retries - 1 and retry_delay_s > 0:
                time.sleep(retry_delay_s)
        else:
            _print_failure(f"warmup {device}", result, ignore_codes=())


def _validate_distinct_devices(devices: Sequence[str]) -> None:
    normalized = [os.path.realpath(device) for device in devices if device]
    if len(normalized) != len(set(normalized)):
        raise RuntimeError(f"duplicate camera device in launch arguments: {', '.join(devices)}")


def parse_args(argv: Sequence[str]) -> Tuple[argparse.Namespace, List[str]]:
    guard_args, command = _split_guard_and_command(argv)
    parser = argparse.ArgumentParser(description="Prepare a V4L2 camera device before starting a ROS camera node")
    parser.add_argument("--role", default="camera")
    parser.add_argument("--node-name", action="append", default=[])
    parser.add_argument("--device", action="append", default=[])
    parser.add_argument("--peer-device", action="append", default=[])
    parser.add_argument("--startup-delay", type=float, default=0.0)
    parser.add_argument("--post-clean-delay", type=float, default=0.4)
    parser.add_argument("--kill-timeout", type=float, default=1.5)
    parser.add_argument("--kill-delay", type=float, default=0.3)
    parser.add_argument("--warmup-frames", type=int, default=4)
    parser.add_argument("--warmup-retries", type=int, default=2)
    parser.add_argument("--warmup-retry-delay", type=float, default=0.5)
    parser.add_argument("--no-rosnode-kill", action="store_true")
    parser.add_argument("--no-fuser-kill", action="store_true")
    parser.add_argument("--no-warmup", action="store_true")
    return parser.parse_args(guard_args), command


def main(argv: Sequence[str]) -> int:
    args, command = parse_args(argv)
    if args.startup_delay > 0:
        time.sleep(args.startup_delay)

    devices = [str(device).strip() for device in args.device if str(device).strip()]
    peer_devices = [str(device).strip() for device in args.peer_device if str(device).strip()]
    node_names = [str(name).strip() for name in args.node_name if str(name).strip()]
    _validate_distinct_devices(devices + peer_devices)
    print(
        f"[camera_guard] preparing {args.role}: "
        f"nodes={node_names or '-'} devices={devices or '-'}",
        flush=True,
    )

    if not args.no_rosnode_kill:
        _kill_ros_nodes(node_names, timeout_s=max(0.2, args.kill_timeout))
    if not args.no_fuser_kill:
        _release_devices(devices, timeout_s=max(0.2, args.kill_timeout), kill_delay_s=max(0.0, args.kill_delay))
    if not args.no_warmup:
        _warmup_devices(
            devices,
            frame_count=max(0, args.warmup_frames),
            retries=max(1, args.warmup_retries),
            retry_delay_s=max(0.0, args.warmup_retry_delay),
        )
    if args.post_clean_delay > 0:
        time.sleep(args.post_clean_delay)

    if not command:
        return 0
    print(f"[camera_guard] starting {args.role}: {' '.join(command)}", flush=True)
    os.execvp(command[0], command)
    return 127


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

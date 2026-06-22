#!/usr/bin/env python3
import argparse
import os
import select
import sys
import termios
import tty
from typing import Dict, Tuple

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.tools.vrpn_config import (
    TARGETS,
    TARGET_BY_KEY,
    load_launch_contents,
    selected_contents,
    target_text,
    write_updates as write_vrpn_updates,
)


class UserQuit(Exception):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Inspect and optionally update the default server IP in "
            "VRPN-related launch files"
        )
    )
    parser.add_argument(
        "--target",
        choices=sorted(TARGET_BY_KEY),
        default="all",
        help="launch file target to check/update; default: all",
    )
    parser.add_argument(
        "--server",
        default="",
        help=(
            "set selected launch file(s) to this server IP without entering "
            "the interactive edit flow"
        ),
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip confirmation when used with --server",
    )
    return parser.parse_args()


def _confirm(message: str) -> bool:
    while True:
        answer = input(message).strip().lower()
        if not answer:
            return False
        if answer == "q":
            raise UserQuit
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        print("请输入 y、n、直接回车或 q。", flush=True)


def _prompt_ip(message: str) -> str:
    while True:
        value = input(message).strip()
        if value.lower() == "q":
            raise UserQuit
        if value:
            return value
        print("IP 不能为空。输入 q 可退出。", flush=True)


def _print_target_table(
    all_contents: Dict[str, Tuple[str, str]],
    selected_key: str = "all",
    clear: bool = False,
) -> None:
    if clear:
        print("\033[2J\033[H", end="")
    print("┌ CONFIG CANDIDATES", flush=True)
    print("│ SEL   ACTION  FILE                    CURRENT", flush=True)
    for target in TARGETS:
        selected = selected_key == target.key
        marker = "▶" if selected else " "
        checkbox = "[x]" if selected else "[ ]"
        servers = [all_contents.get(path, ("", "<missing>"))[1] for path in target.launch_files]
        current = " / ".join(servers)
        print(f"│ {marker} {checkbox}  UPDATE  {target.label:22s} {current}", flush=True)
    print("└", flush=True)
    print("↑/↓ 选择，Enter 确认，q 退出", flush=True)


def _read_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = os.read(fd, 1)
        if ch == b"\x1b":
            ready, _, _ = select.select([sys.stdin], [], [], 0.3)
            if not ready:
                return "escape"
            ch2 = os.read(fd, 1)
            if ch2 != b"[":
                return "escape"
            ready, _, _ = select.select([sys.stdin], [], [], 0.3)
            if not ready:
                return "escape"
            ch3 = os.read(fd, 1)
            if ch3 == b"A":
                return "up"
            if ch3 == b"B":
                return "down"
            return "escape"
        if ch in {b"\r", b"\n"}:
            return "enter"
        if ch in {b"\x03", b"\x04", b"q", b"Q"}:
            return "quit"
        return ch.decode(errors="ignore").lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _select_target_interactive(all_contents: Dict[str, Tuple[str, str]], default_key: str) -> str:
    selected_index = next(
        (index for index, target in enumerate(TARGETS) if target.key == default_key),
        len(TARGETS) - 1,
    )
    while True:
        selected_key = TARGETS[selected_index].key
        _print_target_table(all_contents, selected_key=selected_key, clear=True)
        key = _read_key()
        if key == "enter":
            return selected_key
        if key == "quit":
            raise UserQuit
        if key == "up":
            selected_index = (selected_index - 1) % len(TARGETS)
            continue
        if key == "down":
            selected_index = (selected_index + 1) % len(TARGETS)
            continue


def _print_selected_servers(contents: Dict[str, Tuple[str, str]]) -> None:
    print("当前选择：", flush=True)
    for launch_file, (_, server) in contents.items():
        print(f"- {os.path.basename(launch_file):22s} {server}", flush=True)


def _write_updates(contents: Dict[str, Tuple[str, str]], new_server: str) -> None:
    for launch_file in write_vrpn_updates(contents, new_server):
        print(f"已更新: {launch_file}", flush=True)
    print(f"server default 已修改为: {new_server}", flush=True)


def _target_text(contents: Dict[str, Tuple[str, str]]) -> str:
    return target_text(contents)


def main():
    args = parse_args()
    try:
        all_contents = load_launch_contents()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    try:
        target = args.target
        if not args.server and sys.stdin.isatty():
            target = _select_target_interactive(all_contents, args.target)
        else:
            _print_target_table(all_contents, selected_key=args.target)

        contents = selected_contents(all_contents, target)
        _print_selected_servers(contents)

        if args.server:
            should_update = args.yes or not sys.stdin.isatty()
            if not should_update:
                message = f"确认将 {_target_text(contents)} 修改为 {args.server}？[y/N/q]: "
                should_update = _confirm(message)
            if should_update:
                _write_updates(contents, args.server)
            else:
                print("未修改。", flush=True)
            return 0

        if not sys.stdin.isatty():
            return 0

        new_server = _prompt_ip("请输入新的 IP [q退出]: ")
        if _confirm(f"确认将 {_target_text(contents)} 修改为 {new_server}？[y/N/q]: "):
            _write_updates(contents, new_server)
        else:
            print("未修改。", flush=True)
        return 0
    except UserQuit:
        print("已退出。", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

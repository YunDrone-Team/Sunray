#!/usr/bin/env python3
import argparse
import os
import re
import select
import sys
import termios
import tty
from typing import Dict, List, NamedTuple, Tuple


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
SUNRAY_CONTROL_LAUNCH_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "sunray_uav_control", "launch")
)
DEFAULT_LAUNCH_FILES = [
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "external_fusion.launch"),
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "sunray_vrpn.launch"),
]


class Target(NamedTuple):
    key: str
    label: str
    launch_files: List[str]


TARGETS = [
    Target("external", "external_fusion.launch", [DEFAULT_LAUNCH_FILES[0]]),
    Target("vrpn", "sunray_vrpn.launch", [DEFAULT_LAUNCH_FILES[1]]),
    Target("all", "all", DEFAULT_LAUNCH_FILES),
]
TARGET_BY_KEY = {target.key: target for target in TARGETS}


class UserQuit(Exception):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect and optionally update the default server IP in VRPN-related launch files"
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
        help="set selected launch file(s) to this server IP without entering the interactive edit flow",
    )
    parser.add_argument("--yes", action="store_true", help="skip confirmation when used with --server")
    return parser.parse_args()


def _read_launch_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _write_launch_file(path: str, content: str) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(content)


def _extract_server(content: str) -> str:
    match = re.search(r'<arg\s+name="server"\s+default="([^"]+)"\s*/>', content)
    if not match:
        raise RuntimeError('cannot find <arg name="server" .../> in launch file')
    return match.group(1)


def _replace_server(content: str, new_server: str) -> str:
    pattern = r'(<arg\s+name="server"\s+default=")([^"]+)("\s*/>)'
    replacement = rf"\g<1>{new_server}\g<3>"
    new_content, count = re.subn(pattern, replacement, content, count=1)
    if count != 1:
        raise RuntimeError('failed to update <arg name="server" .../> in launch file')
    return new_content


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


def _load_launch_contents(launch_files: List[str]) -> Dict[str, Tuple[str, str]]:
    contents = {}
    for launch_file in launch_files:
        if not os.path.isfile(launch_file):
            raise RuntimeError(f"launch file not found: {launch_file}")
        content = _read_launch_file(launch_file)
        contents[launch_file] = (content, _extract_server(content))
    return contents


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
    for launch_file, (content, _) in contents.items():
        updated = _replace_server(content, new_server)
        _write_launch_file(launch_file, updated)
        print(f"已更新: {launch_file}", flush=True)
    print(f"server default 已修改为: {new_server}", flush=True)


def _target_text(contents: Dict[str, Tuple[str, str]]) -> str:
    return "、".join(os.path.basename(path) for path in contents)


def main():
    args = parse_args()
    try:
        all_contents = _load_launch_contents(DEFAULT_LAUNCH_FILES)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    try:
        target = args.target
        if not args.server and sys.stdin.isatty():
            target = _select_target_interactive(all_contents, args.target)
        else:
            _print_target_table(all_contents, selected_key=args.target)

        contents = {
            launch_file: all_contents[launch_file]
            for launch_file in TARGET_BY_KEY[target].launch_files
        }
        _print_selected_servers(contents)

        if args.server:
            should_update = args.yes or not sys.stdin.isatty()
            if not should_update:
                should_update = _confirm(f"确认将 {_target_text(contents)} 修改为 {args.server}？[y/N/q]: ")
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

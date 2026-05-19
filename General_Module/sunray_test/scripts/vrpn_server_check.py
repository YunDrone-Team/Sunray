#!/usr/bin/env python3
import argparse
import os
import re
import sys


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
SUNRAY_CONTROL_LAUNCH_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "sunray_uav_control", "launch")
)
DEFAULT_LAUNCH_FILES = [
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "external_fusion.launch"),
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "sunray_vrpn.launch"),
]


class UserQuit(Exception):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect and optionally update the default server IP in VRPN-related launch files"
    )
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


def _prompt_yes_no(message: str) -> bool:
    while True:
        answer = input(message).strip().lower()
        if answer == "q":
            raise UserQuit
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        print("请输入 y、n 或 q。", flush=True)


def _prompt_ip(message: str) -> str:
    while True:
        value = input(message).strip()
        if value.lower() == "q":
            raise UserQuit
        if value:
            return value
        print("IP 不能为空。输入 q 可退出。", flush=True)


def _select_launch_files() -> list:
    launch_info = []
    for launch_file in DEFAULT_LAUNCH_FILES:
        if not os.path.isfile(launch_file):
            launch_info.append((launch_file, "<missing>"))
            continue
        content = _read_launch_file(launch_file)
        launch_info.append((launch_file, _extract_server(content)))

    options = {
        "1": [DEFAULT_LAUNCH_FILES[0]],
        "2": [DEFAULT_LAUNCH_FILES[1]],
        "3": DEFAULT_LAUNCH_FILES,
    }
    print("请选择要修改的文件：", flush=True)
    print(f"1) external_fusion.launch  当前IP={launch_info[0][1]}", flush=True)
    print(f"2) sunray_vrpn.launch      当前IP={launch_info[1][1]}", flush=True)
    print("3) all", flush=True)
    print("q) 退出", flush=True)

    while True:
        choice = input("请输入编号 [1/2/3/q]: ").strip().lower()
        if choice == "q":
            return []
        if choice in options:
            return options[choice]
        print("请输入 1、2、3 或 q。", flush=True)


def main():
    parse_args()
    launch_files = _select_launch_files()
    if not launch_files:
        print("已退出。", flush=True)
        return 0

    contents = {}
    for launch_file in launch_files:
        if not os.path.isfile(launch_file):
            print(f"launch file not found: {launch_file}", file=sys.stderr)
            return 1
        content = _read_launch_file(launch_file)
        current_server = _extract_server(content)
        contents[launch_file] = content
        print(f"{os.path.basename(launch_file)} 当前 server IP: {current_server}", flush=True)

    try:
        if _prompt_yes_no("是否确认当前 IP？[y/n/q]: "):
            return 0

        new_server = _prompt_ip("请输入新的 IP [q退出]: ")
        while True:
            print(f"你输入的 IP 是: {new_server}", flush=True)
            if _prompt_yes_no("是否确认修改为该 IP？[y/n/q]: "):
                for launch_file, content in contents.items():
                    updated = _replace_server(content, new_server)
                    _write_launch_file(launch_file, updated)
                    print(f"已更新: {launch_file}", flush=True)
                print(f"server default 已修改为: {new_server}", flush=True)
                return 0
            new_server = _prompt_ip("请重新输入新的 IP [q退出]: ")
    except UserQuit:
        print("已退出。", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

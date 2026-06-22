import os
import select
import shlex
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union

from sunray_test.tools.livox_config import update_lidar_ip
from sunray_test.tools.vrpn_config import (
    TARGETS,
    load_launch_contents,
    selected_contents,
    target_text,
    write_updates,
)


@dataclass(frozen=True)
class DashboardTool:
    tool_id: str
    title: str
    command: Tuple[str, ...]
    embedded: bool = True


DASHBOARD_TOOLS = (
    DashboardTool(
        tool_id="livox_mid360",
        title="雷达IP自动配置",
        command=("rosrun", "sunray_test", "livox_mid360_autoconfig.py"),
    ),
    DashboardTool(
        tool_id="vrpn_server",
        title="VRPN 服务器检查",
        command=("rosrun", "sunray_test", "vrpn_server_check.py"),
    ),
)
TOOL_BY_ID = {tool.tool_id: tool for tool in DASHBOARD_TOOLS}

VRPN_MODE_SELECT = "select"
VRPN_MODE_INPUT = "input"
VRPN_MODE_CONFIRM = "confirm"


@dataclass
class VrpnToolState:
    target_index: int = 2
    mode: str = VRPN_MODE_SELECT
    input_value: str = ""
    status: str = ""
    error: str = ""
    contents: Dict[str, tuple] = field(default_factory=dict)

    @property
    def target_key(self) -> str:
        return TARGETS[self.target_index].key

    def refresh(self) -> None:
        try:
            self.contents = load_launch_contents()
            self.error = ""
        except RuntimeError as exc:
            self.contents = {}
            self.error = str(exc)

    def move_target(self, delta: int) -> None:
        if self.mode != VRPN_MODE_SELECT:
            return
        self.target_index = (self.target_index + delta) % len(TARGETS)

    def begin_input(self) -> None:
        if self.error:
            return
        self.mode = VRPN_MODE_INPUT
        self.input_value = ""
        self.status = ""

    def backspace_input(self) -> None:
        if self.mode == VRPN_MODE_INPUT:
            self.input_value = self.input_value[:-1]

    def append_input(self, value: str) -> None:
        if self.mode != VRPN_MODE_INPUT:
            return
        if value and all(char in "0123456789." for char in value):
            self.input_value += value

    def confirm_input(self) -> None:
        if self.mode != VRPN_MODE_INPUT:
            return
        if not self.input_value.strip():
            self.status = "IP 不能为空"
            return
        self.mode = VRPN_MODE_CONFIRM
        self.status = ""

    def cancel(self) -> bool:
        if self.mode == VRPN_MODE_CONFIRM:
            self.mode = VRPN_MODE_INPUT
            return False
        if self.mode == VRPN_MODE_INPUT:
            self.mode = VRPN_MODE_SELECT
            self.input_value = ""
            self.status = ""
            return False
        return True

    def apply(self) -> None:
        if self.mode != VRPN_MODE_CONFIRM:
            return
        try:
            all_contents = load_launch_contents()
            contents = selected_contents(all_contents, self.target_key)
            updated_paths = write_updates(contents, self.input_value.strip())
            self.contents = load_launch_contents()
            self.mode = VRPN_MODE_SELECT
            self.input_value = ""
            self.error = ""
            self.status = "已更新: " + target_text({path: ("", "") for path in updated_paths})
        except RuntimeError as exc:
            self.error = str(exc)


@dataclass
class ProcessToolState:
    title: str
    command: Union[str, Sequence[str]]
    logs: List[str] = field(default_factory=list)
    process: subprocess.Popen = None
    status: str = ""
    scroll: int = 0
    follow_tail: bool = True

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, extra_args: Sequence[str] = ()) -> None:
        if self.running:
            self.status = "任务正在运行"
            return
        self.logs = [f"$ {self.command_text(extra_args)}"]
        self.scroll = 0
        self.follow_tail = True
        self.status = "运行中"
        self.process = subprocess.Popen(
            self.command_argv(extra_args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            cwd=os.getcwd(),
        )

    def command_argv(self, extra_args: Sequence[str] = ()) -> List[str]:
        if isinstance(self.command, str):
            argv = shlex.split(self.command)
        else:
            argv = [str(part) for part in self.command]
        return [*argv, *(str(arg) for arg in extra_args)]

    def command_text(self, extra_args: Sequence[str] = ()) -> str:
        return shlex.join(self.command_argv(extra_args))

    def stop(self) -> None:
        if not self.running:
            return
        self.process.terminate()
        self.status = "已请求停止"

    def cleanup(self) -> None:
        if not self.running:
            self.process = None
            return
        self.stop()
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process = None

    def poll(self) -> None:
        if self.process is None or self.process.stdout is None:
            return
        while True:
            ready, _, _ = select.select([self.process.stdout], [], [], 0)
            if not ready:
                break
            line = self.process.stdout.readline()
            if not line:
                break
            self.append_log(line.rstrip())
        status = self.process.poll()
        if status is not None:
            remainder = self.process.stdout.read()
            if remainder:
                for line in remainder.splitlines():
                    self.append_log(line)
            self.status = f"已结束: {status}"
            self.process = None

    def move_scroll(self, delta: int) -> None:
        self.follow_tail = False
        self.scroll = max(0, self.scroll + delta)

    def follow_latest(self) -> None:
        self.follow_tail = True

    def append_log(self, line: str) -> None:
        self.logs.append(line)
        if self.follow_tail:
            self.scroll = max(0, len(self.logs))

    def result_fields(self) -> Tuple[List[Tuple[str, str]], List[str]]:
        fields: Dict[str, str] = {}
        order: List[str] = []
        errors = []
        for line in self.logs:
            if line.startswith("ERROR:"):
                errors.append(line)
                continue
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            if key in MID360_RESULT_KEYS:
                if key not in fields:
                    order.append(key)
                fields[key] = normalize_mid360_result_value(key, value.strip())
        return [(MID360_RESULT_KEYS[key], fields[key]) for key in order], errors[-3:]


@dataclass
class Mid360ToolState(ProcessToolState):
    update_declined: bool = False
    update_done: bool = False

    def start(self, extra_args: Sequence[str] = ()) -> None:
        self.update_declined = False
        self.update_done = False
        super().start(extra_args=extra_args)

    def apply_update(self) -> None:
        lidar_ip = self.detected_lidar_ip()
        if not lidar_ip:
            self.status = "未检测到雷达 IP，无法更新"
            return

        targets = self.updatable_config_paths()
        if not targets:
            self.status = "没有可更新的配置文件"
            return

        updated = []
        for path in targets:
            try:
                update_lidar_ip(path, lidar_ip)
                updated.append(str(path))
                self.append_log(f"updated: {path}")
                self.append_log(f"config_lidar_ip: {lidar_ip}")
                self.append_log("config_status:   match")
            except (OSError, ValueError, TypeError) as exc:
                self.append_log(f"ERROR: update failed {path}: {exc}")

        if updated:
            self.update_done = True
            self.status = "已更新: " + ", ".join(updated)
        else:
            self.status = "更新失败"

    def skip_update(self) -> None:
        self.update_declined = True
        self.status = "已跳过更新"

    @property
    def update_available(self) -> bool:
        if self.running or self.update_declined or self.update_done:
            return False
        return bool(self.detected_lidar_ip() and self.updatable_config_paths())

    def config_status_values(self) -> List[str]:
        return [state["status"] for state in self.config_states() if state.get("status")]

    def detected_lidar_ip(self) -> str:
        return self.latest_log_value("lidar_ip")

    def latest_log_value(self, field: str) -> str:
        prefix = f"{field}:"
        for line in reversed(self.logs):
            if not line.startswith(prefix):
                continue
            value = line.split(":", 1)[1].strip()
            return "" if value in {"", "N/A"} else value
        return ""

    def config_states(self) -> List[Dict[str, str]]:
        states: List[Dict[str, str]] = []
        current: Dict[str, str] = {}
        for line in self.logs:
            if line.startswith("config:"):
                if current:
                    states.append(current)
                current = {"path": line.split(":", 1)[1].strip()}
            elif line.startswith("config_lidar_ip:") and current:
                current["configured_ip"] = line.split(":", 1)[1].strip()
            elif line.startswith("config_status:") and current:
                current["status"] = line.split(":", 1)[1].strip()
        if current:
            states.append(current)
        return states

    def updatable_config_paths(self) -> List[Path]:
        paths = []
        for state in self.config_states():
            status = state.get("status", "")
            if not (status == "missing_lidar_ip" or status.startswith("mismatch")):
                continue
            path = Path(state.get("path", "")).expanduser()
            if path.is_file():
                paths.append(path)
        return paths

MID360_RESULT_KEYS = {
    "iface": "网卡",
    "iface_ip": "网卡IP",
    "lidar_ip": "雷达IP",
    "broadcast_code": "SN",
    "arp_host_ip": "ARP主机IP",
    "discovery_pkts": "发现包",
    "detect_method": "检测方式",
    "config": "配置文件",
    "config_lidar_ip": "配置雷达IP",
    "config_status": "配置状态",
}

MID360_STATUS_TEXT = {
    "match": "匹配",
    "unavailable": "不可用",
    "missing_file": "配置文件不存在",
    "missing_lidar_ip": "未配置雷达IP",
}


def normalize_mid360_result_value(key: str, value: str) -> str:
    value = value or "N/A"
    if key != "config_status":
        return value
    if value.startswith("mismatch "):
        return "不一致 " + value[len("mismatch ") :]
    return MID360_STATUS_TEXT.get(value, value)


@dataclass
class EmbeddedToolsState:
    vrpn: VrpnToolState = field(default_factory=VrpnToolState)
    mid360: Mid360ToolState = field(
        default_factory=lambda: Mid360ToolState(
            title=TOOL_BY_ID["livox_mid360"].title,
            command=TOOL_BY_ID["livox_mid360"].command,
        )
    )

    def refresh(self) -> None:
        self.vrpn.refresh()
        self.mid360.poll()

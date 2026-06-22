from dataclasses import dataclass, field
from typing import List

from sunray_test.dashboard.tools import EmbeddedToolsState, VRPN_MODE_CONFIRM
from sunray_test.dashboard.text_format import display_width, pad_cells
from sunray_test.dashboard.tui_state import (
    PANE_FUNCTION,
    PANE_HARDWARE,
    PANE_TOOLS,
    TuiState,
)
from sunray_test.tools.vrpn_config import TARGETS


@dataclass
class ToolView:
    title: str
    lines: List[str]
    scroll: int = 0
    result_title: str = ""
    result_lines: List[str] = field(default_factory=list)


def handle_tool_key(state: TuiState, tools_state: EmbeddedToolsState, key_name: str) -> None:
    if not state.tool_active:
        state.enter_tool()
        if state.active_tool_id == "livox_mid360":
            tools_state.mid360.start()
        return
    handler = TOOL_KEY_HANDLERS.get(state.active_tool_id)
    if handler:
        handler(state, tools_state, key_name)


def handle_vrpn_tool_key(
    state: TuiState,
    tools_state: EmbeddedToolsState,
    key_name: str,
) -> None:
    vrpn = tools_state.vrpn
    if key_name == "esc":
        if vrpn.cancel():
            state.exit_tool()
    elif key_name == "up":
        vrpn.move_target(-1)
    elif key_name == "down":
        vrpn.move_target(1)
    elif key_name == "enter":
        if vrpn.mode == VRPN_MODE_CONFIRM:
            vrpn.apply()
        else:
            vrpn.confirm_input()
    elif key_name in {" ", "space"}:
        vrpn.begin_input()
    elif key_name == "backspace":
        vrpn.backspace_input()
    elif len(key_name) == 1:
        vrpn.append_input(key_name)


def handle_mid360_tool_key(
    state: TuiState,
    tools_state: EmbeddedToolsState,
    key_name: str,
) -> None:
    mid360 = tools_state.mid360
    if key_name == "esc":
        state.exit_tool()
    elif key_name in {"y", "Y"}:
        if mid360.update_available:
            mid360.apply_update()
    elif key_name in {"n", "N"}:
        if mid360.update_available:
            mid360.skip_update()
    elif key_name == "enter":
        mid360.start()
    elif key_name in {" ", "space"}:
        mid360.stop()
    elif key_name == "up":
        mid360.move_scroll(-1)
    elif key_name == "down":
        mid360.move_scroll(1)


def build_tool_view(
    state: TuiState,
    tools_state: EmbeddedToolsState,
    visible_height: int,
) -> ToolView:
    if state.active_tool_id == "vrpn_server":
        return vrpn_tool_view(tools_state)
    if state.active_tool_id == "livox_mid360":
        return mid360_tool_view(tools_state, visible_height)
    return ToolView(title="工具", lines=["未知工具"])


def vrpn_tool_view(tools_state: EmbeddedToolsState) -> ToolView:
    vrpn = tools_state.vrpn
    lines = vrpn_table_lines(vrpn)
    lines.append("")
    if vrpn.mode == "input":
        lines.append(f"新 server IP: {vrpn.input_value}")
        lines.append("输入数字和点，Enter 确认，Esc 返回")
    elif vrpn.mode == "confirm":
        lines.append(f"确认写入 {vrpn.input_value} ?")
        lines.append("Enter 写入，Esc 返回输入")
    else:
        lines.append("↑/↓ 选择目标，Space 输入新 IP")
    if vrpn.status:
        lines.extend(["", vrpn.status])
    if vrpn.error:
        lines.extend(["", "ERROR: " + vrpn.error])
    return ToolView(title="VRPN 服务器检查", lines=lines)


def mid360_tool_view(
    tools_state: EmbeddedToolsState,
    visible_height: int,
) -> ToolView:
    mid360 = tools_state.mid360
    lines = [
        *(mid360.logs or ["启动中..."]),
    ]
    if mid360.follow_tail:
        mid360.scroll = max(0, len(lines) - max(0, visible_height))
    return ToolView(
        title=mid360.title,
        lines=lines,
        scroll=mid360.scroll,
        result_title="检测结果",
        result_lines=mid360_result_lines(mid360),
    )


def vrpn_table_lines(vrpn) -> List[str]:
    if not vrpn.contents:
        return ["未读取到 VRPN launch 配置"]
    lines = ["目标        当前 server"]
    for index, target in enumerate(TARGETS):
        marker = ">" if index == vrpn.target_index else " "
        servers = [
            vrpn.contents.get(path, ("", "<missing>"))[1]
            for path in target.launch_files
        ]
        lines.append(f"{marker} {target.label:22s} {' / '.join(servers)}")
    return lines


def mid360_result_lines(mid360) -> List[str]:
    fields, errors = mid360.result_fields()
    lines = [format_mid360_result_line("状态", mid360.status or "未运行")]
    if fields:
        lines.append("")
        lines.extend(format_mid360_result_line(label, value) for label, value in fields)
    elif mid360.running:
        lines.extend(["", "等待检测结果"])
    else:
        lines.extend(["", "尚未启动"])
    if errors:
        lines.append("")
        lines.extend(errors)
    if getattr(mid360, "update_available", False):
        lines.extend(["", "检测到配置可更新", "按 Y 写入配置，按 N 跳过"])
    return lines


def format_mid360_result_line(label: str, value: str) -> str:
    return f"{pad_cells(label, 12)}: {value}"


def footer_help_line(state: TuiState, width: int = 0) -> str:
    if state.external_source_selector:
        return shortcut_line(*EXTERNAL_SOURCE_SHORTCUTS, width=width)
    if state.tool_active:
        return shortcut_line(*TOOL_SHORTCUTS.get(state.active_tool_id, DEFAULT_TOOL_SHORTCUTS), width=width)
    if state.editing_params:
        return shortcut_line(*PARAM_SHORTCUTS, width=width)
    if state.active_pane in {PANE_FUNCTION, PANE_HARDWARE}:
        return shortcut_line(*ITEM_SHORTCUTS, width=width)
    if state.active_pane == PANE_TOOLS:
        return shortcut_line(*TOOLS_PANE_SHORTCUTS, width=width)
    return shortcut_line(*DEFAULT_SHORTCUTS, width=width)


def shortcut_line(*parts: str, width: int = 0) -> str:
    items = [part for part in parts if part]
    if not items:
        return ""
    if width <= 0:
        return "   ".join(items)
    visible = list(items)
    gap_width = 2
    while visible:
        line = (" " * gap_width).join(visible)
        if display_width(line) <= width:
            return line
        visible.pop()
    return ""


TOOL_KEY_HANDLERS = {
    "vrpn_server": handle_vrpn_tool_key,
    "livox_mid360": handle_mid360_tool_key,
}

EXTERNAL_SOURCE_SHORTCUTS = ("[UP/DOWN]:选择", "[ENTER]:确认", "[TAB/ESC]:返回")
PARAM_SHORTCUTS = (
    "[LEFT/RIGHT]:调整",
    "[UP/DOWN]:移动",
    "[ENTER]:输入",
    "[BACKSPACE]:默认",
    "[TAB/ESC]:返回",
)
ITEM_SHORTCUTS = (
    "[LEFT/RIGHT]:切换区域",
    "[UP/DOWN]:选择",
    "[SPACE]:勾选",
    "[TAB]:参数",
    "[M]:机型",
    "[E]:定位源",
    "[A]:全选",
    "[C]:清空",
    "[ENTER]:启动",
    "[Q]:退出",
)
TOOLS_PANE_SHORTCUTS = (
    "[LEFT/RIGHT]:切换区域",
    "[UP/DOWN]:选择工具",
    "[M]:机型",
    "[E]:定位源",
    "[ENTER]:打开工具",
    "[Q]:退出",
)
TOOL_SHORTCUTS = {
    "vrpn_server": (
        "[UP/DOWN]:选择",
        "[SPACE]:输入",
        "[ENTER]:确认/写入",
        "[TAB/ESC]:返回",
    ),
    "livox_mid360": (
        "[ENTER]:启动",
        "[SPACE]:停止",
        "[Y/N]:更新/跳过",
        "[UP/DOWN]:滚动日志",
        "[TAB/ESC]:返回",
    ),
}
DEFAULT_TOOL_SHORTCUTS = ("[TAB/ESC]:返回",)
DEFAULT_SHORTCUTS = ("[LEFT/RIGHT]:切换区域", "[Q]:退出")

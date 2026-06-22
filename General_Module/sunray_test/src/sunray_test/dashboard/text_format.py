import os
import textwrap
import unicodedata
from typing import Dict, List, Sequence

from sunray_test.dashboard.types import PACKAGE_ROOT, TestItem


def trim_text(text: str, width: int) -> str:
    if width <= 0:
        return ""
    if display_width(text) <= width:
        return text
    if width <= 3:
        return trim_cells(text, width)
    return trim_cells(text, width - 3) + "..."


def format_item_row(
    cursor_marker: str,
    selection_marker: str,
    item: TestItem,
    suffix: str,
    width: int,
    item_id_width: int = None,
) -> str:
    prefix = f"{cursor_marker} [{selection_marker}] "
    name = item.name
    suffix_text = f" {suffix}" if suffix else ""
    fixed_width = display_width(prefix) + display_width(suffix_text) + 1
    available = max(8, width - fixed_width)
    if item_id_width is None:
        item_id_width = display_width(item.item_id)
    item_id_width = clamp_int(item_id_width, 8, min(18, available))
    name_width = max(4, available - item_id_width - 1)
    return (
        f"{prefix}"
        f"{pad_cells(trim_text(item.item_id, item_id_width), item_id_width)} "
        f"{pad_cells(trim_text(name, name_width), name_width)}"
        f"{suffix_text}"
    )


def format_item_short_tags(item: TestItem, auto_selected: bool) -> str:
    tags: List[str] = []
    if auto_selected:
        tags.append("自动")
    if item.sim_only:
        tags.append("仿真")
    if item.exp_only:
        tags.append("实机")
    return "/".join(tags)


def format_param_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".")
    return "-" if value is None else str(value)


def param_detail_lines(spec: Dict[str, object], width: int) -> List[str]:
    path = str(spec.get("path", ""))
    param_type = str(spec.get("type", "string"))
    default = format_param_value(spec.get("default"))
    options = spec.get("options", [])
    bounds = []
    if spec.get("min") is not None:
        bounds.append(f"min={spec['min']}")
    if spec.get("max") is not None:
        bounds.append(f"max={spec['max']}")
    if spec.get("step") is not None:
        bounds.append(f"step={spec['step']}")
    meta = f"path={path}  type={param_type}  default={default}"
    if options:
        meta += "  options=" + ",".join(str(option) for option in options)
    if bounds:
        meta += "  " + "  ".join(bounds)
    lines = [meta]
    description = str(spec.get("description", "")).strip()
    if description:
        lines.extend(wrap_lines(description, width))
    return lines


def format_display_path(path: str) -> str:
    if not path:
        return "-"
    absolute_path = os.path.abspath(path)
    workspace_root = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", ".."))
    try:
        relative_path = os.path.relpath(absolute_path, workspace_root)
    except ValueError:
        return absolute_path
    if relative_path == ".":
        return "."
    if not relative_path.startswith(".."):
        return relative_path
    return absolute_path


def format_display_text(text: str) -> str:
    workspace_root = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", ".."))
    return text.replace(workspace_root + os.sep, "")


def item_id_column_width(items: Sequence[TestItem], width: int) -> int:
    if not items:
        return 8
    max_id_width = max(display_width(item.item_id) for item in items)
    max_reasonable = max(8, min(18, width // 2))
    return clamp_int(max_id_width, 8, max_reasonable)


def split_body_heights(body_height: int) -> tuple:
    pane_rows = max(16, body_height - 1)
    top = clamp_int(int(pane_rows * 0.32), 8, 10)
    preview = pane_rows - top
    if preview < 7:
        top -= min(7 - preview, max(0, top - 6))
        preview = pane_rows - top
    return top, max(7, preview)


def split_preview_widths(width: int) -> tuple:
    step_width = clamp_int(width // 3, 28, width - 42)
    return step_width, width - step_width - 1


def split_top_widths(width: int) -> tuple:
    available = max(0, width - 2)
    base_width = available // 3
    remainder = available % 3
    widths = [
        base_width + (1 if index < remainder else 0)
        for index in range(3)
    ]
    return tuple(widths)


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


def display_width(text: str) -> int:
    width = 0
    for char in text:
        if unicodedata.combining(char):
            continue
        width += 2 if unicodedata.east_asian_width(char) in {"F", "W"} else 1
    return width


def trim_cells(text: str, width: int) -> str:
    if width <= 0:
        return ""
    used = 0
    result: List[str] = []
    for char in text:
        char_width = 0 if unicodedata.combining(char) else (
            2 if unicodedata.east_asian_width(char) in {"F", "W"} else 1
        )
        if used + char_width > width:
            break
        result.append(char)
        used += char_width
    return "".join(result)


def pad_cells(text: str, width: int) -> str:
    padding = max(0, width - display_width(text))
    return text + (" " * padding)


def wrap_lines(text: str, width: int) -> List[str]:
    return textwrap.wrap(text, max(10, width)) or [""]


def expand_wrapped(lines: Sequence[str], width: int) -> List[str]:
    wrapped: List[str] = []
    for line in lines:
        if not line:
            wrapped.append("")
            continue
        wrapped.extend(wrap_lines(line, width))
    return wrapped


def format_tab_lines(tabs: Sequence[Dict[str, object]]) -> List[str]:
    if not tabs:
        return [" - none"]
    lines: List[str] = []
    for index, tab in enumerate(tabs, 1):
        delay_s = float(tab.get("delay_s", 0.0))
        delay = f" delay={delay_s:.1f}s" if delay_s > 0 else ""
        command = format_display_text(str(tab.get("command", "")))
        lines.append(f" {index}. {tab.get('title', 'tab')}{delay}: {command}")
    return lines

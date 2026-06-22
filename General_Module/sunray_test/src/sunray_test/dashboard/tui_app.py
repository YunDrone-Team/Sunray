import curses
import os
import sys
from typing import Dict, List, Sequence

from sunray_test.dashboard.text_format import (
    clamp_int,
    display_width,
    expand_wrapped,
    format_display_path,
    format_display_text,
    format_item_row,
    format_item_short_tags,
    format_param_value,
    format_tab_lines,
    item_id_column_width,
    pad_cells,
    param_detail_lines,
    split_body_heights,
    split_preview_widths,
    split_top_widths,
    trim_text,
)
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest, DashboardSession
from sunray_test.dashboard.suite_runtime import build_bringup_tabs, build_runner_tabs
from sunray_test.dashboard.tools import EmbeddedToolsState
from sunray_test.dashboard.tui_tools import (
    build_tool_view,
    footer_help_line,
    handle_tool_key,
)
from sunray_test.dashboard.types import DASHBOARD_UAV_ID, DashboardPlan, TestItem
from sunray_test.tools.vrpn_config import DEFAULT_LAUNCH_FILES, extract_server, read_launch_file
from sunray_test.dashboard.tui_state import (
    ACTION_CANCEL,
    ACTION_NONE,
    ACTION_START,
    ACTION_TOOL,
    PANE_FUNCTION,
    PANE_HARDWARE,
    PANE_PREVIEW,
    PANE_TOOLS,
    TuiState,
)


TUI_MIN_WIDTH = 80
TUI_MIN_HEIGHT = 24
TUI_PREFERRED_WIDTH = 132
TUI_PREFERRED_HEIGHT = 38


def run_dashboard_tui(
    session: DashboardSession,
    args,
    build_plan_callback,
    build_suite_path_callback,
):
    if not can_use_curses():
        return None
    request_terminal_size(TUI_PREFERRED_HEIGHT, TUI_PREFERRED_WIDTH)

    state = TuiState(
        model=session.model,
        environment=session.environment,
        uav_id=session.uav_id,
        external_source_override=args.external_source,
        profile_override=args.profile,
        record_rosbag=args.record_rosbag,
        continue_on_failure=args.continue_on_failure,
        no_bringup=args.no_bringup,
    )

    def runner(screen):
        return TuiApp(
            screen=screen,
            state=state,
            args=args,
            output_dir=session.output_dir,
            build_plan_callback=build_plan_callback,
            build_suite_path_callback=build_suite_path_callback,
        ).run()

    return curses.wrapper(runner)


def can_use_curses() -> bool:
    if not os.isatty(0) or not os.isatty(1):
        return False
    term = os.environ.get("TERM", "")
    return bool(term and term != "dumb")


def request_terminal_size(rows: int, cols: int) -> None:
    if os.environ.get("SUNRAY_TEST_TUI_NO_RESIZE"):
        return
    if not os.isatty(1):
        return
    term = os.environ.get("TERM", "")
    if not term or term == "dumb":
        return
    try:
        sys.stdout.write(f"\033[8;{rows};{cols}t")
        sys.stdout.flush()
    except OSError:
        pass


def external_fusion_vrpn_ip() -> str:
    try:
        return extract_server(read_launch_file(DEFAULT_LAUNCH_FILES[0]))
    except (OSError, RuntimeError):
        return ""


def profile_display_label(profile: str) -> str:
    labels = {
        "sunray150_basic": "basic",
        "sunray150_lidar": "lidar",
    }
    return labels.get(profile, profile or "-")


class TuiApp:
    def __init__(
        self,
        screen,
        state: TuiState,
        args,
        output_dir: str,
        build_plan_callback,
        build_suite_path_callback,
    ) -> None:
        self.screen = screen
        self.state = state
        self.args = args
        self.output_dir = output_dir
        self.build_plan_callback = build_plan_callback
        self.build_suite_path_callback = build_suite_path_callback
        self.colors: Dict[str, int] = {}
        self.plan: DashboardPlan = None
        self.plan_error = ""
        self.suite_path = ""
        self.runner_tabs: List[Dict[str, object]] = []
        self.bringup_tabs: List[Dict[str, object]] = []
        self.tools_state = EmbeddedToolsState()
        self.plan_cache_key = None

    def run(self):
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.screen.keypad(True)
        self.screen.timeout(200)
        self.init_colors()
        while True:
            self.tools_state.refresh()
            self.state.flush_pending_external_source_key()
            self.refresh_plan()
            self.draw()
            key_name = read_key_name(self.screen)
            if not key_name:
                continue
            if key_name == "enter" and self.state.editing_params:
                self.prompt_current_param_value()
                continue
            action = self.state.handle_key_name(key_name)
            if action == ACTION_NONE:
                continue
            if action == ACTION_CANCEL:
                self.cleanup_tools()
                return TuiResult(action=ACTION_CANCEL)
            if action == ACTION_TOOL:
                handle_tool_key(self.state, self.tools_state, key_name)
                continue
            if action == ACTION_START:
                if self.plan is not None:
                    self.cleanup_tools()
                    return self.result(ACTION_START)
                self.state.status = self.plan_error or "未选择任何测试项目"

    def result(self, action: str):
        return TuiResult(
            action=action,
            request=self.state.request(),
            plan=self.plan,
            suite_path=self.suite_path,
        )

    def init_colors(self) -> None:
        if not curses.has_colors():
            return
        curses.start_color()
        curses.use_default_colors()
        definitions = {
            "title": (curses.COLOR_CYAN, -1),
            "active": (curses.COLOR_BLACK, curses.COLOR_CYAN),
            "selected": (curses.COLOR_GREEN, -1),
            "auto": (curses.COLOR_YELLOW, -1),
            "muted": (curses.COLOR_BLUE, -1),
            "error": (curses.COLOR_RED, -1),
            "border": (curses.COLOR_BLUE, -1),
            "dialog": (curses.COLOR_WHITE, curses.COLOR_BLACK),
            "footer": (curses.COLOR_BLACK, curses.COLOR_CYAN),
        }
        for index, (name, colors) in enumerate(definitions.items(), start=1):
            curses.init_pair(index, colors[0], colors[1])
            self.colors[name] = curses.color_pair(index)

    def color(self, name: str) -> int:
        return self.colors.get(name, 0)

    def refresh_plan(self) -> None:
        cache_key = self.state.plan_cache_key()
        if cache_key == self.plan_cache_key:
            return
        self.plan_cache_key = cache_key
        if not self.state.requested_item_ids:
            self.plan = None
            self.plan_error = "请选择至少一个硬件或功能测试项目"
            self.suite_path = ""
            self.runner_tabs = []
            self.bringup_tabs = []
            return
        try:
            self.plan = self.build_plan_callback(self.state.request())
            self.plan_error = getattr(self.plan, "validation_warning", "")
            self.suite_path = self.build_suite_path_callback(self.plan)
            self.bringup_tabs = build_bringup_tabs(self.state.model, self.plan)
            self.runner_tabs = build_runner_tabs(
                self.state.model,
                self.args,
                self.plan,
                self.suite_path,
                self.output_dir,
            )
        except SystemExit as exc:
            self.plan = None
            self.plan_error = str(exc)
            self.suite_path = ""
            self.runner_tabs = []
            self.bringup_tabs = []

    def cleanup_tools(self) -> None:
        self.tools_state.mid360.cleanup()

    def draw(self) -> None:
        if self.state.needs_full_redraw:
            self.screen.clear()
            self.state.needs_full_redraw = False
        else:
            self.screen.erase()
        height, width = self.screen.getmaxyx()
        if height < TUI_MIN_HEIGHT or width < TUI_MIN_WIDTH:
            self.draw_too_small(height, width)
            self.screen.refresh()
            return

        self.draw_header(width)
        self.draw_body(height, width)
        if self.state.external_source_selector:
            self.draw_external_source_selector(height, width)
        self.draw_footer(height, width)
        self.screen.refresh()

    def draw_too_small(self, height: int, width: int) -> None:
        message = (
            f"terminal too small: {width}x{height}, need {TUI_MIN_WIDTH}x{TUI_MIN_HEIGHT}; "
            f"preferred {TUI_PREFERRED_WIDTH}x{TUI_PREFERRED_HEIGHT}"
        )
        safe_addstr(self.screen, 0, 0, message[: max(0, width - 1)], self.color("error"))

    def draw_header(self, width: int) -> None:
        title = " Sunray Test Dashboard "
        safe_addstr(self.screen, 0, 1, title, self.color("title") | curses.A_BOLD)
        profile = self.plan.profile if self.plan else "-"
        profile_label = profile_display_label(profile)
        profile_source = "manual" if self.state.profile_override else "auto"
        profile_text = f"{profile_label}({profile_source})"
        source = "sim" if self.state.environment == "sim" else "exp"
        external_source = self.plan.external_source if self.plan else self.state.current_external_source()
        external_label = (
            self.plan.external_source_label
            if self.plan
            else str(self.state.current_external_source_option().get("label", external_source))
        )
        header_primary = (
            f" ENV:{source}  UAV:{DASHBOARD_UAV_ID}  "
            f"OUTPUT:{format_display_path(self.output_dir)}"
        )
        header_secondary = (
            f" PLATFORM:{profile_text}  "
            f"EXTERNAL_SOURCE:{external_label}({external_source})"
        )
        vrpn_ip = external_fusion_vrpn_ip() if int(external_source) == 3 else ""
        if vrpn_ip:
            header_secondary += f"  VRPN_IP:{vrpn_ip}"
        safe_addstr(self.screen, 1, 1, trim_text(header_primary, width - 2), self.color("muted"))
        safe_addstr(self.screen, 2, 1, trim_text(header_secondary, width - 2), self.color("muted"))

    def draw_body(self, height: int, width: int) -> None:
        top = 4
        bottom = height - 2
        body_height = bottom - top
        if self.state.tool_active:
            self.draw_embedded_tool(
                y=top,
                x=0,
                height=body_height,
                width=width,
            )
            return

        top_height, preview_height = split_body_heights(body_height)
        function_width, hardware_width, tools_width = split_top_widths(width)
        hardware_x = function_width + 1
        tools_x = hardware_x + hardware_width + 1
        preview_y = top + top_height + 1

        self.draw_item_pane(
            y=top,
            x=0,
            height=top_height,
            width=function_width,
            title="功能测试",
            pane=PANE_FUNCTION,
            items=self.state.function_items,
            selected_ids=self.state.selected_functions,
            cursor=self.state.function_cursor,
        )
        self.draw_item_pane(
            y=top,
            x=hardware_x,
            height=top_height,
            width=hardware_width,
            title="硬件测试",
            pane=PANE_HARDWARE,
            items=self.state.hardware_items,
            selected_ids=self.state.selected_hardware,
            cursor=self.state.hardware_cursor,
        )
        self.draw_tools_pane(
            y=top,
            x=tools_x,
            height=top_height,
            width=tools_width,
        )

        if self.state.editing_params:
            self.draw_param_editor(
                y=preview_y,
                x=0,
                height=preview_height,
                width=width,
            )
            return

        self.draw_preview_grid(preview_y, 0, preview_height, width)

    def draw_preview_grid(self, y: int, x: int, height: int, width: int) -> None:
        step_width, right_width = split_preview_widths(width)
        right_x = x + step_width + 1

        self.draw_steps_pane(y, x, height, step_width)
        self.draw_bringup_pane(y, right_x, height, right_width)

    def draw_item_pane(
        self,
        y: int,
        x: int,
        height: int,
        width: int,
        title: str,
        pane: str,
        items: Sequence[TestItem],
        selected_ids: Sequence[str],
        cursor: int,
    ) -> None:
        active = self.state.active_pane == pane
        draw_box(self.screen, y, x, height, width, title, active, self.color("active"))
        auto_added = set(self.plan.selection.auto_added_item_ids if self.plan else [])
        visible_height = height - 2
        item_id_width = item_id_column_width(items, width - 2)
        start = visible_start(cursor, visible_height, len(items))
        for row, item in enumerate(items[start : start + visible_height]):
            item_index = start + row
            is_selected = item.item_id in selected_ids or item.item_id in auto_added
            marker = "x" if is_selected else " "
            cursor_marker = ">" if active and item_index == cursor else " "
            auto_selected = item.item_id in auto_added
            suffix = format_item_short_tags(item, auto_selected)
            if auto_selected:
                marker = "a"
            line = format_item_row(
                cursor_marker,
                marker,
                item,
                suffix,
                width - 2,
                item_id_width=item_id_width,
            )
            attr = self.color("selected") if is_selected else 0
            if auto_selected:
                attr = self.color("auto")
            if active and item_index == cursor:
                attr |= curses.A_REVERSE
            safe_addstr(self.screen, y + 1 + row, x + 1, trim_text(line, width - 2), attr)

    def draw_tools_pane(self, y: int, x: int, height: int, width: int) -> None:
        active = self.state.active_pane == PANE_TOOLS
        draw_box(self.screen, y, x, height, width, "工具", active, self.color("active"))
        visible_height = height - 2
        for row, tool in enumerate(self.state.tools[:visible_height]):
            marker = ">" if active and row == self.state.tool_cursor else " "
            line = f"{marker} [ ] {tool.title}"
            line = pad_cells(trim_text(line, width - 2), width - 2)
            attr = curses.A_REVERSE if active and row == self.state.tool_cursor else 0
            safe_addstr(self.screen, y + 1 + row, x + 1, line, attr)

    def draw_embedded_tool(self, y: int, x: int, height: int, width: int) -> None:
        view = build_tool_view(self.state, self.tools_state, max(0, height - 2))
        if view.result_lines:
            log_width = max(20, (width * 2) // 3)
            result_width = max(20, width - log_width - 1)
            if log_width + result_width + 1 > width:
                log_width = max(20, width - result_width - 1)
            self.draw_text_pane(
                y=y,
                x=x,
                height=height,
                width=log_width,
                title=view.title,
                lines=expand_wrapped(view.lines, log_width - 2),
                active=True,
                scroll=view.scroll,
            )
            self.draw_text_pane(
                y=y,
                x=x + log_width + 1,
                height=height,
                width=result_width,
                title=view.result_title or "检测结果",
                lines=expand_wrapped(view.result_lines, result_width - 2),
                active=True,
            )
            return
        self.draw_text_pane(
            y=y,
            x=x,
            height=height,
            width=width,
            title=view.title,
            lines=expand_wrapped(view.lines, width - 2),
            active=True,
            scroll=view.scroll,
        )

    def draw_steps_pane(self, y: int, x: int, height: int, width: int) -> None:
        active = self.state.active_pane == PANE_PREVIEW
        self.draw_text_pane(
            y=y,
            x=x,
            height=height,
            width=width,
            title="测试步骤",
            lines=self.steps_lines(width - 2),
            active=active,
        )

    def draw_bringup_pane(self, y: int, x: int, height: int, width: int) -> None:
        active = self.state.active_pane == PANE_PREVIEW
        self.draw_text_pane(
            y=y,
            x=x,
            height=height,
            width=width,
            title="启动链路",
            lines=self.bringup_lines(width - 2),
            active=active,
        )

    def draw_param_editor(self, y: int, x: int, height: int, width: int) -> None:
        item = self.state.model.item_by_id.get(self.state.editing_params_for)
        title = f"参数设置 - {item.name if item else self.state.editing_params_for}"
        draw_box(self.screen, y, x, height, width, title, True, self.color("active"))
        specs = self.state.current_param_specs()
        if not specs:
            safe_addstr(self.screen, y + 1, x + 1, "当前测试项没有可编辑参数")
            return

        table_height = max(3, height - 7)
        name_width = 20
        value_width = 16
        unit_width = 8
        header = (
            f"{pad_cells('参数', name_width)} "
            f"{pad_cells('当前值', value_width)} "
            f"{pad_cells('单位', unit_width)} 说明"
        )
        safe_addstr(self.screen, y + 1, x + 1, trim_text(header, width - 2), self.color("muted"))
        max_start = max(0, len(specs) - table_height)
        start = clamp_int(self.state.param_cursor - table_height + 1, 0, max_start)
        visible_specs = specs[start : start + table_height]
        for row, spec in enumerate(visible_specs):
            spec_index = start + row
            current = self.state.get_param_value(self.state.editing_params_for, spec)
            default = spec.get("default")
            changed = current != default
            cursor = ">" if spec_index == self.state.param_cursor else " "
            name = str(spec.get("name", spec.get("path", "")))
            value = format_param_value(current)
            if changed:
                value += " *"
            line = (
                f"{cursor} "
                f"{pad_cells(trim_text(name, name_width), name_width)} "
                f"{pad_cells(trim_text(value, value_width), value_width)} "
                f"{pad_cells(trim_text(str(spec.get('unit', '-')), unit_width), unit_width)} "
                f"{spec.get('description', '')}"
            )
            attr = curses.A_REVERSE if spec_index == self.state.param_cursor else 0
            safe_addstr(self.screen, y + 2 + row, x + 1, trim_text(line, width - 2), attr)

        spec = self.state.current_param_spec()
        detail_y = y + height - 5
        if spec:
            details = param_detail_lines(spec, width - 2)
            for offset, line in enumerate(details[:3]):
                safe_addstr(
                    self.screen,
                    detail_y + offset,
                    x + 1,
                    trim_text(line, width - 2),
                    self.color("muted"),
                )

    def draw_external_source_selector(self, height: int, width: int) -> None:
        options = self.state.external_source_options()
        if not options:
            return
        box_width = min(64, max(42, width - 8))
        box_height = min(len(options) + 4, max(8, height - 6))
        y = max(2, (height - box_height) // 2)
        x = max(0, (width - box_width) // 2)
        dialog_attr = self.color("dialog")
        fill_rect(self.screen, y, x, box_height, box_width, dialog_attr)
        draw_box(self.screen, y, x, box_height, box_width, "external_source", True, self.color("active"))
        safe_addstr(
            self.screen,
            y + 1,
            x + 2,
            pad_cells(trim_text("↑/↓ 选择  Enter 确认  Esc/Tab 返回", box_width - 4), box_width - 4),
            dialog_attr,
        )
        visible_height = max(1, box_height - 3)
        start = clamp_int(
            self.state.external_source_cursor - visible_height + 1,
            0,
            max(0, len(options) - visible_height),
        )
        for row, option in enumerate(options[start : start + visible_height]):
            index = start + row
            label = str(option["label"])
            value = int(option["value"])
            desc = str(option.get("description", ""))
            marker = ">" if index == self.state.external_source_cursor else " "
            line = f"{marker} {pad_cells(label, 12)} {value:<2} {desc}"
            attr = self.color("active") if index == self.state.external_source_cursor else dialog_attr
            safe_addstr(
                self.screen,
                y + 2 + row,
                x + 2,
                pad_cells(trim_text(line, box_width - 4), box_width - 4),
                attr,
            )

    def draw_text_pane(
        self,
        y: int,
        x: int,
        height: int,
        width: int,
        title: str,
        lines: Sequence[str],
        active: bool,
        scroll: int = 0,
    ) -> None:
        draw_box(self.screen, y, x, height, width, title, active, self.color("active"))
        visible_height = max(0, height - 2)
        max_scroll = max(0, len(lines) - visible_height)
        start = clamp_int(scroll, 0, max_scroll)
        visible = lines[start : start + visible_height]
        for row in range(visible_height):
            safe_addstr(self.screen, y + 1 + row, x + 1, " " * max(0, width - 2))
        for row, line in enumerate(visible):
            attr = self.color("error") if line.startswith("ERROR") else 0
            safe_addstr(
                self.screen,
                y + 1 + row,
                x + 1,
                pad_cells(trim_text(line, width - 2), width - 2),
                attr,
            )

    def steps_lines(self, width: int) -> List[str]:
        if self.plan is None:
            return ["-"]
        lines: List[str] = []
        for index, step in enumerate(self.plan.suite["steps"], 1):
            if "phase" in step:
                lines.append(f" {index}. phase {step['phase']}")
            else:
                lines.append(f" {index}. case {step['case']} ({step.get('name', step['case'])})")
        return expand_wrapped(lines, width)

    def bringup_lines(self, width: int) -> List[str]:
        if self.plan is None:
            return ["-"]
        if self.state.no_bringup:
            return [" - disabled (--no-bringup)"]
        return expand_wrapped(format_tab_lines(self.bringup_tabs), width)

    def draw_footer(self, height: int, width: int) -> None:
        status = self.state.status or self.plan_error
        if status:
            safe_addstr(
                self.screen,
                height - 2,
                1,
                trim_text(status, width - 2),
                self.color("muted"),
            )
        help_width = max(0, width - 2)
        help_line = footer_help_line(self.state, help_width)
        footer_attr = self.color("footer") or self.color("muted")
        safe_addstr(self.screen, height - 1, 0, " " * max(0, width - 1), footer_attr)
        safe_addstr(
            self.screen,
            height - 1,
            1,
            pad_cells(help_line, help_width),
            footer_attr,
        )

    def prompt_current_param_value(self) -> None:
        spec = self.state.current_param_spec()
        if not spec:
            return
        height, width = self.screen.getmaxyx()
        dialog_width = min(74, max(44, width - 8))
        dialog_height = 8
        y = max(2, (height - dialog_height) // 2)
        x = max(0, (width - dialog_width) // 2)
        dialog_attr = self.color("dialog")
        name = str(spec.get("name", spec.get("path", "")))
        current = self.state.get_param_value(self.state.editing_params_for, spec)
        fill_rect(self.screen, y, x, dialog_height, dialog_width, dialog_attr)
        draw_box(self.screen, y, x, dialog_height, dialog_width, "输入参数", True, self.color("active"))
        safe_addstr(
            self.screen,
            y + 1,
            x + 2,
            pad_cells(trim_text(f"{name}", dialog_width - 4), dialog_width - 4),
            dialog_attr,
        )
        safe_addstr(
            self.screen,
            y + 2,
            x + 2,
            pad_cells(trim_text(f"当前值: {format_param_value(current)}", dialog_width - 4), dialog_width - 4),
            dialog_attr,
        )
        safe_addstr(
            self.screen,
            y + 5,
            x + 2,
            pad_cells(trim_text("Enter 确认，Esc 取消", dialog_width - 4), dialog_width - 4),
            dialog_attr,
        )
        input_y = y + 4
        input_x = x + 2
        input_width = max(1, dialog_width - 4)
        safe_addstr(self.screen, input_y, input_x, " " * input_width, self.color("active"))
        self.screen.refresh()
        value = self.read_dialog_input(input_y, input_x, input_width, dialog_attr)
        if value is not None:
            if value:
                try:
                    self.state.set_param_value(self.state.editing_params_for, spec, value)
                    self.state.status = ""
                except (TypeError, ValueError) as exc:
                    self.state.status = f"参数输入无效: {exc}"
        self.state.needs_full_redraw = True

    def read_dialog_input(self, y: int, x: int, width: int, attr: int) -> str:
        text = ""
        try:
            curses.curs_set(1)
        except curses.error:
            pass
        try:
            while True:
                visible = trim_text(text, width)
                safe_addstr(self.screen, y, x, pad_cells(visible, width), self.color("active") or attr)
                self.screen.move(y, x + min(width - 1, display_width(visible)))
                self.screen.refresh()
                key_name = read_key_name(self.screen)
                if key_name in {"enter", "\n", "\r"}:
                    return text.strip()
                if key_name in {"esc"}:
                    return None
                if key_name in {"backspace"}:
                    text = text[:-1]
                    continue
                if len(key_name) == 1 and key_name.isprintable():
                    text += key_name
        except (curses.error, UnicodeDecodeError, ValueError) as exc:
            self.state.status = f"参数输入无效: {exc}"
            return None
        finally:
            try:
                curses.curs_set(0)
            except curses.error:
                pass


class TuiResult:
    def __init__(
        self,
        action: str,
        request: DashboardRequest = None,
        plan: DashboardPlan = None,
        suite_path: str = "",
    ) -> None:
        self.action = action
        self.request = request
        self.plan = plan
        self.suite_path = suite_path


def read_key_name(screen) -> str:
    key = screen.getch()
    if key in (curses.KEY_ENTER, 10, 13):
        return "enter"
    if key in (curses.KEY_BACKSPACE, 8, 127):
        return "backspace"
    if key in (9,):
        return "tab"
    if key in (27,):
        return "esc"
    if key == curses.KEY_BTAB:
        return "shift_tab"
    if key == curses.KEY_UP:
        return "up"
    if key == curses.KEY_DOWN:
        return "down"
    if key == curses.KEY_LEFT:
        return "left"
    if key == curses.KEY_RIGHT:
        return "right"
    if key == curses.KEY_NPAGE:
        return "page_down"
    if key == curses.KEY_PPAGE:
        return "page_up"
    if 0 <= key < 256:
        return chr(key)
    return ""


def draw_box(
    screen,
    y: int,
    x: int,
    height: int,
    width: int,
    title: str,
    active: bool,
    attr: int,
) -> None:
    border_attr = attr if active else 0
    try:
        screen.attron(border_attr)
        for col in range(x + 1, x + width - 1):
            safe_addch(screen, y, col, curses.ACS_HLINE, border_attr)
            safe_addch(screen, y + height - 1, col, curses.ACS_HLINE, border_attr)
        for row in range(y + 1, y + height - 1):
            safe_addch(screen, row, x, curses.ACS_VLINE, border_attr)
            safe_addch(screen, row, x + width - 1, curses.ACS_VLINE, border_attr)
        safe_addch(screen, y, x, curses.ACS_ULCORNER, border_attr)
        safe_addch(screen, y, x + width - 1, curses.ACS_URCORNER, border_attr)
        safe_addch(screen, y + height - 1, x, curses.ACS_LLCORNER, border_attr)
        safe_addch(screen, y + height - 1, x + width - 1, curses.ACS_LRCORNER, border_attr)
    finally:
        if border_attr:
            screen.attroff(border_attr)
    safe_addstr(screen, y, x + 2, f" {title} ", border_attr | curses.A_BOLD)


def safe_addstr(screen, y: int, x: int, text: str, attr: int = 0) -> None:
    try:
        screen.addstr(y, x, text, attr)
    except curses.error:
        pass


def fill_rect(screen, y: int, x: int, height: int, width: int, attr: int = 0) -> None:
    line = " " * max(0, width)
    for row in range(y, y + max(0, height)):
        safe_addstr(screen, row, x, line, attr)


def visible_start(cursor: int, visible_height: int, total: int) -> int:
    if visible_height <= 0 or total <= visible_height:
        return 0
    return clamp_int(cursor - visible_height + 1, 0, max(0, total - visible_height))


def safe_addch(screen, y: int, x: int, ch, attr: int = 0) -> None:
    try:
        screen.addch(y, x, ch, attr)
    except curses.error:
        pass

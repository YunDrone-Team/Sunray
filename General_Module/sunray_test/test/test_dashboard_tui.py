#!/usr/bin/env python3
import os
import sys
import unittest
import inspect
from argparse import Namespace
from io import StringIO
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.tui_app import (
    TuiApp,
    TUI_PREFERRED_HEIGHT,
    TUI_PREFERRED_WIDTH,
    can_use_curses,
    external_fusion_vrpn_ip,
    clamp_int,
    display_width,
    format_display_path,
    format_display_text,
    format_item_row,
    format_item_short_tags,
    format_tab_lines,
    item_id_column_width,
    pad_cells,
    request_terminal_size,
    split_body_heights,
    split_preview_widths,
    split_top_widths,
    trim_text,
    visible_start,
)
from sunray_test.dashboard.tui_tools import footer_help_line, shortcut_line
from sunray_test.dashboard.tui_state import (
    ACTION_CANCEL,
    ACTION_NONE,
    ACTION_START,
    ACTION_TOOL,
    PANE_FUNCTION,
    PANE_HARDWARE,
    PANE_PARAMS,
    PANE_PREVIEW,
    PANE_TOOLS,
    TuiState,
)


class DashboardTuiStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = DashboardModel.load("dashboard")

    def make_state(self, environment="exp"):
        return TuiState(
            model=self.model,
            environment=environment,
            uav_id=1,
            record_rosbag=True,
            continue_on_failure=False,
        )

    def test_default_state_selects_battery_only(self):
        state = self.make_state()

        self.assertEqual(state.active_pane, PANE_FUNCTION)
        self.assertEqual(state.selected_hardware, ["battery"])
        self.assertEqual(state.selected_functions, [])
        self.assertEqual(state.requested_item_ids, ["battery"])

    def test_space_toggles_current_hardware_item(self):
        state = self.make_state()
        state.active_pane = PANE_HARDWARE

        self.assertEqual(state.hardware_items[state.hardware_cursor].item_id, "front_camera")
        self.assertEqual(state.handle_key_name("space"), ACTION_NONE)
        self.assertIn("front_camera", state.selected_hardware)
        self.assertEqual(state.handle_key_name("space"), ACTION_NONE)
        self.assertNotIn("front_camera", state.selected_hardware)

    def test_arrows_switch_panes(self):
        state = self.make_state()

        state.handle_key_name("right")
        self.assertEqual(state.active_pane, PANE_HARDWARE)
        state.handle_key_name("right")
        self.assertEqual(state.active_pane, PANE_TOOLS)
        state.handle_key_name("right")
        self.assertEqual(state.active_pane, PANE_FUNCTION)
        state.handle_key_name("left")
        self.assertEqual(state.active_pane, PANE_TOOLS)

    def test_tab_enters_and_exits_param_editor(self):
        state = self.make_state()
        self.assertEqual(state.function_items[state.function_cursor].item_id, "hover")

        state.handle_key_name("tab")
        self.assertEqual(state.active_pane, PANE_PARAMS)
        self.assertEqual(state.editing_params_for, "hover")
        self.assertTrue(state.current_param_specs())
        state.handle_key_name("tab")
        self.assertEqual(state.active_pane, PANE_FUNCTION)

    def test_vim_keys_are_not_tui_shortcuts(self):
        state = self.make_state()

        state.handle_key_name("j")
        self.assertEqual(state.function_cursor, 0)
        state.handle_key_name("k")
        self.assertEqual(state.function_cursor, 0)
        self.assertEqual(state.handle_key_name("w"), ACTION_NONE)

    def test_tool_pane_uses_enter_to_launch_tool_action(self):
        state = self.make_state()
        state.active_pane = PANE_TOOLS

        self.assertEqual(state.current_tool().tool_id, "livox_mid360")
        self.assertEqual(state.handle_key_name("enter"), ACTION_TOOL)
        state.enter_tool()
        self.assertTrue(state.tool_active)
        self.assertEqual(state.handle_key_name("1"), ACTION_TOOL)
        self.assertEqual(state.handle_key_name("tab"), ACTION_NONE)
        self.assertFalse(state.tool_active)
        self.assertEqual(state.active_pane, PANE_TOOLS)

    def test_tool_cursor_moves_with_arrow_keys(self):
        state = self.make_state()
        state.active_pane = PANE_TOOLS

        state.handle_key_name("down")
        self.assertEqual(state.current_tool().tool_id, "vrpn_server")
        state.handle_key_name("down")
        self.assertEqual(state.current_tool().tool_id, "livox_mid360")
        state.handle_key_name("up")
        self.assertEqual(state.current_tool().tool_id, "vrpn_server")

    def test_item_cursor_moves_in_loop(self):
        state = self.make_state()
        state.active_pane = PANE_FUNCTION

        state.handle_key_name("up")
        self.assertEqual(state.function_cursor, len(state.function_items) - 1)
        state.handle_key_name("down")
        self.assertEqual(state.function_cursor, 0)

    def test_escape_exits_active_tool_via_tool_action(self):
        state = self.make_state()
        state.active_pane = PANE_TOOLS
        state.enter_tool()

        self.assertEqual(state.handle_key_name("esc"), ACTION_TOOL)
        state.exit_tool()
        self.assertFalse(state.tool_active)

    def test_footer_help_is_scoped_to_active_pane(self):
        state = self.make_state()

        state.active_pane = PANE_FUNCTION
        function_help = footer_help_line(state)
        self.assertIn("[LEFT/RIGHT]:切换区域", function_help)
        self.assertIn("[UP/DOWN]:选择", function_help)
        self.assertIn("[SPACE]:勾选", function_help)
        self.assertIn("[A]:全选", function_help)
        self.assertNotIn("打开工具", function_help)

        state.active_pane = PANE_TOOLS
        tool_help = footer_help_line(state)
        self.assertIn("[ENTER]:打开工具", tool_help)
        self.assertNotIn("[A]:全选", tool_help)
        self.assertNotIn("[SPACE]:勾选", tool_help)

    def test_shortcut_line_uses_consistent_spacing(self):
        self.assertEqual(shortcut_line("[A]:全选", "[Q]:退出"), "[A]:全选   [Q]:退出")
        self.assertEqual(shortcut_line("[A]:全选", "[Q]:退出", width=24), "[A]:全选  [Q]:退出")
        self.assertEqual(shortcut_line("[A]:全选", "[Q]:退出", width=12), "[A]:全选")

    def test_item_row_pads_chinese_name_column(self):
        item = self.model.item_by_id["ego_goal"]
        line = format_item_row(">", " ", item, "", 24, item_id_width=8)

        self.assertLessEqual(display_width(line), 24)
        self.assertTrue(line.startswith("> [ ] ego_goal"))

    def test_function_selection_auto_adds_required_hardware_in_plan(self):
        state = self.make_state(environment="sim")
        state.active_pane = PANE_FUNCTION
        state.function_cursor = 1

        self.assertEqual(state.function_items[state.function_cursor].item_id, "ego_goal")
        state.handle_key_name("space")
        plan = state.build_plan()

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.selection.auto_added_item_ids, ["lidar"])
        self.assertEqual(plan.selection.item_ids, ["battery", "lidar", "ego_goal"])

    def test_profile_key_cycles_between_auto_basic_and_lidar(self):
        state = self.make_state(environment="sim")

        self.assertEqual(state.profile_override, "")
        self.assertEqual(state.profile_override_label(), "自动")
        state.handle_key_name("m")
        self.assertEqual(state.profile_override, "sunray150_basic")
        self.assertEqual(state.profile_override_label(), "基础款")
        self.assertEqual(state.status, "")
        state.handle_key_name("m")
        self.assertEqual(state.profile_override, "sunray150_lidar")
        self.assertEqual(state.profile_override_label(), "雷达款")
        self.assertEqual(state.status, "")
        state.handle_key_name("m")
        self.assertEqual(state.profile_override, "")
        self.assertEqual(state.status, "")

    def test_lidar_profile_override_keeps_hover_plan_on_lidar_platform(self):
        state = self.make_state(environment="sim")
        state.selected_functions = ["hover"]
        state.profile_override = "sunray150_lidar"

        plan = state.build_plan()

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.profile_reason, "user")
        self.assertFalse(plan.runtime_state["condition_context"]["uses_lidar"])

    def test_param_editor_adjusts_and_resets_values(self):
        state = self.make_state()
        state.handle_key_name("tab")

        self.assertEqual(state.editing_params_for, "hover")
        spec = state.current_param_spec()
        self.assertEqual(spec["path"], "duration_s")
        state.handle_key_name("right")
        self.assertEqual(state.param_overrides["hover"]["duration_s"], 65.0)
        plan = state.build_plan()
        hover_step = [step for step in plan.suite["steps"] if step.get("case") == "hover_stability"]
        self.assertEqual(hover_step, [])

        state.selected_functions = ["hover"]
        plan = state.build_plan()
        hover_step = [
            step
            for step in plan.suite["steps"]
            if step.get("case") == "hover_stability"
        ][0]
        self.assertEqual(hover_step["params"]["duration_s"], 65.0)

        state.handle_key_name("backspace")
        self.assertNotIn("hover", state.param_overrides)

    def test_preview_scroll_uses_arrow_keys_when_preview_active(self):
        state = self.make_state()
        state.active_pane = PANE_PREVIEW

        state.handle_key_name("down")
        self.assertEqual(state.preview_scroll, 1)
        state.handle_key_name("up")
        self.assertEqual(state.preview_scroll, 0)

    def test_group_shortcuts_select_all_and_clear(self):
        state = self.make_state()
        state.active_pane = PANE_HARDWARE

        state.handle_key_name("a")
        self.assertEqual(
            state.selected_hardware,
            ["front_camera", "down_camera", "battery", "lidar"],
        )
        state.handle_key_name("c")
        self.assertEqual(state.selected_hardware, [])

    def test_action_keys_return_runner_actions(self):
        state = self.make_state()

        self.assertEqual(state.handle_key_name("enter"), ACTION_START)
        self.assertEqual(state.handle_key_name("q"), ACTION_CANCEL)

    def test_suite_key_is_not_a_tui_shortcut(self):
        state = self.make_state()

        self.assertEqual(state.handle_key_name("s"), ACTION_NONE)
        self.assertEqual(state.active_pane, PANE_FUNCTION)

    def test_footer_status_does_not_use_error_color(self):
        source = inspect.getsource(TuiApp.draw_footer)

        self.assertNotIn('self.color("error")', source)

    def test_active_tool_uses_full_body_before_top_grid(self):
        source = inspect.getsource(TuiApp.draw_body)

        self.assertLess(
            source.index("if self.state.tool_active"),
            source.index("split_body_heights"),
        )

    def test_external_source_short_press_toggles_odom_and_mocap(self):
        state = self.make_state()

        self.assertEqual(state.current_external_source(), 3)
        self.assertEqual(state.current_external_source_option()["label"], "MOCAP")
        self.assertEqual(state.handle_key_name("e"), ACTION_NONE)
        state.flush_pending_external_source_key(force=True)
        self.assertEqual(state.current_external_source(), 0)
        self.assertEqual(state.request().external_source_override, 0)

        plan = state.build_plan()
        self.assertEqual(plan.external_source, 0)
        self.assertEqual(plan.external_source_label, "ODOM")

    def test_lidar_profile_updates_default_external_source_until_user_override(self):
        state = self.make_state()

        self.assertEqual(state.current_external_source(), 3)
        state.selected_functions = ["ego_goal"]
        self.assertEqual(state.current_profile(), "sunray150_lidar")
        self.assertEqual(state.current_external_source(), 0)
        self.assertEqual(state.current_external_source_option()["label"], "ODOM")

        state.set_external_source(3)
        self.assertEqual(state.current_external_source(), 3)
        self.assertEqual(state.request().external_source_override, 3)

        plan = state.build_plan()
        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.external_source, 3)
        self.assertEqual(plan.external_source_label, "MOCAP")

    def test_external_source_repeat_e_opens_selector_without_quick_toggle(self):
        state = self.make_state()

        state.handle_key_name("e")
        state.handle_key_name("e")
        state.handle_key_name("e")

        self.assertTrue(state.external_source_selector)
        self.assertEqual(state.current_external_source(), 3)

    def test_external_source_selector_confirms_selected_value(self):
        state = self.make_state()

        state.open_external_source_selector()
        state.handle_key_name("up")
        state.handle_key_name("enter")

        self.assertFalse(state.external_source_selector)
        self.assertEqual(state.current_external_source(), 2)
        self.assertEqual(state.current_external_source_option()["label"], "GAZEBO")

    def test_can_use_curses_requires_real_tty_and_term(self):
        with patch("sunray_test.dashboard.tui_app.os.isatty", return_value=False):
            self.assertFalse(can_use_curses())
        with patch("sunray_test.dashboard.tui_app.os.isatty", return_value=True), patch.dict(
            os.environ,
            {"TERM": "dumb"},
            clear=False,
        ):
            self.assertFalse(can_use_curses())
        with patch("sunray_test.dashboard.tui_app.os.isatty", return_value=True), patch.dict(
            os.environ,
            {"TERM": "xterm-256color"},
            clear=False,
        ):
            self.assertTrue(can_use_curses())

    def test_trim_text_uses_ascii_ellipsis(self):
        self.assertEqual(trim_text("abcdefghij", 6), "abc...")
        self.assertEqual(trim_text("abc", 6), "abc")
        self.assertLessEqual(display_width(trim_text("视觉降落测试", 8)), 8)

    def test_pad_cells_handles_wide_characters(self):
        self.assertEqual(display_width(pad_cells("电池", 8)), 8)

    def test_clamp_int_bounds_values(self):
        self.assertEqual(clamp_int(10, 20, 30), 20)
        self.assertEqual(clamp_int(40, 20, 30), 30)
        self.assertEqual(clamp_int(25, 20, 30), 25)

    def test_display_path_uses_workspace_relative_path(self):
        output_dir = os.path.join(PACKAGE_ROOT, "..", "..", "tests", "output")

        self.assertEqual(format_display_path(output_dir), os.path.join("tests", "output"))

    def test_display_text_hides_workspace_prefix(self):
        workspace_root = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", ".."))
        suite_path = os.path.join(workspace_root, "tests", "output", "a.yaml")
        command = f"rosrun sunray_test run.py --suite-file {suite_path}"

        self.assertEqual(
            format_display_text(command),
            "rosrun sunray_test run.py --suite-file tests/output/a.yaml",
        )

    def test_split_body_heights_reserves_preview_grid(self):
        top, preview = split_body_heights(18)
        self.assertEqual(top + preview + 1, 18)
        self.assertGreaterEqual(top, 6)
        self.assertGreaterEqual(preview, 7)

        top, preview = split_body_heights(32)
        self.assertEqual(top + preview + 1, 32)
        self.assertGreater(preview, top)

    def test_split_preview_grid_sizes(self):
        step_width, right_width = split_preview_widths(132)
        self.assertEqual(step_width + right_width + 1, 132)
        self.assertGreaterEqual(step_width, 40)
        self.assertLessEqual(step_width, 46)

    def test_split_top_grid_sizes(self):
        function_width, hardware_width, tools_width = split_top_widths(132)
        self.assertEqual(function_width + hardware_width + tools_width + 2, 132)
        self.assertLessEqual(
            max(function_width, hardware_width, tools_width)
            - min(function_width, hardware_width, tools_width),
            1,
        )

    def test_visible_start_keeps_cursor_visible(self):
        self.assertEqual(visible_start(cursor=0, visible_height=4, total=10), 0)
        self.assertEqual(visible_start(cursor=3, visible_height=4, total=10), 0)
        self.assertEqual(visible_start(cursor=4, visible_height=4, total=10), 1)
        self.assertEqual(visible_start(cursor=9, visible_height=4, total=10), 6)

    def test_request_terminal_size_writes_resize_sequence(self):
        buffer = StringIO()
        with patch("sunray_test.dashboard.tui_app.os.isatty", return_value=True), patch(
            "sunray_test.dashboard.tui_app.sys.stdout",
            buffer,
        ), patch.dict(os.environ, {"TERM": "xterm-256color"}, clear=False):
            request_terminal_size(TUI_PREFERRED_HEIGHT, TUI_PREFERRED_WIDTH)

        self.assertEqual(buffer.getvalue(), "\033[8;38;132t")

    def test_request_terminal_size_can_be_disabled(self):
        buffer = StringIO()
        with patch("sunray_test.dashboard.tui_app.os.isatty", return_value=True), patch(
            "sunray_test.dashboard.tui_app.sys.stdout",
            buffer,
        ), patch.dict(
            os.environ,
            {"TERM": "xterm-256color", "SUNRAY_TEST_TUI_NO_RESIZE": "1"},
            clear=False,
        ):
            request_terminal_size(TUI_PREFERRED_HEIGHT, TUI_PREFERRED_WIDTH)

        self.assertEqual(buffer.getvalue(), "")

    def test_external_fusion_vrpn_ip_reads_external_launch_server(self):
        with patch(
            "sunray_test.dashboard.tui_app.DEFAULT_LAUNCH_FILES",
            ["/tmp/external_fusion.launch"],
        ), patch(
            "sunray_test.dashboard.tui_app.read_launch_file",
            return_value='<launch><arg name="server" default="192.168.20.46"/></launch>',
        ):
            self.assertEqual(external_fusion_vrpn_ip(), "192.168.20.46")

    def test_external_fusion_vrpn_ip_returns_empty_when_missing(self):
        with patch(
            "sunray_test.dashboard.tui_app.DEFAULT_LAUNCH_FILES",
            ["/tmp/external_fusion.launch"],
        ), patch(
            "sunray_test.dashboard.tui_app.read_launch_file",
            side_effect=OSError("missing"),
        ):
            self.assertEqual(external_fusion_vrpn_ip(), "")

    def test_format_tab_lines_handles_empty_and_delay(self):
        self.assertEqual(format_tab_lines([]), [" - none"])
        lines = format_tab_lines([{"title": "run", "delay_s": 2.0, "command": "echo ok"}])
        self.assertEqual(lines, [" 1. run delay=2.0s: echo ok"])

    def test_item_row_stays_within_pane_width(self):
        item = self.model.item_by_id["visual_landing"]
        suffix = format_item_short_tags(item, auto_selected=False)
        line = format_item_row(">", " ", item, suffix, width=36)

        self.assertLessEqual(display_width(line), 36)
        self.assertIn("visual_landing", line)
        self.assertNotIn("飞行", line)
        self.assertNotIn("依赖", line)

    def test_item_rows_align_names_with_fixed_id_width(self):
        function_items = self.model.items_by_group("function")
        item_id_width = item_id_column_width(function_items, width=64)
        rows = [
            format_item_row(" ", " ", item, "", width=64, item_id_width=item_id_width)
            for item in function_items
        ]
        name_columns = [
            display_width(row.split(item.name, 1)[0])
            for row, item in zip(rows, function_items)
        ]

        self.assertEqual(len(set(name_columns)), 1)

    def test_auto_added_item_uses_short_auto_marker(self):
        item = self.model.item_by_id["lidar"]
        suffix = format_item_short_tags(item, auto_selected=True)
        line = format_item_row(" ", "a", item, suffix, width=30)

        self.assertLessEqual(display_width(line), 30)
        self.assertIn("[a]", line)
        self.assertIn("自动", line)

    def test_prompt_param_value_reports_invalid_input_without_crashing(self):
        state = self.make_state()
        state.enter_param_editor()
        screen = FakeScreen()
        app = make_app(screen, state)

        with patch_curses_box_chars(), patch.object(app, "read_dialog_input", return_value="bad-number"):
            app.prompt_current_param_value()

        self.assertIn("参数输入无效", state.status)

    def test_refresh_plan_reuses_cached_plan_until_state_changes(self):
        state = self.make_state()
        screen = FakeScreen()
        calls = []

        def build_plan(_request):
            calls.append(1)
            return state.build_plan()

        app = make_app(screen, state, build_plan_callback=build_plan)

        app.refresh_plan()
        app.refresh_plan()
        self.assertEqual(len(calls), 1)

        state.selected_functions = ["hover"]
        app.refresh_plan()
        self.assertEqual(len(calls), 2)

    def test_run_cleans_up_tools_before_starting_tests(self):
        state = self.make_state()
        screen = FakeScreen(keys=[10])
        app = make_app(screen, state)
        app.tools_state.refresh = lambda: None

        with patch_curses_box_chars(), patch("sunray_test.dashboard.tui_app.curses.curs_set"), patch(
            "sunray_test.dashboard.tui_app.curses.has_colors",
            return_value=False,
        ), patch(
            "sunray_test.dashboard.tui_app.external_fusion_vrpn_ip",
            return_value="",
        ), patch.object(app, "cleanup_tools") as cleanup:
            result = app.run()

        self.assertEqual(result.action, ACTION_START)
        cleanup.assert_called_once_with()

class FakeScreen:
    def __init__(self, keys=None, height=38, width=132):
        self.keys = list(keys or [])
        self.height = height
        self.width = width

    def keypad(self, _enabled):
        pass

    def timeout(self, _milliseconds):
        pass

    def getmaxyx(self):
        return self.height, self.width

    def getch(self):
        return self.keys.pop(0) if self.keys else -1

    def clear(self):
        pass

    def erase(self):
        pass

    def refresh(self):
        pass

    def addstr(self, *_args):
        pass

    def addch(self, *_args):
        pass

    def attron(self, *_args):
        pass

    def attroff(self, *_args):
        pass

    def move(self, *_args):
        pass


def make_app(screen, state, build_plan_callback=None):
    args = Namespace(
        sn="",
        tester="",
        no_prompt=True,
    )
    return TuiApp(
        screen=screen,
        state=state,
        args=args,
        output_dir="/tmp/sunray-output",
        build_plan_callback=build_plan_callback or (lambda _request: state.build_plan()),
        build_suite_path_callback=lambda _plan: "/tmp/sunray-output/20260617_120000/suite.yaml",
    )


def patch_curses_box_chars():
    return patch.multiple(
        "sunray_test.dashboard.tui_app.curses",
        ACS_HLINE="-",
        ACS_VLINE="|",
        ACS_ULCORNER="+",
        ACS_URCORNER="+",
        ACS_LLCORNER="+",
        ACS_LRCORNER="+",
        create=True,
    )


if __name__ == "__main__":
    unittest.main()

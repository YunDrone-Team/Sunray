#!/usr/bin/env python3
import os
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest, DashboardSession
from sunray_test.dashboard.ui import (
    format_item_line,
    parse_selection_tokens,
    print_dependency_preview,
    print_dashboard_header,
    print_items,
    print_suite_preview,
    print_suite_yaml,
    prompt_main_menu,
    prompt_item_selection,
    prompt_plan_action,
    select_dashboard_interactively,
)


class DashboardUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = DashboardModel.load("dashboard")

    def test_format_item_line_shows_named_dependencies(self):
        item = self.model.item_by_id["visual_landing"]
        line = format_item_line(4, item, item_by_id=self.model.item_by_id)
        self.assertIn("visual_landing", line)
        self.assertIn("依赖硬件: battery(电池电压),down_camera(下视相机)", line)

    def test_print_items_shows_direct_test_items_without_presets(self):
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_items(self.model)
        output = buffer.getvalue()
        self.assertIn("硬件测试项目", output)
        self.assertIn("功能测试项目", output)
        self.assertIn("visual_landing", output)
        self.assertNotIn("常用测试组合", output)

    def test_print_dashboard_header_keeps_environment_line_short(self):
        session = DashboardSession(
            model=self.model,
            environment="exp",
            output_dir="/tmp/sunray-output",
            uav_id=1,
        )
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_dashboard_header(session)
        output = buffer.getvalue()
        self.assertIn("环境: exp\n", output)
        self.assertNotIn("默认 exp 链路", output)
        self.assertNotIn("需要仿真时使用 --sim", output)

    def test_main_menu_does_not_show_latest_history_hint(self):
        session = DashboardSession(
            model=self.model,
            environment="exp",
            output_dir="/tmp/sunray-output",
            uav_id=1,
        )
        buffer = StringIO()
        with patch("sys.stdin.isatty", return_value=True), patch(
            "builtins.input",
            return_value="q",
        ), patch(
            "sunray_test.dashboard.history_ui.collect_history_runs",
            return_value={
                "name": "20260617_113654",
                "environment": "sim",
                "status": "pass=5",
                "report_path": "/tmp/report.html",
            },
        ), redirect_stdout(buffer):
            self.assertEqual(prompt_main_menu(session, history_limit=8), "exit")

        output = buffer.getvalue()
        self.assertIn("1. 启动测试", output)
        self.assertNotIn("最近结果:", output)

    def test_interactive_selection_does_not_prompt_for_uav_id(self):
        session = DashboardSession(
            model=self.model,
            environment="exp",
            output_dir="/tmp/sunray-output",
            uav_id=1,
        )
        buffer = StringIO()
        with patch("sys.stdin.isatty", return_value=True), patch(
            "builtins.input",
            side_effect=["lidar", "8"],
        ) as input_mock, redirect_stdout(buffer):
            request = select_dashboard_interactively(session)

        self.assertEqual(request.item_ids, ["lidar", "visual_landing"])
        prompts = [call.args[0] for call in input_mock.call_args_list]
        self.assertNotIn("UAV ID [1]: ", prompts)
        self.assertEqual(len(prompts), 2)
        self.assertIn("选择 [battery]: ", prompts)
        self.assertIn("选择 [none]: ", prompts)

    def test_function_selection_accepts_case_name_alias(self):
        function_items = self.model.items_by_group("function")
        selected = parse_selection_tokens("ego_goal_flight", function_items, [], start_index=5)
        self.assertEqual(selected, ["ego_goal"])

    def test_parse_selection_tokens_supports_index_id_all_and_none(self):
        hardware_items = self.model.items_by_group("hardware")
        self.assertEqual(parse_selection_tokens("1,battery", hardware_items, []), ["front_camera", "battery"])
        self.assertEqual(
            parse_selection_tokens("all", hardware_items, []),
            ["front_camera", "down_camera", "battery", "lidar"],
        )
        self.assertEqual(parse_selection_tokens("none", hardware_items, ["battery"]), [])
        self.assertEqual(parse_selection_tokens("", hardware_items, ["battery"]), ["battery"])

    def test_parse_selection_tokens_supports_index_ranges(self):
        hardware_items = self.model.items_by_group("hardware")
        self.assertEqual(
            parse_selection_tokens("1-3,lidar", hardware_items, []),
            ["front_camera", "down_camera", "battery", "lidar"],
        )
        self.assertEqual(
            parse_selection_tokens("3-1,battery", hardware_items, []),
            ["battery", "down_camera", "front_camera"],
        )

    def test_parse_selection_tokens_supports_global_start_index(self):
        function_items = self.model.items_by_group("function")
        self.assertEqual(
            parse_selection_tokens("5,8", function_items, [], start_index=5),
            ["hover", "visual_landing"],
        )
        self.assertEqual(
            parse_selection_tokens("8-6", function_items, [], start_index=5),
            ["visual_landing", "waypoint", "ego_goal"],
        )

    def test_parse_selection_tokens_rejects_invalid_ranges(self):
        hardware_items = self.model.items_by_group("hardware")
        with self.assertRaises(SystemExit):
            parse_selection_tokens("1-99", hardware_items, [])

    def test_prompt_item_selection_retries_after_invalid_selection(self):
        function_items = self.model.items_by_group("function")
        buffer = StringIO()
        with patch("builtins.input", side_effect=["1-99", "8"]), redirect_stdout(buffer):
            selected = prompt_item_selection(
                self.model,
                "功能测试",
                function_items,
                [],
                start_index=5,
            )
        self.assertEqual(selected, ["visual_landing"])
        output = buffer.getvalue()
        self.assertIn("无效测试项目范围: 1-99", output)
        self.assertIn("请重新输入。", output)

    def test_prompt_item_selection_can_reprint_choices(self):
        function_items = self.model.items_by_group("function")
        buffer = StringIO()
        with patch("builtins.input", side_effect=["?", "visual_landing"]), redirect_stdout(buffer):
            selected = prompt_item_selection(
                self.model,
                "功能测试",
                function_items,
                [],
                start_index=5,
            )
        self.assertEqual(selected, ["visual_landing"])
        output = buffer.getvalue()
        self.assertGreaterEqual(output.count("功能测试"), 2)
        self.assertIn(" 8.   visual_landing", output)
        self.assertIn("?=重打列表", output)

    def test_print_dependency_preview_shows_auto_added_hardware(self):
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_dependency_preview(self.model, ["visual_landing"])
        output = buffer.getvalue()
        self.assertIn("依赖预览", output)
        self.assertIn("visual_landing(视觉降落) -> battery(电池电压), down_camera(下视相机)", output)
        self.assertIn("将自动补齐: battery(电池电压), down_camera(下视相机)", output)

    def test_print_dependency_preview_is_quiet_without_dependencies(self):
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_dependency_preview(self.model, ["battery"])
        self.assertEqual(buffer.getvalue(), "")

    def test_prompt_plan_action_supports_show_suite(self):
        with patch("builtins.input", return_value="s"):
            self.assertEqual(
                prompt_plan_action(interactive=True, dry_run=False, yes=False, no_prompt=False),
                "show_suite",
            )

    def test_prompt_plan_action_supports_write_suite(self):
        with patch("builtins.input", return_value="w"):
            self.assertEqual(
                prompt_plan_action(interactive=True, dry_run=False, yes=False, no_prompt=False),
                "write_suite",
            )

    def test_prompt_plan_action_supports_preview(self):
        with patch("builtins.input", return_value="p"):
            self.assertEqual(
                prompt_plan_action(interactive=True, dry_run=False, yes=False, no_prompt=False),
                "preview",
            )

    def test_print_suite_yaml_outputs_generated_yaml(self):
        suite = {
            "name": "dashboard",
            "steps": [{"case": "battery_voltage", "name": "电池电压"}],
        }
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_suite_yaml(suite)
        output = buffer.getvalue()
        self.assertIn("=== Generated Suite YAML ===", output)
        self.assertIn("name: dashboard", output)
        self.assertIn("name: 电池电压", output)

    def test_print_suite_preview_shows_run_policy_and_preview_path(self):
        plan = self.model.build_plan(
            requested_item_ids=["visual_landing"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=False,
            continue_on_failure=True,
        )
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_suite_preview(
                model=self.model,
                plan=plan,
                request=DashboardRequest(item_ids=["visual_landing"], source="items"),
                uav_id=1,
                no_bringup=True,
                output_dir="/tmp/sunray-output",
                suite_path="/tmp/sunray-output/generated_suites/dashboard.yaml",
                suite_will_be_written=False,
                record_rosbag=False,
                continue_on_failure=True,
                terminal_status="gnome-terminal required for runner only (missing)",
                runner_command=(
                    "rosrun sunray_test run_test_dashboard.py --run-suite "
                    "--suite-file /tmp/sunray-output/generated_suites/dashboard.yaml"
                ),
            )
        output = buffer.getvalue()
        self.assertIn("preview path, not written", output)
        self.assertIn("[运行策略]", output)
        self.assertIn("record rosbag: no", output)
        self.assertIn("failure policy: continue on failure", output)
        self.assertIn("airborne phases: arm_and_takeoff required", output)
        self.assertIn("bringup launch: manual/disabled", output)
        self.assertIn("terminal: gnome-terminal required for runner only (missing)", output)
        self.assertIn("runner command: rosrun sunray_test run_test_dashboard.py --run-suite --suite-file", output)

    def test_print_suite_preview_shows_write_suite_only_path_state(self):
        plan = self.model.build_plan(
            requested_item_ids=["battery"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_suite_preview(
                model=self.model,
                plan=plan,
                request=DashboardRequest(item_ids=["battery"], source="items"),
                uav_id=1,
                no_bringup=True,
                output_dir="/tmp/sunray-output",
                suite_path="/tmp/sunray-output/generated_suites/dashboard.yaml",
                suite_will_be_written=True,
                suite_write_mode="before exit",
            )
        output = buffer.getvalue()
        self.assertIn("will be written before exit", output)
        self.assertIn("airborne phases: not required", output)


if __name__ == "__main__":
    unittest.main()

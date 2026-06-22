#!/usr/bin/env python3
import os
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.console import (
    format_plan_items_table,
    print_config_check,
    print_items,
    print_plan_preview,
    print_suite_yaml,
)
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest


class DashboardConsoleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = DashboardModel.load("dashboard")

    def test_print_items_shows_direct_test_items_without_presets(self):
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_items(self.model)

        output = buffer.getvalue()
        self.assertIn("硬件测试项目", output)
        self.assertIn("功能测试项目", output)
        self.assertIn("visual_landing", output)
        self.assertNotIn("常用测试组合", output)

    def test_print_config_check_outputs_dashboard_summary(self):
        buffer = StringIO()
        with redirect_stdout(buffer):
            print_config_check(self.model)

        output = buffer.getvalue()
        self.assertIn("status: ok", output)
        self.assertIn("items: total=8, hardware=4, function=4", output)
        self.assertIn("default_items: battery", output)

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

    def test_format_plan_items_table_uses_chinese_execution_steps(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        table = format_plan_items_table(self.model, plan)

        self.assertIn("测试项目", table)
        self.assertIn("电池电压", table)
        self.assertIn("解锁/起飞", table)
        self.assertIn("悬停", table)
        self.assertIn("降落", table)
        self.assertNotIn("来源", table)
        self.assertNotIn("battery", table)

    def test_print_plan_preview_shows_plan_and_tabs(self):
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
            print_plan_preview(
                model=self.model,
                plan=plan,
                request=DashboardRequest(item_ids=["visual_landing"], source="items"),
                output_dir="/tmp/sunray-output",
                suite_path="/tmp/sunray-output/generated_suites/dashboard.yaml",
                bringup_tabs=[{"title": "control", "command": "roslaunch control.launch"}],
                runner_tabs=[{"title": "runner", "command": "rosrun sunray_test run_test_dashboard.py"}],
            )

        output = buffer.getvalue()
        self.assertIn("=== Dashboard 测试计划 ===", output)
        self.assertIn("environment: exp", output)
        self.assertIn("requested: visual_landing(视觉降落)", output)
        self.assertIn("测试项目", output)
        self.assertIn("bringup:", output)
        self.assertIn("runner:", output)


if __name__ == "__main__":
    unittest.main()

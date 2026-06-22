#!/usr/bin/env python3
import os
import sys
import unittest
from argparse import Namespace
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard import parse_args
from sunray_test.dashboard.cli import build_validated_plan
from sunray_test.dashboard.console import format_plan_items_table
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.suite_runtime import (
    build_manual_runner_command,
)
from sunray_test.dashboard.terminal import (
    launch_terminal_window,
    write_gnome_terminal_config,
)


class DashboardCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = DashboardModel.load("dashboard")

    def test_build_manual_runner_command_reuses_runner_cli(self):
        args = Namespace(
            uav_id=1,
            sn="SN001",
            tester="tester",
            no_prompt=True,
        )
        plan = self.model.build_plan(
            requested_item_ids=["visual_landing"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        command = build_manual_runner_command(
            model=self.model,
            args=args,
            plan=plan,
            suite_path="/tmp/dashboard.yaml",
            output_dir="/tmp/output",
        )

        self.assertTrue(command.startswith("rosrun sunray_test run_test_dashboard.py "))
        self.assertIn("--run-suite", command)
        self.assertIn("--platform sunray150_basic", command)
        self.assertIn("--environment exp", command)
        self.assertIn("--suite-file /tmp/dashboard.yaml", command)
        self.assertNotIn("--uav-id", command)
        self.assertIn("--output-dir /tmp/output", command)
        self.assertIn("--external-source 3", command)
        self.assertIn("--sn SN001", command)
        self.assertIn("--tester tester", command)
        self.assertIn("--no-prompt", command)

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
        self.assertNotIn("自动补齐", table)
        self.assertNotIn("用户选择", table)
        self.assertNotIn("battery", table)
        self.assertNotIn("hover", table)
        self.assertNotIn("arm_and_takeoff", table)

    def test_build_validated_plan_stores_validation_warning_on_frozen_plan(self):
        args = Namespace(
            external_source=None,
            record_rosbag=True,
            continue_on_failure=False,
            profile="",
            uav_id=1,
            items="",
            list=False,
            check_config=False,
            history=False,
            open_latest_report=False,
            dry_run=False,
            show_suite=False,
        )
        request = Namespace(
            item_ids=["battery"],
            external_source_override=None,
            param_overrides={},
            profile_override="",
        )

        with patch(
            "sunray_test.dashboard.cli.validate_dashboard_suite_schema",
            return_value="warning",
        ):
            plan = build_validated_plan(self.model, args, request, "exp")

        self.assertEqual(plan.validation_warning, "warning")

    def test_launch_terminal_window_uses_single_session_config(self):
        tabs = [
            {
                "title": "tab1",
                "command": "echo hello",
                "delay_s": 0.0,
                "hold_open": False,
            },
            {
                "title": "tab2",
                "command": "echo world",
                "delay_s": 1.0,
                "hold_open": True,
            },
        ]
        with patch("sunray_test.dashboard.terminal.shutil.which", return_value="/usr/bin/gnome-terminal"), patch(
            "sunray_test.dashboard.terminal.subprocess.Popen"
        ) as popen_mock:
            launch_terminal_window("test", tabs)

        self.assertEqual(popen_mock.call_count, 1)
        argv = popen_mock.call_args.args[0]
        self.assertEqual(argv[0], "gnome-terminal")
        self.assertNotIn("--command", argv)
        self.assertNotIn("-e", argv)
        self.assertEqual(len(argv), 2)
        self.assertTrue(argv[1].startswith("--load-config="))

        config_path = argv[1].split("=", 1)[1]
        try:
            with open(config_path, "r", encoding="utf-8") as handle:
                config = handle.read()
        finally:
            os.unlink(config_path)

        self.assertIn("Windows=Window0;", config)
        self.assertIn("Terminals=Terminal0;Terminal1;", config)
        self.assertIn("[Terminal0]", config)
        self.assertIn("[Terminal1]", config)
        self.assertIn("Title=tab1", config)
        self.assertIn("Title=tab2", config)
        self.assertIn("bash -lc", config)
        self.assertIn("echo hello", config)
        self.assertIn("sleep 1.0", config)
        self.assertIn("echo world", config)


    def test_gnome_terminal_session_config_command_is_parseable(self):
        try:
            from gi.repository import GLib
        except (ImportError, ValueError):
            self.skipTest("python gi/GLib is not available")

        tabs = [
            {
                "title": "runner",
                "command": "rosrun sunray_test run_test_dashboard.py --run-suite --suite-file /tmp/a.yaml",
                "delay_s": 15.0,
                "hold_open": True,
            }
        ]
        config_path = write_gnome_terminal_config("test", tabs)
        try:
            key_file = GLib.KeyFile()
            key_file.load_from_file(config_path, GLib.KeyFileFlags.NONE)
            command = GLib.strcompress(key_file.get_string("Terminal0", "Command"))
            _ok, argv = GLib.shell_parse_argv(command)
        finally:
            os.unlink(config_path)

        self.assertEqual(argv[0:2], ["bash", "-lc"])
        self.assertIn('echo ""', argv[2])
        self.assertIn('echo "[sunray_test] tab exited', argv[2])
        self.assertIn("--run-suite", argv[2])

    def test_run_suite_mode_requires_generated_suite_file(self):
        old_argv = sys.argv
        try:
            sys.argv = ["run_test_dashboard.py", "--run-suite", "--platform", "sunray150_basic"]
            with self.assertRaises(SystemExit):
                parse_args()
                from sunray_test.dashboard import main

                main()
        finally:
            sys.argv = old_argv

    def test_preview_actions_are_mutually_exclusive(self):
        argv = [
            "run_test_dashboard.py",
            "--items",
            "visual_landing",
            "--dry-run",
            "--show-suite",
        ]
        old_argv = sys.argv
        try:
            sys.argv = argv
            with self.assertRaises(SystemExit):
                parse_args()
        finally:
            sys.argv = old_argv

    def test_dashboard_does_not_accept_uav_id_argument(self):
        old_argv = sys.argv
        try:
            sys.argv = ["run_test_dashboard.py", "--uav-id", "2"]
            with self.assertRaises(SystemExit):
                parse_args()
        finally:
            sys.argv = old_argv


if __name__ == "__main__":
    unittest.main()

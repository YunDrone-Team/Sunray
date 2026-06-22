#!/usr/bin/env python3
import os
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.tools import vrpn_config as vrpn_tool
from sunray_test.dashboard.tools import (
    TOOL_BY_ID,
    VRPN_MODE_CONFIRM,
    VRPN_MODE_INPUT,
    EmbeddedToolsState,
    ProcessToolState,
    VrpnToolState,
)
from sunray_test.dashboard.tui_tools import build_tool_view, handle_tool_key
from sunray_test.dashboard.tui_tools import mid360_result_lines


class DashboardVrpnToolTest(unittest.TestCase):
    def test_extract_and_replace_server(self):
        content = '<launch><arg name="server" default="192.168.1.10" /></launch>'

        self.assertEqual(vrpn_tool.extract_server(content), "192.168.1.10")
        updated = vrpn_tool.replace_server(content, "192.168.1.20")
        self.assertIn('default="192.168.1.20"', updated)

    def test_load_select_and_write_updates(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            first = os.path.join(tmpdir, "external_fusion.launch")
            second = os.path.join(tmpdir, "sunray_vrpn.launch")
            self.write_launch(first, "10.0.0.1")
            self.write_launch(second, "10.0.0.2")

            contents = vrpn_tool.load_launch_contents([first, second])
            target = vrpn_tool.VrpnTarget("both", "both", (first, second))
            with patch_target(target):
                selected = vrpn_tool.selected_contents(contents, "both")
                updated_paths = vrpn_tool.write_updates(selected, "10.0.0.9")

            self.assertEqual(updated_paths, [first, second])
            self.assertEqual(vrpn_tool.extract_server(open_text(first)), "10.0.0.9")
            self.assertEqual(vrpn_tool.extract_server(open_text(second)), "10.0.0.9")

    def test_vrpn_state_input_confirm_cancel(self):
        state = VrpnToolState()

        state.begin_input()
        self.assertEqual(state.mode, VRPN_MODE_INPUT)
        state.append_input("1")
        state.append_input("a")
        state.append_input(".")
        self.assertEqual(state.input_value, "1.")
        state.confirm_input()
        self.assertEqual(state.mode, VRPN_MODE_CONFIRM)
        self.assertFalse(state.cancel())
        self.assertEqual(state.mode, VRPN_MODE_INPUT)
        self.assertFalse(state.cancel())
        self.assertEqual(state.mode, "select")
        self.assertTrue(state.cancel())

    def test_process_tool_follows_latest_log_until_user_scrolls(self):
        state = ProcessToolState(title="demo", command="echo ok")

        state.append_log("one")
        state.append_log("two")
        self.assertEqual(state.scroll, 2)
        self.assertTrue(state.follow_tail)

        state.move_scroll(-1)
        self.assertFalse(state.follow_tail)
        self.assertEqual(state.scroll, 1)
        state.append_log("three")
        self.assertEqual(state.scroll, 1)

    def test_process_tool_cleanup_terminates_running_process(self):
        state = ProcessToolState(title="demo", command="demo")
        process = FakeProcess()
        state.process = process

        state.cleanup()

        self.assertTrue(process.terminated)
        self.assertTrue(process.waited)
        self.assertIsNone(state.process)

    def test_embedded_mid360_uses_tool_definition_command(self):
        state = EmbeddedToolsState()

        self.assertEqual(TOOL_BY_ID["livox_mid360"].title, "雷达IP自动配置")
        self.assertEqual(state.mid360.title, TOOL_BY_ID["livox_mid360"].title)
        self.assertEqual(state.mid360.command, TOOL_BY_ID["livox_mid360"].command)

    def test_enter_mid360_tool_starts_process_immediately(self):
        tui_state = make_tui_state()
        tui_state.active_pane = "tools"
        tools_state = EmbeddedToolsState()

        with patch.object(tools_state.mid360, "start") as start:
            handle_tool_key(tui_state, tools_state, "enter")

        self.assertEqual(tui_state.active_tool_id, "livox_mid360")
        start.assert_called_once_with()

    def test_mid360_tool_view_splits_log_and_result(self):
        tui_state = make_tui_state()
        tui_state.active_tool_id = "livox_mid360"
        tools_state = EmbeddedToolsState()
        tools_state.mid360.logs = [
            "$ rosrun sunray_test livox_mid360_autoconfig.py",
            "lidar_ip:        192.168.1.148",
            "broadcast_code:  ABCD123456",
            "config_status:   match",
        ]

        view = build_tool_view(tui_state, tools_state, visible_height=10)

        self.assertEqual(view.title, "雷达IP自动配置")
        self.assertIn("lidar_ip:        192.168.1.148", view.lines)
        self.assertEqual(view.result_title, "检测结果")
        self.assertIn("雷达IP      : 192.168.1.148", view.result_lines)
        self.assertIn("SN          : ABCD123456", view.result_lines)
        self.assertIn("配置状态    : 匹配", view.result_lines)

    def test_mid360_result_explains_missing_config_file(self):
        state = ProcessToolState(title="demo", command="demo")
        state.logs = ["config_status:   missing_file"]

        self.assertIn("配置状态    : 配置文件不存在", mid360_result_lines(state))

    def test_mid360_missing_config_file_is_not_updatable(self):
        state = EmbeddedToolsState().mid360
        state.logs = [
            "lidar_ip:        192.168.1.148",
            "config:          /tmp/sunray_missing_mid360_config.json",
            "config_lidar_ip: N/A",
            "config_status:   missing_file",
        ]

        self.assertFalse(state.update_available)

    def test_mid360_prompts_for_update_when_config_mismatches(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "MID360_config.json"
            config_path.write_text('{"lidar_configs":[{"ip":"192.168.1.100"}]}', encoding="utf-8")
            tools_state = EmbeddedToolsState()
            tools_state.mid360.logs = [
                "lidar_ip:        192.168.1.148",
                f"config:          {config_path}",
                "config_lidar_ip: 192.168.1.100",
                "config_status:   mismatch (192.168.1.100 != 192.168.1.148)",
            ]

            self.assertTrue(tools_state.mid360.update_available)
            self.assertIn("检测到配置可更新", mid360_result_lines(tools_state.mid360))

    def test_mid360_apply_update_uses_scanned_result_without_rescan(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "MID360_config.json"
            config_path.write_text(
                json.dumps({"lidar_configs": [{"ip": "192.168.1.100"}]}),
                encoding="utf-8",
            )
            state = EmbeddedToolsState().mid360
            state.logs = [
                "lidar_ip:        192.168.1.148",
                f"config:          {config_path}",
                "config_lidar_ip: 192.168.1.100",
                "config_status:   mismatch (192.168.1.100 != 192.168.1.148)",
            ]

            state.apply_update()

            data = json.loads(config_path.read_text(encoding="utf-8"))
            self.assertEqual(data["lidar_configs"][0]["ip"], "192.168.1.148")
            self.assertTrue(state.update_done)
            self.assertFalse(state.update_available)
            self.assertIn(f"updated: {config_path}", state.logs)
            result_lines = mid360_result_lines(state)
            self.assertIn("配置雷达IP  : 192.168.1.148", result_lines)
            self.assertIn("配置状态    : 匹配", result_lines)
            self.assertNotIn("配置状态    : 不一致", "\n".join(result_lines))

    def test_mid360_skip_update_hides_prompt(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "MID360_config.json"
            config_path.write_text('{"lidar_configs":[{}]}', encoding="utf-8")
            state = EmbeddedToolsState().mid360
            state.logs = [
                "lidar_ip:        192.168.1.148",
                f"config:          {config_path}",
                "config_status:   missing_lidar_ip",
            ]

            self.assertTrue(state.update_available)
            state.skip_update()

            self.assertFalse(state.update_available)
            self.assertEqual(state.status, "已跳过更新")

    def test_mid360_tool_key_applies_update_after_scan(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "MID360_config.json"
            config_path.write_text('{"lidar_configs":[{"ip":"1.1.1.1"}]}', encoding="utf-8")
            tui_state = make_tui_state()
            tui_state.active_tool_id = "livox_mid360"
            tools_state = EmbeddedToolsState()
            tools_state.mid360.logs = [
                "lidar_ip:        192.168.1.148",
                f"config:          {config_path}",
                "config_status:   mismatch (1.1.1.1 != 192.168.1.148)",
            ]

            with patch.object(tools_state.mid360, "apply_update") as apply_update:
                handle_tool_key(tui_state, tools_state, "y")

            apply_update.assert_called_once_with()

    @staticmethod
    def write_launch(path: str, server: str) -> None:
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(f'<launch><arg name="server" default="{server}" /></launch>')


class patch_target:
    def __init__(self, target):
        self.target = target
        self.original = dict(vrpn_tool.TARGET_BY_KEY)

    def __enter__(self):
        vrpn_tool.TARGET_BY_KEY[self.target.key] = self.target

    def __exit__(self, exc_type, exc, traceback):
        vrpn_tool.TARGET_BY_KEY.clear()
        vrpn_tool.TARGET_BY_KEY.update(self.original)


def open_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def make_tui_state():
    from sunray_test.dashboard.model import DashboardModel
    from sunray_test.dashboard.tui_state import TuiState

    return TuiState(
        model=DashboardModel.load("dashboard"),
        environment="exp",
        uav_id=1,
    )


class FakeProcess:
    stdout = None

    def __init__(self):
        self.terminated = False
        self.waited = False

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        self.waited = True
        return 0

    def kill(self):
        pass


if __name__ == "__main__":
    unittest.main()

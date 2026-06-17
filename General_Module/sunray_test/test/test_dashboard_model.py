#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.model import DashboardModel


class DashboardModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = DashboardModel.load("dashboard")

    def test_visual_landing_exp_plan(self):
        plan = self.model.build_plan(
            requested_item_ids=["visual_landing"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        self.assertEqual(plan.profile, "sunray150_basic")
        self.assertEqual(plan.external_source, 3)
        self.assertEqual(plan.selection.auto_added_item_ids, ["battery", "down_camera"])
        self.assertEqual(plan.selection.item_ids, ["down_camera", "battery", "visual_landing"])
        self.assertEqual(
            [step.get("case") or step.get("phase") for step in plan.suite["steps"]],
            ["down_camera_alive", "battery_voltage", "arm_and_takeoff", "visual_landing"],
        )
        detection_tabs = [
            tab
            for tab in self.model.build_tabs("bringup", "exp", plan.runtime_state)
            if tab["title"] == "detection"
        ]
        self.assertEqual(len(detection_tabs), 1)
        self.assertIn("down_image_topic:=/web_cam/image_raw", detection_tabs[0]["command"])

    def test_unknown_item_fails(self):
        with self.assertRaises(SystemExit):
            self.model.parse_item_ids("missing")

    def test_case_name_alias_selects_matching_item(self):
        self.assertEqual(self.model.parse_item_ids("ego_goal_flight"), ["ego_goal"])

    def test_config_summary(self):
        summary = self.model.config_summary()
        self.assertEqual(summary["name"], "dashboard")
        self.assertEqual(summary["test_items"]["hardware"], 4)
        self.assertEqual(summary["test_items"]["function"], 4)
        self.assertEqual(summary["default_items"], ["battery"])
        self.assertEqual(summary["runtime_profiles"]["default"], "sunray150_basic")
        self.assertGreaterEqual(summary["bringup_tabs"]["sim"], 1)
        self.assertGreaterEqual(summary["runner_tabs"]["exp"], 1)

    def test_ego_goal_exp_plan_has_ego_bringup_tab(self):
        plan = self.model.build_plan(
            requested_item_ids=self.model.parse_item_ids("ego_goal_flight"),
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        tabs = self.model.build_tabs("bringup", "exp", plan.runtime_state)
        ego_tabs = [tab for tab in tabs if tab["title"] == "ego"]
        self.assertEqual(len(ego_tabs), 1)
        self.assertIn("sunray_tests_ego.launch", ego_tabs[0]["command"])

    def test_ego_goal_sim_plan_uses_lidar_profile_and_vehicle(self):
        plan = self.model.build_plan(
            requested_item_ids=["ego_goal", "visual_landing"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.external_source, 2)
        self.assertEqual(plan.runtime_state["variables"]["sim_vehicle"], "sunray150_with_mid360")
        self.assertEqual(plan.selection.auto_added_item_ids, ["battery", "lidar", "down_camera"])
        self.assertEqual(
            plan.selection.item_ids,
            ["down_camera", "battery", "lidar", "ego_goal", "visual_landing"],
        )
        simulator_tabs = [
            tab
            for tab in self.model.build_tabs("bringup", "sim", plan.runtime_state)
            if tab["title"] == "simulator"
        ]
        self.assertEqual(len(simulator_tabs), 1)
        self.assertIn("vehicle:=sunray150_with_mid360", simulator_tabs[0]["command"])


if __name__ == "__main__":
    unittest.main()

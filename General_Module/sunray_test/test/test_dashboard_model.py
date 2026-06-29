#!/usr/bin/env python3
import os
import sys
import unittest
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.suite_runtime import validate_dashboard_suite_schema


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

    def test_airborne_function_plan_adds_land_phase(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover", "ego_goal", "waypoint"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        self.assertEqual(
            [step.get("case") or step.get("phase") for step in plan.suite["steps"]],
            [
                "battery_voltage",
                "lidar_health_check",
                "arm_and_takeoff",
                "hover_stability",
                "ego_goal_flight",
                "waypoint_flight",
                "land",
            ],
        )

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
        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.external_source, 0)
        self.assertEqual(plan.external_source_label, "ODOM")
        self.assertEqual(len(ego_tabs), 1)
        self.assertIn("sunray_tests_ego.launch", ego_tabs[0]["command"])

    def test_lidar_profile_defaults_to_odom_in_exp(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            profile_override="sunray150_lidar",
        )

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.external_source, 0)
        self.assertEqual(plan.external_source_label, "ODOM")

    def test_lidar_profile_keeps_user_external_source_override(self):
        plan = self.model.build_plan(
            requested_item_ids=["ego_goal"],
            environment="exp",
            uav_id=1,
            external_source_override=3,
            record_rosbag=True,
            continue_on_failure=False,
        )

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.external_source, 3)
        self.assertEqual(plan.external_source_label, "MOCAP")

    def test_external_source_override_updates_exp_fusion_command(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="exp",
            uav_id=1,
            external_source_override=0,
            record_rosbag=True,
            continue_on_failure=False,
        )

        tabs = self.model.build_tabs("bringup", "exp", plan.runtime_state)
        fusion_tabs = [tab for tab in tabs if tab["title"] == "fusion"]
        self.assertEqual(plan.external_source, 0)
        self.assertEqual(plan.external_source_label, "ODOM")
        self.assertEqual(len(fusion_tabs), 1)
        self.assertIn("external_source:=0", fusion_tabs[0]["command"])
        self.assertNotIn("position_topic", fusion_tabs[0]["command"])

    def test_exp_web_cam_bringup_only_when_camera_is_used(self):
        hover_plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )
        hover_titles = [
            tab["title"]
            for tab in self.model.build_tabs("bringup", "exp", hover_plan.runtime_state)
        ]
        self.assertNotIn("web_cam", hover_titles)

        camera_plan = self.model.build_plan(
            requested_item_ids=["front_camera"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )
        camera_titles = [
            tab["title"]
            for tab in self.model.build_tabs("bringup", "exp", camera_plan.runtime_state)
        ]
        self.assertIn("web_cam", camera_titles)

        web_cam_tabs = [
            tab
            for tab in self.model.build_tabs("bringup", "exp", camera_plan.runtime_state)
            if tab["title"] == "web_cam"
        ]
        self.assertIn("roslaunch --wait", web_cam_tabs[0]["command"])

    def test_hardware_only_camera_bringup_starts_master_first(self):
        plan = self.model.build_plan(
            requested_item_ids=["front_camera"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        tabs = self.model.build_tabs("bringup", "exp", plan.runtime_state)
        self.assertEqual(tabs[0]["title"], "roscore")
        self.assertEqual(tabs[0]["command"], "rosrun sunray_test ensure_ros_master.py")
        self.assertNotIn("mavros", [tab["title"] for tab in tabs])

    def test_hardware_only_battery_bringup_starts_mavros(self):
        plan = self.model.build_plan(
            requested_item_ids=["battery"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        tab_titles = [
            tab["title"]
            for tab in self.model.build_tabs("bringup", "exp", plan.runtime_state)
        ]
        self.assertIn("mavros", tab_titles)
        self.assertNotIn("fusion", tab_titles)
        self.assertNotIn("control", tab_titles)

    def test_all_bringup_roslaunch_commands_wait_for_shared_master(self):
        item_sets = [
            ["front_camera"],
            ["down_camera"],
            ["battery"],
            ["lidar"],
            ["hover"],
            ["ego_goal"],
            ["waypoint"],
            ["visual_landing"],
            ["hover", "front_camera"],
            ["visual_landing", "ego_goal"],
        ]

        for environment in ("exp", "sim"):
            for item_ids in item_sets:
                with self.subTest(environment=environment, item_ids=item_ids):
                    plan = self.model.build_plan(
                        requested_item_ids=item_ids,
                        environment=environment,
                        uav_id=1,
                        external_source_override=None,
                        record_rosbag=True,
                        continue_on_failure=False,
                    )
                    tabs = self.model.build_tabs("bringup", environment, plan.runtime_state)
                    self.assertEqual(tabs[0]["title"], "roscore")
                    for tab in tabs[1:]:
                        command = tab["command"]
                        if command.startswith("roslaunch "):
                            self.assertTrue(
                                command.startswith("roslaunch --wait "),
                                f"{tab['title']} command missing --wait: {command}",
                            )

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

    def test_profile_override_keeps_lidar_model_for_hover_without_lidar_bringup(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            profile_override="sunray150_lidar",
        )

        self.assertEqual(plan.profile, "sunray150_lidar")
        self.assertEqual(plan.profile_reason, "user")
        self.assertEqual(plan.runtime_state["variables"]["sim_vehicle"], "sunray150_with_mid360")
        self.assertFalse(plan.runtime_state["condition_context"]["uses_lidar"])

        tabs = self.model.build_tabs("bringup", "sim", plan.runtime_state)
        tab_titles = [tab["title"] for tab in tabs]
        self.assertIn("simulator", tab_titles)
        self.assertNotIn("ego", tab_titles)
        self.assertIn("rviz", tab_titles)

    def test_lidar_profile_with_odom_starts_lidar_localization_bringup(self):
        plan = self.model.build_plan(
            requested_item_ids=["hover"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            profile_override="sunray150_lidar",
        )

        self.assertEqual(plan.external_source, 0)
        self.assertTrue(plan.runtime_state["condition_context"]["uses_lidar"])
        tabs = self.model.build_tabs("bringup", "exp", plan.runtime_state)
        tab_titles = [tab["title"] for tab in tabs]
        self.assertIn("mid360_driver", tab_titles)
        self.assertIn("mid360_mapping", tab_titles)
        self.assertNotIn("ego", tab_titles)

    def test_basic_profile_override_does_not_disable_lidar_resource_bringup(self):
        plan = self.model.build_plan(
            requested_item_ids=["ego_goal"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            profile_override="sunray150_basic",
        )

        self.assertEqual(plan.profile, "sunray150_basic")
        self.assertEqual(plan.profile_reason, "user")
        self.assertTrue(plan.runtime_state["condition_context"]["uses_lidar"])
        self.assertEqual(plan.runtime_state["variables"]["sim_vehicle"], "sunray150")
        tabs = self.model.build_tabs("bringup", "sim", plan.runtime_state)
        self.assertIn("ego", [tab["title"] for tab in tabs])

    def test_param_schema_and_overrides_are_applied_to_suite(self):
        specs = self.model.item_param_specs("visual_landing")
        paths = [spec["path"] for spec in specs]

        self.assertIn("launch_args.error_xy", paths)
        self.assertIn("水平误差阈值", [spec["name"] for spec in specs])

        plan = self.model.build_plan(
            requested_item_ids=["visual_landing"],
            environment="exp",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            param_overrides={
                "visual_landing": {
                    "height_m": 1.8,
                    "launch_args.error_xy": 0.12,
                }
            },
        )
        visual_step = [step for step in plan.suite["steps"] if step.get("case") == "visual_landing"][0]

        self.assertEqual(visual_step["params"]["height_m"], 1.8)
        self.assertEqual(visual_step["params"]["launch_args"]["error_xy"], 0.12)

    def test_goal_and_waypoint_points_can_be_overridden(self):
        ego_goals = self.model.normalize_param_value(
            {"path": "goals", "type": "points"},
            "[[1.0, 0.0, 1.2], [2.0, 0.5, 1.2]]",
        )
        waypoints = self.model.normalize_param_value(
            {"path": "waypoints", "type": "points"},
            "[[0.5, -1.0, 1.2], [0.5, 1.0, 1.2]]",
        )

        plan = self.model.build_plan(
            requested_item_ids=["ego_goal", "waypoint"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
            param_overrides={
                "ego_goal": {"goals": ego_goals},
                "waypoint": {"waypoints": waypoints},
            },
        )

        ego_step = [step for step in plan.suite["steps"] if step.get("case") == "ego_goal_flight"][0]
        waypoint_step = [step for step in plan.suite["steps"] if step.get("case") == "waypoint_flight"][0]

        self.assertEqual(ego_step["params"]["goals"], [[1.0, 0.0, 1.2], [2.0, 0.5, 1.2]])
        self.assertEqual(waypoint_step["params"]["waypoints"], [[0.5, -1.0, 1.2], [0.5, 1.0, 1.2]])
        validate_dashboard_suite_schema(
            plan.profile,
            plan.environment,
            plan.suite,
            uav_id=1,
            external_source=plan.external_source,
        )

    def test_param_schema_defaults_build_valid_suites(self):
        for item in self.model.test_items:
            overrides = {
                spec["path"]: spec["default"]
                for spec in self.model.item_param_specs(item.item_id)
                if spec.get("default") != ""
            }
            with self.subTest(item=item.item_id):
                plan = self.model.build_plan(
                    requested_item_ids=[item.item_id],
                    environment="sim",
                    uav_id=1,
                    external_source_override=None,
                    record_rosbag=True,
                    continue_on_failure=False,
                    param_overrides={item.item_id: overrides},
                )
                validate_dashboard_suite_schema(
                    plan.profile,
                    plan.environment,
                    plan.suite,
                    uav_id=1,
                    external_source=plan.external_source,
                )

    def test_schema_validation_warning_is_returned_not_printed(self):
        plan = self.model.build_plan(
            requested_item_ids=["ego_goal"],
            environment="sim",
            uav_id=1,
            external_source_override=None,
            record_rosbag=True,
            continue_on_failure=False,
        )

        with patch(
            "sunray_test.core.suite_loader.load_config_triplet",
            side_effect=ModuleNotFoundError("No module named quadrotor_msgs", name="quadrotor_msgs"),
        ):
            warning = validate_dashboard_suite_schema(
                plan.profile,
                plan.environment,
                plan.suite,
                uav_id=1,
                external_source=plan.external_source,
            )

        self.assertIn("quadrotor_msgs", warning)
        self.assertIn("已跳过完整 ROS case 校验", warning)

    def test_normalize_param_value_clamps_numeric_values(self):
        spec = {
            "path": "timeout_s",
            "type": "float",
            "min": 1.0,
            "max": 5.0,
        }

        self.assertEqual(self.model.normalize_param_value(spec, "9.0"), 5.0)
        self.assertEqual(self.model.normalize_param_value(spec, "0.1"), 1.0)


if __name__ == "__main__":
    unittest.main()

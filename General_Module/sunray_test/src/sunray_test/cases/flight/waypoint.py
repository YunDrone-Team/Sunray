import rospy

from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


@register_case("flight.waypoint")
class WaypointMissionCase(BaseCase):
    case_type = "flight.waypoint"
    category = "flight"
    default_required_state = "airborne"
    default_resulting_state = "airborne"
    _INPUT_CANCELLED = object()
    _MIN_INPUT_ALTITUDE_M = 0.4

    @classmethod
    def _prompt_waypoint_value(cls, axis_name):
        while True:
            raw_text = input(f"desired_pos: --- {axis_name} [m] ").strip()
            if raw_text.lower() == "q":
                return None
            try:
                value = float(raw_text)
                if axis_name == "z" and value < cls._MIN_INPUT_ALTITUDE_M:
                    print(
                        f"z 不能低于 {cls._MIN_INPUT_ALTITUDE_M:.1f} m，请重新输入或按 q 退出。",
                        flush=True,
                    )
                    continue
                return value
            except ValueError:
                print(f"{axis_name} 输入无效，请输入数字或 q 退出。", flush=True)

    def _prompt_single_waypoint(self):
        print("XyzPos: 惯性系定点控制, 保持当前偏航角", flush=True)
        print("请输入期望的 [X Y Z]，输入 q 退出", flush=True)

        x_value = self._prompt_waypoint_value("x")
        if x_value is None:
            return self._INPUT_CANCELLED

        y_value = self._prompt_waypoint_value("y")
        if y_value is None:
            return self._INPUT_CANCELLED

        z_value = self._prompt_waypoint_value("z")
        if z_value is None:
            return self._INPUT_CANCELLED

        return [x_value, y_value, z_value]

    def _resolve_waypoints(self, context):
        waypoint_source = str(
            self.execution_context.params.get(
                "waypoint_source",
                context.defaults.get("waypoint_source", "list"),
            )
        ).strip().lower()

        if waypoint_source == "input":
            return waypoint_source, "runtime_input", None

        if waypoint_source != "list":
            raise ValueError(f"unsupported waypoint_source: {waypoint_source}")

        mission_key = self.execution_context.params["mission_key"]
        mission = context.missions[mission_key]
        waypoints = mission.get("waypoints", mission)
        return waypoint_source, mission_key, waypoints

    def execute(self, context, vehicle, event_logger):
        waypoint_source, mission_key, waypoints = self._resolve_waypoints(context)
        reach_radius_m = float(
            self.execution_context.params.get("reach_radius_m", context.defaults["waypoint_reach_radius_m"])
        )
        stable_time_s = float(
            self.execution_context.params.get("stable_time_s", context.defaults["waypoint_stable_time_s"])
        )
        hold_time_s = float(self.execution_context.params.get("hold_time_s", context.defaults["waypoint_hold_time_s"]))
        timeout_s = float(self.execution_context.params.get("timeout_s", context.defaults["waypoint_timeout_s"]))

        executed_waypoints = []

        if waypoint_source == "input":
            rospy.loginfo(
                "[CASE] %s: 开始航点任务 source=%s mission=%s",
                self.execution_context.case_id,
                waypoint_source,
                mission_key,
            )
            while True:
                waypoint = self._prompt_single_waypoint()
                if waypoint is self._INPUT_CANCELLED:
                    break

                idx = len(executed_waypoints)
                rospy.loginfo(
                    "[CASE] %s: 执行输入航点 %d -> %s",
                    self.execution_context.case_id,
                    idx + 1,
                    waypoint,
                )
                event_logger.log("waypoint_start", f"{self.execution_context.case_id}:{idx}:{waypoint}")
                try:
                    vehicle.goto_waypoint(waypoint, reach_radius_m, stable_time_s, hold_time_s, timeout_s)
                except RuntimeError as exc:
                    event_logger.log("waypoint_fail", f"{self.execution_context.case_id}:{idx}:{exc}")
                    rospy.logwarn("[CASE] %s: 航点失败 %s", self.execution_context.case_id, exc)
                    print("航点执行失败，请重新输入下一个航点。", flush=True)
                    continue

                event_logger.log("waypoint_end", f"{self.execution_context.case_id}:{idx}")
                executed_waypoints.append(waypoint)
        else:
            rospy.loginfo(
                "[CASE] %s: 开始航点任务 source=%s mission=%s count=%d",
                self.execution_context.case_id,
                waypoint_source,
                mission_key,
                len(waypoints),
            )
            for idx, waypoint in enumerate(waypoints):
                rospy.loginfo(
                    "[CASE] %s: 执行航点 %d/%d -> %s",
                    self.execution_context.case_id,
                    idx + 1,
                    len(waypoints),
                    waypoint,
                )
                event_logger.log("waypoint_start", f"{self.execution_context.case_id}:{idx}:{waypoint}")
                vehicle.goto_waypoint(waypoint, reach_radius_m, stable_time_s, hold_time_s, timeout_s)
                event_logger.log("waypoint_end", f"{self.execution_context.case_id}:{idx}")
                executed_waypoints.append(waypoint)

        if not executed_waypoints:
            rospy.logwarn("[CASE] %s: 用户主动退出航点输入", self.execution_context.case_id)
            return self._result(
                "unsupported",
                detail="waypoint input cancelled by user",
                metrics={
                    "waypoint_source": waypoint_source,
                    "mission_key": mission_key,
                    "waypoint_count": 0,
                    "waypoints": [],
                },
            )

        if waypoint_source == "input":
            rospy.loginfo("[CASE] %s: 用户结束航点输入，共执行 %d 个航点", self.execution_context.case_id, len(executed_waypoints))
        rospy.loginfo("[CASE] %s 完成", self.execution_context.case_id)
        return self._result(
            "pass",
            detail=f"completed {len(executed_waypoints)} waypoints",
            metrics={
                "waypoint_source": waypoint_source,
                "mission_key": mission_key,
                "waypoint_count": len(executed_waypoints),
                "waypoints": executed_waypoints,
            },
        )

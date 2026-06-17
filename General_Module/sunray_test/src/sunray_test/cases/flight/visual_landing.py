import rospy

from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


@register_case("flight.visual_landing")
class VisualLandingCase(BaseCase):
    case_type = "flight.visual_landing"
    category = "flight"
    default_required_state = "airborne"
    default_resulting_state = "landed"
    _DEFAULT_FAILURE_PATTERNS = (
        "over time",
    )

    def execute(self, context, vehicle, event_logger):
        if not context.platform.get("capabilities", {}).get("visual_landing", False):
            rospy.logwarn("[CASE] %s: 当前机型不支持视觉降落", self.execution_context.case_id)
            return self._result("unsupported", detail="platform does not support visual landing")

        launch_file = self.execution_context.params.get("launch_file", "auto_land_by_pose.launch")
        auto_takeoff = bool(
            self.execution_context.params.get("auto_takeoff", context.defaults["visual_landing_auto_takeoff"])
        )
        height_m = float(self.execution_context.params.get("height_m", context.defaults["visual_landing_height_m"]))
        launch_args = dict(self.execution_context.params.get("launch_args", {}))
        launch_args.setdefault("uav_id", context.uav_id)
        remaps = self.execution_context.params.get("remaps", [])
        pre_stop_nodes = self.execution_context.params.get("pre_stop_nodes", [])
        rospy.loginfo(
            "[CASE] %s: 开始视觉降落 launch=%s auto_takeoff=%s height=%.2f remaps=%s",
            self.execution_context.case_id,
            launch_file,
            auto_takeoff,
            height_m,
            remaps,
        )
        stop_results = {}
        if pre_stop_nodes:
            rospy.loginfo("[CASE] %s: 视觉降落前停止冲突节点 %s", self.execution_context.case_id, pre_stop_nodes)
            event_logger.log("visual_landing_pre_stop_nodes", f"{self.execution_context.case_id}:{pre_stop_nodes}")
            stop_results = vehicle.stop_nodes(pre_stop_nodes)
            rospy.sleep(0.5)

        failure_patterns = self.execution_context.params.get("failure_patterns", self._DEFAULT_FAILURE_PATTERNS)
        launch_result = vehicle.visual_land(
            launch_file,
            auto_takeoff,
            height_m,
            launch_args=launch_args,
            remaps=remaps,
        )
        return_code = int(launch_result["return_code"])
        output_lines = launch_result.get("output_lines", [])
        matched_failure_patterns = sorted(
            {
                pattern
                for pattern in failure_patterns
                for line in output_lines
                if str(pattern) in line
            }
        )
        result = "pass" if return_code == 0 and not matched_failure_patterns else "fail"
        detail = f"visual landing launch exit code: {return_code}"
        if matched_failure_patterns:
            detail += f", matched failure output: {matched_failure_patterns}"
        rospy.loginfo(
            "[CASE] %s 结束 result=%s return_code=%d matched_failure_patterns=%s",
            self.execution_context.case_id,
            result,
            return_code,
            matched_failure_patterns,
        )
        return self._result(
            result,
            detail=detail,
            metrics={
                "launch_file": launch_file,
                "height_m": height_m,
                "launch_args": launch_args,
                "remaps": launch_result.get("remaps", remaps),
                "return_code": return_code,
                "matched_failure_patterns": matched_failure_patterns,
                "pre_stop_nodes": pre_stop_nodes,
                "pre_stop_results": stop_results,
            },
        )

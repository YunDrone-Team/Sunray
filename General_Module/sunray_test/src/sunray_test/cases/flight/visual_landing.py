import rospy

from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


@register_case("flight.visual_landing")
class VisualLandingCase(BaseCase):
    case_type = "flight.visual_landing"
    category = "flight"
    default_required_state = "airborne"
    default_resulting_state = "landed"

    def execute(self, context, vehicle, event_logger):
        if not context.platform.get("capabilities", {}).get("visual_landing", False):
            rospy.logwarn("[CASE] %s: 当前机型不支持视觉降落", self.execution_context.case_id)
            return self._result("unsupported", detail="platform does not support visual landing")

        launch_file = self.execution_context.params.get("launch_file", "auto_land_by_pose.launch")
        auto_takeoff = bool(
            self.execution_context.params.get("auto_takeoff", context.defaults["visual_landing_auto_takeoff"])
        )
        height_m = float(self.execution_context.params.get("height_m", context.defaults["visual_landing_height_m"]))
        rospy.loginfo(
            "[CASE] %s: 开始视觉降落 launch=%s auto_takeoff=%s height=%.2f",
            self.execution_context.case_id,
            launch_file,
            auto_takeoff,
            height_m,
        )
        return_code = vehicle.visual_land(launch_file, auto_takeoff, height_m)
        result = "pass" if return_code == 0 else "fail"
        rospy.loginfo("[CASE] %s 结束 result=%s return_code=%d", self.execution_context.case_id, result, return_code)
        return self._result(
            result,
            detail=f"visual landing launch exit code: {return_code}",
            metrics={"launch_file": launch_file, "height_m": height_m, "return_code": return_code},
        )

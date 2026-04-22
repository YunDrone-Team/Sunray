import rospy

from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


@register_case("flight.hover")
class HoverCase(BaseCase):
    case_type = "flight.hover"
    category = "flight"
    default_required_state = "airborne"
    default_resulting_state = "airborne"

    def execute(self, context, vehicle, event_logger):
        duration_s = float(self.execution_context.params.get("duration_s", context.defaults["hover_duration_s"]))
        rospy.loginfo("[CASE] %s: 开始悬停测试 duration=%.1fs", self.execution_context.case_id, duration_s)
        vehicle.hover(duration_s)
        rospy.loginfo("[CASE] %s 完成", self.execution_context.case_id)
        return self._result("pass", detail=f"hovered for {duration_s:.1f}s", metrics={"duration_s": duration_s})

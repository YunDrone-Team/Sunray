import rospy

from sunray_test.capabilities.hardware_check import HardwareCheck
from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


RED = "\033[31m"
GREEN = "\033[32m"
RESET = "\033[0m"

@register_case("hardware.camera_alive")
class CameraAliveCase(BaseCase):
    case_type = "hardware.camera_alive"
    category = "hardware"
    default_required_state = "precheck"
    default_resulting_state = "precheck"

    def execute(self, context, vehicle, event_logger):
        topic_key = self.execution_context.params["topic_key"]
        timeout_s = float(self.execution_context.params.get("timeout_s", context.defaults["hardware_check_timeout_s"]))
        require_non_uniform_frame = bool(
            self.execution_context.params.get(
                "require_non_uniform_frame",
                True,
            )
        )
        topic = context.resolved_topics[topic_key]
        device_path = self.execution_context.params.get("device_path")
        rospy.loginfo(
            "[CASE] %s: 检查相机 topic=%s timeout=%.1f require_non_uniform_frame=%s",
            self.execution_context.case_id,
            topic,
            timeout_s,
            require_non_uniform_frame,
        )
        result, detail = HardwareCheck.camera_alive(
            topic,
            timeout_s,
            device_path=device_path,
            require_non_uniform_frame=require_non_uniform_frame,
        )
        message = f"[CASE] {self.execution_context.case_id} 结果={result} detail={detail}"
        if result == "fail":
            rospy.logerr("%s%s%s", RED, message, RESET)
        elif result == "pass":
            rospy.loginfo("%s%s%s", GREEN, message, RESET)
        else:
            rospy.loginfo(message)
        return self._result(
            result,
            detail=detail,
            metrics={"topic": topic, "require_non_uniform_frame": require_non_uniform_frame},
        )

import rospy

from sunray_test.capabilities.hardware_check import HardwareCheck
from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


RED = "\033[31m"
GREEN = "\033[32m"
RESET = "\033[0m"

@register_case("hardware.battery_voltage")
class BatteryVoltageCase(BaseCase):
    case_type = "hardware.battery_voltage"
    category = "hardware"
    default_required_state = "precheck"
    default_resulting_state = "precheck"

    def execute(self, context, vehicle, event_logger):
        topic_key = self.execution_context.params["topic_key"]
        topic = context.resolved_topics[topic_key]
        timeout_s = float(self.execution_context.params.get("timeout_s", context.defaults["hardware_check_timeout_s"]))
        threshold_v = float(
            self.execution_context.params.get("pass_threshold_v", context.defaults["battery_pass_threshold_v"])
        )
        rospy.loginfo(
            "[CASE] %s: 检查电池 topic=%s timeout=%.1f threshold=%.2fV",
            self.execution_context.case_id,
            topic,
            timeout_s,
            threshold_v,
        )
        result, detail, voltage = HardwareCheck.battery_voltage(topic, timeout_s, threshold_v)
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
            metrics={"topic": topic, "voltage_v": voltage, "pass_threshold_v": threshold_v},
        )

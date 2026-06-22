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
        sample_duration_s = float(
            self.execution_context.params.get(
                "sample_duration_s",
                context.defaults.get("camera_sample_duration_s", 1.5),
            )
        )
        min_messages = int(
            self.execution_context.params.get(
                "min_messages",
                context.defaults.get("camera_min_messages", 3),
            )
        )
        min_rate_hz = float(
            self.execution_context.params.get(
                "min_rate_hz",
                context.defaults.get("camera_min_rate_hz", 2.0),
            )
        )
        max_gap_s = float(
            self.execution_context.params.get(
                "max_gap_s",
                context.defaults.get("camera_max_gap_s", 1.0),
            )
        )
        max_identical_frame_ratio = float(
            self.execution_context.params.get(
                "max_identical_frame_ratio",
                context.defaults.get("camera_max_identical_frame_ratio", 0.95),
            )
        )
        require_timestamp_progress = bool(
            self.execution_context.params.get(
                "require_timestamp_progress",
                context.defaults.get("camera_require_timestamp_progress", True),
            )
        )
        require_frame_content_change = bool(
            self.execution_context.params.get(
                "require_frame_content_change",
                context.defaults.get("camera_require_frame_content_change", False),
            )
        )
        black_mean_threshold = float(
            self.execution_context.params.get(
                "black_mean_threshold",
                context.defaults.get("camera_black_mean_threshold", 25.0),
            )
        )
        black_dynamic_range_threshold = int(
            self.execution_context.params.get(
                "black_dynamic_range_threshold",
                context.defaults.get("camera_black_dynamic_range_threshold", 8),
            )
        )
        max_black_frame_ratio = float(
            self.execution_context.params.get(
                "max_black_frame_ratio",
                context.defaults.get("camera_max_black_frame_ratio", 0.8),
            )
        )
        topic = context.resolved_topics[topic_key]
        device_path = self.execution_context.params.get("device_path")
        rospy.loginfo(
            "[CASE] %s: 检查相机 topic=%s timeout=%.1f sample=%.1f "
            "min_messages=%d min_rate=%.1f max_gap=%.1f require_non_uniform_frame=%s",
            self.execution_context.case_id,
            topic,
            timeout_s,
            sample_duration_s,
            min_messages,
            min_rate_hz,
            max_gap_s,
            require_non_uniform_frame,
        )
        result, detail, metrics = HardwareCheck.camera_alive(
            topic,
            timeout_s,
            device_path=device_path,
            require_non_uniform_frame=require_non_uniform_frame,
            sample_duration_s=sample_duration_s,
            min_messages=min_messages,
            min_rate_hz=min_rate_hz,
            max_gap_s=max_gap_s,
            max_identical_frame_ratio=max_identical_frame_ratio,
            require_timestamp_progress=require_timestamp_progress,
            require_frame_content_change=require_frame_content_change,
            black_mean_threshold=black_mean_threshold,
            black_dynamic_range_threshold=black_dynamic_range_threshold,
            max_black_frame_ratio=max_black_frame_ratio,
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
            metrics=metrics,
        )

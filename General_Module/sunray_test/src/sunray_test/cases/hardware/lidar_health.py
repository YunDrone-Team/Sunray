import os
import subprocess
import sys

import rospy

from sunray_test.capabilities.hardware_check import HardwareCheck
from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


RED = "\033[31m"
GREEN = "\033[32m"
RESET = "\033[0m"


@register_case("hardware.lidar_health")
class LidarHealthCase(BaseCase):
    case_type = "hardware.lidar_health"
    category = "hardware"
    default_required_state = "precheck"
    default_resulting_state = "precheck"

    @staticmethod
    def _tail_text(text, max_lines=30):
        lines = str(text or "").splitlines()
        return "\n".join(lines[-max_lines:])

    def _run_mid360_ip_diagnostic(self, context):
        if str(context.environment_name).strip().lower() != "exp":
            return None

        lidar_config = context.platform.get("lidar") or {}
        if not bool(lidar_config.get("mid360_auto_check", False)):
            return None

        script_path = os.path.join(context.package_root, "scripts", "livox_mid360_autoconfig.py")
        iface = str(lidar_config.get("mid360_iface", "eth0"))
        config_path = os.path.expanduser(
            str(
                lidar_config.get(
                    "mid360_config_path",
                    "~/sunray_map/src/livox_ros_driver2/config/MID360_config.json",
                )
            )
        )
        timeout_s = float(lidar_config.get("mid360_timeout_s", 8.0))

        command = [
            sys.executable,
            script_path,
            "--iface",
            iface,
            "--timeout",
            str(timeout_s),
            "--config",
            config_path,
            "--require-match",
        ]
        rospy.loginfo("[CASE] %s: lidar health failed, checking MID360 IP", self.execution_context.case_id)
        try:
            completed = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_s + 8.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout or ""
            stderr = exc.stderr or ""
            if isinstance(stdout, bytes):
                stdout = stdout.decode(errors="replace")
            if isinstance(stderr, bytes):
                stderr = stderr.decode(errors="replace")
            return {
                "ok": False,
                "returncode": None,
                "detail": "MID360 IP diagnostic timeout",
                "iface": iface,
                "config_path": config_path,
                "stdout_tail": self._tail_text(stdout),
                "stderr_tail": self._tail_text(stderr),
            }

        ok = completed.returncode == 0
        detail = "MID360 IP matches config" if ok else "MID360 IP check failed or mismatched"
        return {
            "ok": ok,
            "returncode": completed.returncode,
            "detail": detail,
            "iface": iface,
            "config_path": config_path,
            "stdout_tail": self._tail_text(completed.stdout),
            "stderr_tail": self._tail_text(completed.stderr),
        }

    def execute(self, context, vehicle, event_logger):
        capabilities = context.platform.get("capabilities", {})
        platform_supports_lidar = capabilities.get("egoplanner", False) or "lidar_pointcloud" in context.resolved_topics

        params = self.execution_context.params
        timeout_s = float(params.get("timeout_s", context.defaults["hardware_check_timeout_s"]))
        sample_duration_s = float(params.get("sample_duration_s", 3.0))
        min_messages = int(params.get("min_messages", 5))
        min_rate_hz = float(params.get("min_rate_hz", 2.0))
        max_gap_s = float(params.get("max_gap_s", 1.0))
        min_points_per_cloud = int(params.get("min_points_per_cloud", 1000))
        min_valid_clouds = int(params.get("min_valid_clouds", 3))
        imu_pattern = str(params.get("imu_topic_pattern", "livox/imu"))
        lidar_pattern = str(params.get("lidar_topic_pattern", "livox/lidar"))

        rospy.loginfo(
            "[CASE] %s: 检查 lidar imu_pattern=%s lidar_pattern=%s duration=%.1fs",
            self.execution_context.case_id,
            imu_pattern,
            lidar_pattern,
            sample_duration_s,
        )
        result, detail, metrics = HardwareCheck.lidar_health(
            imu_pattern=imu_pattern,
            lidar_pattern=lidar_pattern,
            timeout_s=timeout_s,
            sample_duration_s=sample_duration_s,
            min_messages=min_messages,
            min_rate_hz=min_rate_hz,
            max_gap_s=max_gap_s,
            min_points_per_cloud=min_points_per_cloud,
            min_valid_clouds=min_valid_clouds,
        )
        if result == "fail":
            mid360_diagnostic = self._run_mid360_ip_diagnostic(context)
            if mid360_diagnostic is not None:
                metrics["mid360_ip_check"] = mid360_diagnostic
                detail = f"{detail}; {mid360_diagnostic['detail']}"

        if result == "fail" and not platform_supports_lidar and detail.startswith("no topic matches pattern:"):
            rospy.logwarn("[CASE] %s: 当前机型不支持 lidar 检测", self.execution_context.case_id)
            return self._result("unsupported", detail="platform does not support lidar health check", metrics=metrics)

        message = f"[CASE] {self.execution_context.case_id} 结果={result} detail={detail}"
        if result == "fail":
            rospy.logerr("%s%s%s", RED, message, RESET)
        elif result == "pass":
            rospy.loginfo("%s%s%s", GREEN, message, RESET)
        else:
            rospy.loginfo(message)
        return self._result(result, detail=detail, metrics=metrics)

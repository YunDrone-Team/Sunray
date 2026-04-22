from typing import Optional, Tuple

import os
import rospy
from sensor_msgs.msg import BatteryState, Image


class HardwareCheck:
    @staticmethod
    def _wait_for_message(topic: str, msg_type, timeout_s: float):
        try:
            return rospy.wait_for_message(topic, msg_type, timeout=timeout_s)
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("Hardware check interrupted") from exc
        except rospy.ROSException:
            if rospy.is_shutdown():
                raise KeyboardInterrupt("Hardware check interrupted by ROS shutdown")
            return None

    @staticmethod
    def _image_has_variation(image_msg: Image) -> bool:
        if not image_msg.data:
            return False

        first_value = image_msg.data[0]
        for value in image_msg.data[1:]:
            if value != first_value:
                return True
        return False

    @staticmethod
    def camera_alive(
        topic: str,
        timeout_s: float,
        device_path: Optional[str] = None,
        require_non_uniform_frame: bool = True,
    ) -> Tuple[str, str]:
        if device_path and not os.path.exists(device_path):
            return "fail", f"device not found: {device_path}"

        image_msg = HardwareCheck._wait_for_message(topic, Image, timeout_s)
        if image_msg is None:
            return "fail", f"no image from {topic}"

        if require_non_uniform_frame and not HardwareCheck._image_has_variation(image_msg):
            return "fail", f"image from {topic} is uniform; camera may be stuck"

        return "pass", f"topic alive: {topic}"

    @staticmethod
    def battery_voltage(topic: str, timeout_s: float, pass_threshold_v: float) -> Tuple[str, str, Optional[float]]:
        try:
            msg = rospy.wait_for_message(topic, BatteryState, timeout=timeout_s)
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("Battery check interrupted") from exc
        except rospy.ROSException:
            if rospy.is_shutdown():
                raise KeyboardInterrupt("Battery check interrupted by ROS shutdown")
            return "fail", f"no battery message from {topic}", None
        voltage = msg.voltage
        if voltage >= pass_threshold_v:
            return "pass", f"battery voltage {voltage:.2f}V", voltage
        return "fail", f"battery voltage {voltage:.2f}V < {pass_threshold_v:.2f}V", voltage

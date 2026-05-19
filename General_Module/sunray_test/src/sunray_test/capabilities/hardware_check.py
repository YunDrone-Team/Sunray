from typing import Any, Dict, List, Optional, Tuple

import os
import rospy
import roslib.message
from rospy.msg import AnyMsg
from sensor_msgs.msg import BatteryState, Image, Imu, PointCloud2


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

    @staticmethod
    def topic_alive(topic: str, timeout_s: float) -> Tuple[str, str]:
        msg = HardwareCheck._wait_for_message(topic, AnyMsg, timeout_s)
        if msg is None:
            return "fail", f"no message from {topic}"
        return "pass", f"topic alive: {topic}"

    @staticmethod
    def _normalize_topic(value: str) -> str:
        return value.strip().strip("/")

    @staticmethod
    def _find_topic_by_pattern(pattern: str) -> Tuple[Optional[str], Optional[str], List[str]]:
        normalized_pattern = HardwareCheck._normalize_topic(pattern)
        published = rospy.get_published_topics()
        matches: List[Tuple[str, str]] = []
        for topic, topic_type in published:
            normalized_topic = HardwareCheck._normalize_topic(topic)
            if normalized_topic == normalized_pattern or normalized_topic.endswith(normalized_pattern):
                matches.append((topic, topic_type))
            elif normalized_pattern in normalized_topic:
                matches.append((topic, topic_type))

        if not matches:
            return None, None, []

        exact_suffix_matches = [
            item for item in matches if HardwareCheck._normalize_topic(item[0]).endswith(normalized_pattern)
        ]
        selected = sorted(exact_suffix_matches or matches, key=lambda item: item[0])[0]
        return selected[0], selected[1], [item[0] for item in matches]

    @staticmethod
    def _message_class(topic_type: str, fallback):
        if topic_type:
            msg_cls = roslib.message.get_message_class(topic_type)
            if msg_cls is not None:
                return msg_cls
        return fallback

    @staticmethod
    def _point_count(message: Any) -> Optional[int]:
        if isinstance(message, PointCloud2):
            return int(message.width) * int(message.height)
        if hasattr(message, "point_num"):
            return int(getattr(message, "point_num"))
        if hasattr(message, "points"):
            try:
                return len(getattr(message, "points"))
            except TypeError:
                return None
        return None

    @staticmethod
    def _sample_topic(topic: str, topic_type: str, fallback_type, timeout_s: float, sample_duration_s: float) -> Dict[str, Any]:
        samples: List[Tuple[float, Any]] = []
        msg_cls = HardwareCheck._message_class(topic_type, fallback_type)

        def callback(message):
            samples.append((rospy.Time.now().to_sec(), message))

        subscriber = rospy.Subscriber(topic, msg_cls, callback, queue_size=20)
        try:
            first_deadline = rospy.Time.now().to_sec() + timeout_s
            while not rospy.is_shutdown() and not samples and rospy.Time.now().to_sec() < first_deadline:
                rospy.sleep(0.05)

            if not samples:
                return {"ok": False, "detail": f"no message from {topic}", "message_count": 0}

            sample_end = samples[0][0] + sample_duration_s
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() < sample_end:
                rospy.sleep(0.05)
        finally:
            subscriber.unregister()

        timestamps = [timestamp for timestamp, _ in samples]
        intervals = [timestamps[index] - timestamps[index - 1] for index in range(1, len(timestamps))]
        elapsed_s = max(timestamps[-1] - timestamps[0], 0.0)
        rate_hz = (len(timestamps) - 1) / elapsed_s if len(timestamps) > 1 and elapsed_s > 0 else 0.0
        return {
            "ok": True,
            "detail": "sampled",
            "message_count": len(samples),
            "rate_hz": rate_hz,
            "max_gap_s": max(intervals) if intervals else None,
            "messages": [message for _, message in samples],
        }

    @staticmethod
    def lidar_health(
        imu_pattern: str,
        lidar_pattern: str,
        timeout_s: float,
        sample_duration_s: float,
        min_messages: int,
        min_rate_hz: float,
        max_gap_s: float,
        min_points_per_cloud: int,
        min_valid_clouds: int,
    ) -> Tuple[str, str, Dict[str, Any]]:
        imu_topic, imu_type, imu_matches = HardwareCheck._find_topic_by_pattern(imu_pattern)
        lidar_topic, lidar_type, lidar_matches = HardwareCheck._find_topic_by_pattern(lidar_pattern)
        metrics: Dict[str, Any] = {
            "imu_pattern": imu_pattern,
            "lidar_pattern": lidar_pattern,
            "imu_matches": imu_matches,
            "lidar_matches": lidar_matches,
            "imu_topic": imu_topic,
            "lidar_topic": lidar_topic,
            "imu_type": imu_type,
            "lidar_type": lidar_type,
        }

        if not imu_topic:
            return "fail", f"no topic matches pattern: {imu_pattern}", metrics
        if not lidar_topic:
            return "fail", f"no topic matches pattern: {lidar_pattern}", metrics

        imu_sample = HardwareCheck._sample_topic(imu_topic, imu_type or "", Imu, timeout_s, sample_duration_s)
        lidar_sample = HardwareCheck._sample_topic(lidar_topic, lidar_type or "", PointCloud2, timeout_s, sample_duration_s)
        metrics["imu"] = {key: value for key, value in imu_sample.items() if key != "messages"}
        metrics["lidar"] = {key: value for key, value in lidar_sample.items() if key != "messages"}

        for label, sample in (("imu", imu_sample), ("lidar", lidar_sample)):
            if not sample.get("ok"):
                return "fail", f"{label} unstable: {sample.get('detail', 'no data')}", metrics
            if int(sample["message_count"]) < min_messages:
                return "fail", f"{label} message count {sample['message_count']} < {min_messages}", metrics
            if float(sample["rate_hz"]) < min_rate_hz:
                return "fail", f"{label} rate {sample['rate_hz']:.2f}Hz < {min_rate_hz:.2f}Hz", metrics
            max_gap = sample.get("max_gap_s")
            if max_gap is not None and float(max_gap) > max_gap_s:
                return "fail", f"{label} max gap {max_gap:.2f}s > {max_gap_s:.2f}s", metrics

        point_counts = [
            point_count
            for point_count in (HardwareCheck._point_count(message) for message in lidar_sample.get("messages", []))
            if point_count is not None
        ]
        metrics["lidar"]["point_counts"] = point_counts
        if not point_counts:
            return "fail", f"lidar message type {lidar_type} does not expose point count", metrics

        valid_clouds = [count for count in point_counts if count >= min_points_per_cloud]
        metrics["lidar"]["min_point_count"] = min(point_counts)
        metrics["lidar"]["max_point_count"] = max(point_counts)
        metrics["lidar"]["avg_point_count"] = sum(point_counts) / float(len(point_counts))
        metrics["lidar"]["valid_cloud_count"] = len(valid_clouds)

        if len(valid_clouds) < min_valid_clouds:
            return (
                "fail",
                f"valid lidar clouds {len(valid_clouds)} < {min_valid_clouds}, min points={min_points_per_cloud}",
                metrics,
            )

        return "pass", f"lidar healthy: imu={imu_topic}, lidar={lidar_topic}", metrics

import hashlib
import time
from collections import Counter
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
    def _image_stamp_s(image_msg: Image) -> float:
        stamp = getattr(getattr(image_msg, "header", None), "stamp", None)
        if stamp is None:
            return 0.0
        try:
            return float(stamp.to_sec())
        except Exception:
            return 0.0

    @staticmethod
    def _image_hash(image_msg: Image) -> str:
        try:
            return hashlib.sha1(image_msg.data).hexdigest()
        except TypeError:
            return hashlib.sha1(bytes(image_msg.data)).hexdigest()

    @staticmethod
    def _image_stats(image_msg: Image, max_samples: int = 4096) -> Tuple[float, int]:
        data = image_msg.data
        if not data:
            return 0.0, 0
        if max_samples <= 0 or len(data) <= max_samples:
            sample = bytes(data)
        else:
            stride = max(1, len(data) // max_samples)
            sample = bytes(data[::stride])
        if not sample:
            return 0.0, 0
        min_value = min(sample)
        max_value = max(sample)
        return sum(sample) / float(len(sample)), max_value - min_value

    @staticmethod
    def _sample_image_topic(topic: str, timeout_s: float, sample_duration_s: float) -> Dict[str, Any]:
        samples: List[Tuple[float, Image]] = []

        def callback(message):
            samples.append((time.monotonic(), message))

        subscriber = rospy.Subscriber(topic, Image, callback, queue_size=20)
        try:
            first_deadline = time.monotonic() + timeout_s
            while not rospy.is_shutdown() and not samples and time.monotonic() < first_deadline:
                rospy.sleep(0.03)

            if not samples:
                return {"ok": False, "detail": f"no image from {topic}", "message_count": 0}

            sample_end = samples[0][0] + sample_duration_s
            while not rospy.is_shutdown() and time.monotonic() < sample_end:
                rospy.sleep(0.03)
        finally:
            subscriber.unregister()

        receive_times = [timestamp for timestamp, _ in samples]
        intervals = [receive_times[index] - receive_times[index - 1] for index in range(1, len(receive_times))]
        elapsed_s = max(receive_times[-1] - receive_times[0], 0.0)
        rate_hz = (len(receive_times) - 1) / elapsed_s if len(receive_times) > 1 and elapsed_s > 0 else 0.0
        return {
            "ok": True,
            "detail": "sampled",
            "message_count": len(samples),
            "rate_hz": rate_hz,
            "max_gap_s": max(intervals) if intervals else None,
            "messages": [message for _, message in samples],
        }

    @staticmethod
    def camera_alive(
        topic: str,
        timeout_s: float,
        device_path: Optional[str] = None,
        require_non_uniform_frame: bool = True,
        sample_duration_s: float = 1.5,
        min_messages: int = 3,
        min_rate_hz: float = 2.0,
        max_gap_s: float = 1.0,
        max_identical_frame_ratio: float = 0.95,
        require_timestamp_progress: bool = True,
        require_frame_content_change: bool = False,
        black_mean_threshold: float = 25.0,
        black_dynamic_range_threshold: int = 8,
        max_black_frame_ratio: float = 0.8,
    ) -> Tuple[str, str, Dict[str, Any]]:
        sample_duration_s = max(0.1, float(sample_duration_s))
        min_messages = max(1, int(min_messages))
        min_rate_hz = max(0.0, float(min_rate_hz))
        max_gap_s = max(0.0, float(max_gap_s))
        max_identical_frame_ratio = min(1.0, max(0.0, float(max_identical_frame_ratio)))
        black_mean_threshold = max(0.0, float(black_mean_threshold))
        black_dynamic_range_threshold = max(0, int(black_dynamic_range_threshold))
        max_black_frame_ratio = min(1.0, max(0.0, float(max_black_frame_ratio)))
        metrics: Dict[str, Any] = {
            "topic": topic,
            "timeout_s": timeout_s,
            "sample_duration_s": sample_duration_s,
            "min_messages": min_messages,
            "min_rate_hz": min_rate_hz,
            "max_gap_s": max_gap_s,
            "require_non_uniform_frame": require_non_uniform_frame,
            "max_identical_frame_ratio": max_identical_frame_ratio,
            "require_timestamp_progress": require_timestamp_progress,
            "require_frame_content_change": require_frame_content_change,
            "black_mean_threshold": black_mean_threshold,
            "black_dynamic_range_threshold": black_dynamic_range_threshold,
            "max_black_frame_ratio": max_black_frame_ratio,
        }
        if device_path and not os.path.exists(device_path):
            metrics["device_path"] = device_path
            return "fail", f"device not found: {device_path}", metrics

        sample = HardwareCheck._sample_image_topic(topic, timeout_s, sample_duration_s)
        metrics.update({key: value for key, value in sample.items() if key != "messages"})
        if not sample.get("ok"):
            return "fail", str(sample.get("detail") or f"no image from {topic}"), metrics

        messages = sample.get("messages", [])
        image_messages = [message for message in messages if isinstance(message, Image)]
        metrics["message_count"] = len(image_messages)
        if len(image_messages) < min_messages:
            return "fail", f"camera message count {len(image_messages)} < {min_messages} on {topic}", metrics

        rate_hz = float(metrics.get("rate_hz") or 0.0)
        metrics["rate_hz"] = round(rate_hz, 2)
        if rate_hz < min_rate_hz:
            return "fail", f"camera rate {rate_hz:.2f}Hz < {min_rate_hz:.2f}Hz on {topic}", metrics

        observed_max_gap = metrics.get("max_gap_s")
        if observed_max_gap is not None:
            observed_max_gap = float(observed_max_gap)
            metrics["max_gap_s"] = round(observed_max_gap, 3)
            if observed_max_gap > max_gap_s:
                return "fail", f"camera max gap {observed_max_gap:.2f}s > {max_gap_s:.2f}s on {topic}", metrics

        uniform_count = sum(1 for message in image_messages if not HardwareCheck._image_has_variation(message))
        metrics["uniform_frame_count"] = uniform_count
        image_stats = [HardwareCheck._image_stats(message) for message in image_messages]
        black_frame_count = sum(
            1
            for mean_value, dynamic_range in image_stats
            if mean_value <= black_mean_threshold and dynamic_range <= black_dynamic_range_threshold
        )
        black_frame_ratio = black_frame_count / float(len(image_stats)) if image_stats else 0.0
        metrics["image_mean_min"] = round(min((item[0] for item in image_stats), default=0.0), 2)
        metrics["image_mean_max"] = round(max((item[0] for item in image_stats), default=0.0), 2)
        metrics["image_dynamic_range_max"] = max((item[1] for item in image_stats), default=0)
        metrics["black_frame_count"] = black_frame_count
        metrics["black_frame_ratio"] = round(black_frame_ratio, 3)
        if black_frame_ratio > max_black_frame_ratio:
            return "fail", f"sampled images from {topic} are black; camera may need restart", metrics

        if require_non_uniform_frame and uniform_count == len(image_messages):
            return "fail", f"all sampled images from {topic} are uniform; camera may be stuck", metrics

        hashes = [HardwareCheck._image_hash(message) for message in image_messages]
        unique_hash_count = len(set(hashes))
        dominant_hash_count = Counter(hashes).most_common(1)[0][1] if hashes else 0
        identical_frame_ratio = dominant_hash_count / float(len(hashes)) if hashes else 0.0
        metrics["unique_frame_count"] = unique_hash_count
        metrics["dominant_frame_count"] = dominant_hash_count
        metrics["identical_frame_ratio"] = round(identical_frame_ratio, 3)

        stamps = [HardwareCheck._image_stamp_s(message) for message in image_messages]
        positive_stamps = [stamp for stamp in stamps if stamp > 0.0]
        unique_stamp_count = len(set(positive_stamps))
        metrics["unique_stamp_count"] = unique_stamp_count
        metrics["timestamp_progress"] = unique_stamp_count > 1

        if require_timestamp_progress and positive_stamps and unique_stamp_count <= 1:
            return "fail", f"image timestamp does not progress on {topic}; camera stream may be stuck", metrics

        if require_frame_content_change and identical_frame_ratio >= max_identical_frame_ratio:
            return "fail", f"sampled images from {topic} are identical; camera may be frozen", metrics

        return "pass", f"camera healthy: {topic}, {len(image_messages)} frames, {rate_hz:.2f}Hz", metrics

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

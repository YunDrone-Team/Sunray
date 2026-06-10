#!/usr/bin/env python3
"""Restart camera ROS nodes when their image topics stop publishing or go black."""

import argparse
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rospy
from sensor_msgs.msg import Image


@dataclass
class CameraState:
    role: str
    node_name: str
    topic: str
    last_stamp: float
    last_restart: float
    frames: int = 0
    black_frames: int = 0
    last_mean: float = 0.0
    last_dynamic_range: int = 0
    restarting: bool = False


def _split_csv(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _rosnode_kill(node_name: str, timeout_s: float) -> bool:
    try:
        result = subprocess.run(
            ["rosnode", "kill", node_name],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        rospy.logwarn("[camera_watchdog] rosnode kill timeout: %s", node_name)
        return False
    if result.returncode == 0:
        return True
    detail = (result.stderr or result.stdout or "").strip()
    if detail:
        rospy.logwarn("[camera_watchdog] rosnode kill %s failed rc=%s: %s", node_name, result.returncode, detail)
    else:
        rospy.logwarn("[camera_watchdog] rosnode kill %s failed rc=%s", node_name, result.returncode)
    return False


def _sample_image_stats(msg: Image, max_samples: int) -> tuple:
    data = msg.data
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
    mean_value = sum(sample) / float(len(sample))
    return mean_value, max_value - min_value


class CameraWatchdog:
    def __init__(
        self,
        cameras: List[CameraState],
        stale_timeout_s: float,
        check_period_s: float,
        startup_grace_s: float,
        restart_cooldown_s: float,
        kill_timeout_s: float,
        black_frame_check_enabled: bool,
        black_mean_threshold: float,
        black_dynamic_range_threshold: int,
        black_consecutive_frames: int,
        black_sample_bytes: int,
    ):
        self._cameras: Dict[str, CameraState] = {camera.role: camera for camera in cameras}
        self._stale_timeout_s = stale_timeout_s
        self._startup_grace_s = startup_grace_s
        self._restart_cooldown_s = restart_cooldown_s
        self._kill_timeout_s = kill_timeout_s
        self._black_frame_check_enabled = black_frame_check_enabled
        self._black_mean_threshold = black_mean_threshold
        self._black_dynamic_range_threshold = black_dynamic_range_threshold
        self._black_consecutive_frames = black_consecutive_frames
        self._black_sample_bytes = black_sample_bytes
        self._lock = threading.Lock()
        self._subscribers = [
            rospy.Subscriber(camera.topic, Image, self._on_image, callback_args=camera.role, queue_size=1)
            for camera in cameras
        ]
        self._timer = rospy.Timer(rospy.Duration.from_sec(check_period_s), self._on_timer)

    def _on_image(self, msg: Image, role: str) -> None:
        now = time.monotonic()
        mean_value = 0.0
        dynamic_range = 0
        black_frame = False
        if self._black_frame_check_enabled:
            mean_value, dynamic_range = _sample_image_stats(msg, self._black_sample_bytes)
            black_frame = (
                mean_value <= self._black_mean_threshold
                and dynamic_range <= self._black_dynamic_range_threshold
            )
        with self._lock:
            camera = self._cameras.get(role)
            if not camera:
                return
            camera.last_stamp = now
            camera.frames += 1
            camera.last_mean = mean_value
            camera.last_dynamic_range = dynamic_range
            if black_frame:
                camera.black_frames += 1
            else:
                camera.black_frames = 0
            if camera.restarting:
                rospy.loginfo("[camera_watchdog] %s recovered on %s", role, camera.topic)
            camera.restarting = False

    def _on_timer(self, _event) -> None:
        now = time.monotonic()
        restart_list: List[CameraState] = []
        restart_roles = set()
        with self._lock:
            for camera in self._cameras.values():
                age = now - camera.last_stamp
                since_restart = now - camera.last_restart
                if age < self._stale_timeout_s:
                    continue
                if since_restart < self._startup_grace_s:
                    continue
                if since_restart < self._restart_cooldown_s:
                    continue
                camera.last_restart = now
                camera.restarting = True
                restart_list.append(camera)
                restart_roles.add(camera.role)

            for camera in self._cameras.values():
                if not self._black_frame_check_enabled:
                    continue
                if camera.role in restart_roles:
                    continue
                if camera.black_frames < self._black_consecutive_frames:
                    continue
                since_restart = now - camera.last_restart
                if since_restart < self._startup_grace_s:
                    continue
                if since_restart < self._restart_cooldown_s:
                    continue
                camera.last_restart = now
                camera.restarting = True
                restart_list.append(camera)

        for camera in restart_list:
            age_s = max(0.0, now - camera.last_stamp)
            if self._black_frame_check_enabled and camera.black_frames >= self._black_consecutive_frames:
                rospy.logwarn(
                    "[camera_watchdog] %s image black for %d consecutive frames on %s "
                    "(mean=%.1f range=%d), restarting node %s",
                    camera.role,
                    camera.black_frames,
                    camera.topic,
                    camera.last_mean,
                    camera.last_dynamic_range,
                    camera.node_name,
                )
            else:
                rospy.logwarn(
                    "[camera_watchdog] %s image stale %.1fs on %s, restarting node %s",
                    camera.role,
                    age_s,
                    camera.topic,
                    camera.node_name,
                )
            _rosnode_kill(camera.node_name, timeout_s=self._kill_timeout_s)


def _build_cameras(roles: List[str], nodes: List[str], topics: List[str]) -> List[CameraState]:
    if not (len(roles) == len(nodes) == len(topics)):
        raise ValueError("roles, nodes, and topics must have the same item count")
    now = time.monotonic()
    return [
        CameraState(role=role, node_name=node, topic=topic, last_stamp=now, last_restart=now)
        for role, node, topic in zip(roles, nodes, topics)
    ]


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Restart camera nodes when image topics go stale")
    parser.add_argument("--roles", default="front,down")
    parser.add_argument("--nodes", default="/web_cam_front,/web_cam")
    parser.add_argument("--topics", default="/web_cam_front/image_raw,/web_cam/image_raw")
    parser.add_argument("--stale-timeout", type=float, default=3.0)
    parser.add_argument("--check-period", type=float, default=1.0)
    parser.add_argument("--startup-grace", type=float, default=8.0)
    parser.add_argument("--restart-cooldown", type=float, default=10.0)
    parser.add_argument("--kill-timeout", type=float, default=2.0)
    parser.add_argument("--disable-black-frame-check", action="store_true")
    parser.add_argument("--black-mean-threshold", type=float, default=25.0)
    parser.add_argument("--black-dynamic-range-threshold", type=int, default=8)
    parser.add_argument("--black-consecutive-frames", type=int, default=30)
    parser.add_argument("--black-sample-bytes", type=int, default=4096)
    return parser.parse_args(argv)


def _filtered_cli_args(argv: Optional[List[str]] = None) -> List[str]:
    filtered = rospy.myargv(argv=argv)
    if filtered and not filtered[0].startswith("-"):
        return filtered[1:]
    return filtered


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(_filtered_cli_args(argv))
    rospy.init_node("camera_watchdog", anonymous=False)
    cameras = _build_cameras(_split_csv(args.roles), _split_csv(args.nodes), _split_csv(args.topics))
    rospy.loginfo(
        "[camera_watchdog] watching %s stale_timeout=%.1fs restart_cooldown=%.1fs "
        "black_check=%s black_mean<=%.1f black_range<=%d black_frames=%d",
        ", ".join(f"{camera.role}:{camera.topic}->{camera.node_name}" for camera in cameras),
        args.stale_timeout,
        args.restart_cooldown,
        not args.disable_black_frame_check,
        args.black_mean_threshold,
        args.black_dynamic_range_threshold,
        args.black_consecutive_frames,
    )
    CameraWatchdog(
        cameras=cameras,
        stale_timeout_s=max(0.5, args.stale_timeout),
        check_period_s=max(0.2, args.check_period),
        startup_grace_s=max(0.0, args.startup_grace),
        restart_cooldown_s=max(1.0, args.restart_cooldown),
        kill_timeout_s=max(0.2, args.kill_timeout),
        black_frame_check_enabled=not args.disable_black_frame_check,
        black_mean_threshold=max(0.0, args.black_mean_threshold),
        black_dynamic_range_threshold=max(0, args.black_dynamic_range_threshold),
        black_consecutive_frames=max(1, args.black_consecutive_frames),
        black_sample_bytes=max(0, args.black_sample_bytes),
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

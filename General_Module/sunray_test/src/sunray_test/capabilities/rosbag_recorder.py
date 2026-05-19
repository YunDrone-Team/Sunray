import os
import subprocess
import time
from datetime import datetime
from typing import List

import rospy


class RosbagRecorder:
    def __init__(self) -> None:
        self._proc = None
        self.bag_path = ""

    def start(self, output_dir: str, bag_prefix: str, topics: List[str]) -> str:
        if self._proc is not None:
            return self.bag_path
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bags_dir = os.path.join(output_dir, "bags")
        os.makedirs(bags_dir, exist_ok=True)
        self.bag_path = os.path.join(bags_dir, f"{bag_prefix}_{timestamp}.bag")
        cmd = ["rosbag", "record", "-O", self.bag_path] + topics
        self._proc = subprocess.Popen(cmd)
        rospy.loginfo("rosbag recording started: %s", self.bag_path)
        return self.bag_path

    def stop(self) -> None:
        if self._proc is None:
            return
        self._proc.terminate()
        self._proc.wait()
        self._wait_for_bag_ready()
        rospy.loginfo("rosbag recording stopped")
        self._proc = None

    def _wait_for_bag_ready(self, timeout_s: float = 8.0, stable_checks: int = 3, interval_s: float = 0.2) -> None:
        if not self.bag_path:
            return

        deadline = time.time() + timeout_s
        last_size = None
        unchanged_count = 0
        while time.time() < deadline and not rospy.is_shutdown():
            size = os.path.getsize(self.bag_path) if os.path.isfile(self.bag_path) else 0
            unchanged_count = unchanged_count + 1 if size > 0 and size == last_size else 0
            if unchanged_count >= stable_checks:
                return
            last_size = size
            time.sleep(interval_s)

        rospy.logwarn("rosbag file is not stable before report generation: %s", self.bag_path)

import os
import subprocess
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
        rospy.loginfo("rosbag recording stopped")
        self._proc = None

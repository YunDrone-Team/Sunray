import json
from datetime import datetime
from typing import Dict, List

import rospy


class EventLogger:
    def __init__(self) -> None:
        self._events: List[Dict[str, str]] = []

    def log(self, event: str, detail: str = "") -> None:
        wall_timestamp = datetime.now().timestamp()
        ros_timestamp = rospy.Time.now().to_sec() if rospy.core.is_initialized() else 0.0
        timestamp = ros_timestamp if ros_timestamp > 0 else wall_timestamp
        self._events.append(
            {
                "event": event,
                "detail": detail,
                "timestamp": f"{timestamp:.6f}",
                "time_str": datetime.fromtimestamp(wall_timestamp).strftime("%Y-%m-%d %H:%M:%S.%f"),
                "wall_timestamp": f"{wall_timestamp:.6f}",
                "ros_timestamp": f"{ros_timestamp:.6f}" if ros_timestamp > 0 else "",
            }
        )

    def write_jsonl(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as handle:
            for event in self._events:
                handle.write(json.dumps(event, ensure_ascii=False) + "\n")

    @property
    def events(self) -> List[Dict[str, str]]:
        return list(self._events)

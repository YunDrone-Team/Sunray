#!/usr/bin/env python3
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]

PAGES = [
    ("content/01-getting-started.md", "入门与仓库地图", ["intro", "quick-map", "learn-path", "build-run"]),
    ("content/02-ros-interface.md", "ROS 运行链路与消息接口", ["ros-graph", "messages", "uav", "ugv", "external-position", "planner"]),
    ("content/03-core-packages.md", "重点包深入：common 与 UAV 控制", ["deep-common", "deep-uav-control"]),
    ("content/04-tutorials.md", "重点包深入：sunray_tutorial", ["deep-tutorial", "new-task"]),
    ("content/05-planner-ego.md", "规划工具与 EGO 接入", ["deep-planner-utils", "deep-ego"]),
    ("content/06-scripts-ground-station.md", "快速启动脚本与地面站", ["quick-scripts"]),
    ("content/07-extensions.md", "扩展模块", ["vision", "formation", "gimbal-media", "communication", "fmt"]),
    ("content/08-debug-safety-appendix.md", "仿真、调试与安全速查", ["simulation", "debug", "safety", "appendix"]),
]


def main():
    data = []
    for file_name, title, sections in PAGES:
        path = ROOT / file_name
        data.append(
            {
                "file": file_name,
                "title": title,
                "sections": sections,
                "content": path.read_text(encoding="utf-8"),
            }
        )

    output = "window.SUNRAY_DOCS = "
    output += json.dumps(data, ensure_ascii=False, indent=2)
    output += ";\n"
    (ROOT / "assets" / "doc-data.js").write_text(output, encoding="utf-8")


if __name__ == "__main__":
    main()

from jinja2 import Template
from datetime import datetime
import json
import os

def calc_stats(steps):
    total = len(steps)
    passed = sum(1 for c in steps if c["result"] == "pass")
    failed = sum(1 for c in steps if c["result"] == "fail")
    error = sum(1 for c in steps if c["result"] == "error")
    return total, passed, failed, error


def generate_report(
    template_path = os.path.expanduser("~/Sunray/General_Module/sunray_test/config/template.html"),
    json_path = os.path.expanduser("~/Sunray/tests/output/test_result.json"),
    output_path = os.path.expanduser("~/Sunray/tests/output/report.html")
):
    # ===== 读取模板 =====
    with open(template_path, "r", encoding="utf-8") as f:
        template = Template(f.read())

    # ===== 读取测试数据 =====
    with open(json_path, "r", encoding="utf-8") as f:
        test_data = json.load(f)
        test_info = test_data.get("test_info", {})

    # ===== 测试结果 =====
    groups = []

    # ===== 硬件测试 =====
    hardware_cases = []
    hw = test_data.get("hardware_test", {})

    camera_steps = [
        {"name": "前视摄像头", "result": hw.get("front_cam", "error")},
        {"name": "下视摄像头", "result": hw.get("down_cam", "error")},
    ]

    hardware_cases.append({
        "name": "摄像头",
        "steps": camera_steps
    })

    hardware_cases.append({
        "name": "电池电压",
        "result": hw.get("battery", "error")
    })

    groups.append({
        "name": "硬件测试",
        "cases": hardware_cases
    })

    # ===== 功能测试 =====
    function_cases = []
    fn = test_data.get("function_test", {})

    function_cases.append({
        "name": "MOCAP动捕定位悬停",
        "result": fn.get("mocap_flight", "error"),
        "score": fn.get("mocap_score", 0)
    })

    function_cases.append({
        "name": "指点飞行",
        "result": fn.get("waypoint_flight", "error"),
        "score": fn.get("waypoint_score", 0)
    })

    function_cases.append({
        "name": "apriltags识别降落",
        "result": fn.get("apriltags_landmark", "error"),
        "score": fn.get("apriltags_score", 0)
    })

    groups.append({
        "name": "功能测试",
        "cases": function_cases
    })

    # ===== 统计 =====
    all_items = []

    for g in groups:
        group_items = [
            step
            for case in g["cases"]
            for step in (
                case["steps"] if "steps" in case and case["steps"]
                else [{"name": case["name"], "result": case["result"]}]
            )
        ]

        total, passed, failed, error = calc_stats(group_items)

        g.update({
            "total": total,
            "passed": passed,
            "failed": failed,
            "error": error
        })

        all_items.extend(group_items)

    # ===== 总统计 =====
    total, passed, failed, error = calc_stats(all_items)

    # ===== 渲染数据 =====
    data = {
        "series": "基础款",
        "sn": test_info.get("sn", "unknown"),
        "title": "Sunray150测试报告",
        "start_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "duration": test_info.get("duration", "00:00:00"),
        "human": test_info.get("tester", "unknown"),
        "status": "测试不通过" if failed or error else "测试通过",
        "groups": groups,
        "total": total,
        "passed": passed,
        "failed": failed,
        "error": error
    }

    # ===== 渲染HTML =====
    html = template.render(**data)

    # ===== 写入文件 =====
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)

    print(f"报告生成成功：{output_path}")

if __name__ == "__main__":
    generate_report()
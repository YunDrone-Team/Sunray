from typing import Any, Dict, List

from sunray_test.reports.renderers.common import (
    description_list,
    escape,
    format_display_text,
    format_duration,
    normalize_status,
    score_display,
    status_badge,
)
from sunray_test.reports.renderers.flight import render_flight_section_content


TITLE_TO_CASE_PREFIX = {
    "悬停指标": ("hover_stability", "hover"),
    "航点飞行指标": ("waypoint_flight", "waypoint"),
    "EGO自主规划指标": ("ego_goal_flight", "ego_goal"),
    "视觉降落指标": ("visual_landing",),
}


def _fmt_float(value: Any, digits: int = 2) -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{float(value):.{digits}f}"
    return "-"


def _hardware_metrics_for_display(case: Dict[str, Any]) -> Dict[str, Any]:
    metrics = case.get("metrics", {})
    if not isinstance(metrics, dict):
        return {}

    case_id = str(case.get("id", ""))
    if case_id.startswith(("front_camera", "down_camera")):
        return {
            "topic": metrics.get("topic", "-"),
            "message_count": metrics.get("message_count", "-"),
            "rate_hz": _fmt_float(metrics.get("rate_hz")),
            "max_gap_s": _fmt_float(metrics.get("max_gap_s"), 3),
            "uniform_frame_count": metrics.get("uniform_frame_count", "-"),
            "unique_frame_count": metrics.get("unique_frame_count", "-"),
            "identical_frame_ratio": _fmt_float(metrics.get("identical_frame_ratio"), 3),
            "black_frame_count": metrics.get("black_frame_count", "-"),
            "black_frame_ratio": _fmt_float(metrics.get("black_frame_ratio"), 3),
            "image_mean_min": _fmt_float(metrics.get("image_mean_min")),
            "image_mean_max": _fmt_float(metrics.get("image_mean_max")),
            "image_dynamic_range_max": metrics.get("image_dynamic_range_max", "-"),
            "timestamp_progress": metrics.get("timestamp_progress", "-"),
        }

    if case_id.startswith("lidar_health"):
        imu = metrics.get("imu", {}) if isinstance(metrics.get("imu"), dict) else {}
        lidar = metrics.get("lidar", {}) if isinstance(metrics.get("lidar"), dict) else {}
        message_count = lidar.get("message_count")
        valid_cloud_count = lidar.get("valid_cloud_count")
        return {
            "imu_topic": metrics.get("imu_topic", "-"),
            "lidar_topic": metrics.get("lidar_topic", "-"),
            "imu_rate_hz": _fmt_float(imu.get("rate_hz")),
            "lidar_rate_hz": _fmt_float(lidar.get("rate_hz")),
            "max_gap_s": _fmt_float(lidar.get("max_gap_s"), 3),
            "min_point_count": lidar.get("min_point_count", "-"),
            "avg_point_count": _fmt_float(lidar.get("avg_point_count")),
            "valid_clouds": (
                f"{valid_cloud_count}/{message_count}"
                if valid_cloud_count is not None and message_count
                else "-"
            ),
        }

    return metrics


def _case_metrics_for_display(case: Dict[str, Any]) -> Dict[str, Any]:
    if case.get("category") == "hardware":
        return _hardware_metrics_for_display(case)
    metrics = case.get("metrics", {})
    if not isinstance(metrics, dict):
        return {}
    hidden_keys = {"remaps", "pre_stop_results"}
    return {key: value for key, value in metrics.items() if key not in hidden_keys}


def _pass_score_threshold(grade_thresholds: List[Dict[str, Any]]) -> float:
    positive_thresholds = [
        float(item.get("min", 0))
        for item in grade_thresholds
        if isinstance(item, dict) and float(item.get("min", 0)) > 0
    ]
    return min(positive_thresholds) if positive_thresholds else 60.0


def _display_result(case: Dict[str, Any], grade_thresholds: List[Dict[str, Any]]) -> str:
    result = normalize_status(case.get("result"))
    if case.get("category") != "flight" or result != "pass":
        return result
    score = case.get("score")
    if isinstance(score, (int, float)) and not isinstance(score, bool):
        return "pass" if float(score) >= _pass_score_threshold(grade_thresholds) else "fail"
    return result


def _build_case_flight_map(
    cases: List[Dict[str, Any]],
    flight_sections: List[Dict[str, Any]],
) -> Dict[str, Dict[str, Any]]:
    case_flight_map: Dict[str, Dict[str, Any]] = {}
    for section in flight_sections:
        title = section.get("title", "")
        prefixes = TITLE_TO_CASE_PREFIX.get(title, ())
        for case in cases:
            case_id = str(case.get("id", ""))
            if case_id in prefixes or any(case_id.startswith(prefix) for prefix in prefixes):
                case_flight_map[case_id] = section
                break
    return case_flight_map


def _render_time_and_metrics(case: Dict[str, Any], metrics_html: str) -> str:
    started_at = escape(format_display_text(case.get("started_at", "-")))
    finished_at = escape(format_display_text(case.get("finished_at", "-")))
    return f'<div class="case-time">{started_at} -> {finished_at}</div>{metrics_html}'


def render_case_rows(
    cases: List[Dict[str, Any]],
    flight_sections: List[Dict[str, Any]],
    grade_thresholds: List[Dict[str, Any]],
) -> str:
    case_flight_map = _build_case_flight_map(cases, flight_sections)
    rows: List[str] = []
    for index, case in enumerate(cases, start=1):
        result = _display_result(case, grade_thresholds)
        metrics_html = description_list(_case_metrics_for_display(case), compact=True)
        case_id = str(case.get("id", ""))
        flight_section = case_flight_map.get(case_id)
        expand_html = ""
        detail_row = ""
        if flight_section:
            section_title = escape(str(flight_section.get("title", "飞行指标")))
            detail_id = f"flight-detail-{index}"
            expand_html = (
                f'<details class="case-expand-details" id="{detail_id}">'
                f"<summary>{section_title}</summary>"
                "</details>"
            )
            detail_row = (
                f'<tr class="case-detail-row" id="{detail_id}-row" style="display:none"><td colspan="7">'
                f'<div class="case-flight-body">{render_flight_section_content(flight_section)}</div>'
                "</td></tr>"
            )
        rows.append(
            f'<tr class="case-row result-{result or "unknown"}">'
            f"<td>{index}{expand_html}</td>"
            f"<td><div class=\"case-title\">{escape(case.get('name', case.get('id', '-')))}</div></td>"
            f"<td>{escape(case.get('category', '-'))}</td>"
            f"<td>{status_badge(result)}</td>"
            f"<td>{score_display(case, grade_thresholds)}</td>"
            f"<td>{_render_time_and_metrics(case, metrics_html)}</td>"
            f"<td>{escape(format_duration(case.get('started_at'), case.get('finished_at')))}</td>"
            "</tr>"
            f"{detail_row}"
        )
    return "".join(rows)

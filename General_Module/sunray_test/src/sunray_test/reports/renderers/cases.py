from typing import Any, Dict, List, Tuple

from sunray_test.reports.renderers.common import (
    METRIC_DESCRIPTIONS,
    description_list,
    escape,
    format_display_text,
    format_duration,
    normalize_status,
    pretty_value_for_key,
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

LONG_CASE_TITLE_PREFIXES = ("EGO-Planner", "EGO自主规划")
CASE_SUMMARY_FALLBACK_LIMIT = 2
DETAIL_BLOCK_VALUE_KEYS = {"launch_args"}
DETAIL_BLOCK_VALUE_LENGTH = 56

SUMMARY_KEYS_BY_CASE_ID = {
    "battery": ("voltage_v", "pass_threshold_v"),
    "hover": ("duration_s",),
    "hover_stability": ("duration_s",),
    "ego_goal": ("goal_count", "goal_source"),
    "ego_goal_flight": ("goal_count", "goal_source"),
    "waypoint": ("waypoint_count", "waypoint_source"),
    "waypoint_flight": ("waypoint_count", "waypoint_source"),
    "visual_landing": ("height_m", "return_code"),
}

SUMMARY_KEY_PREFIXES = (
    ("battery", ("voltage_v", "pass_threshold_v")),
    ("front_camera", ("topic", "message_count")),
    ("down_camera", ("topic", "message_count")),
    ("lidar_health", ("lidar_rate_hz", "valid_clouds")),
    ("ego_goal", ("goal_count", "goal_source")),
    ("waypoint", ("waypoint_count", "waypoint_source")),
    ("visual_landing", ("height_m", "return_code")),
)


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
    hidden_keys = {"remaps", "pre_stop_results", "goals", "waypoints"}
    return {key: value for key, value in metrics.items() if key not in hidden_keys}


def _ordered_metric_keys(metrics: Dict[str, Any]) -> List[str]:
    return [key for key in metrics if key != "mission_key"]


def _summary_keys_for_case(case: Dict[str, Any], metrics: Dict[str, Any]) -> List[str]:
    case_id = str(case.get("id", ""))
    preferred = SUMMARY_KEYS_BY_CASE_ID.get(case_id)
    if not preferred:
        for prefix, keys in SUMMARY_KEY_PREFIXES:
            if case_id.startswith(prefix):
                preferred = keys
                break

    if preferred:
        keys = [key for key in preferred if key in metrics]
        if keys:
            return keys

    return _ordered_metric_keys(metrics)[:CASE_SUMMARY_FALLBACK_LIMIT]


def _subset_metrics(metrics: Dict[str, Any], keys: List[str]) -> Dict[str, Any]:
    return {key: metrics[key] for key in keys if key in metrics}


def _remaining_metrics(metrics: Dict[str, Any], visible_keys: List[str]) -> Dict[str, Any]:
    visible = set(visible_keys)
    return {key: value for key, value in metrics.items() if key not in visible and key != "mission_key"}


def _metric_info_html(key: str) -> str:
    desc = METRIC_DESCRIPTIONS.get(key)
    if not desc:
        return ""
    return f'<span class="metric-info">!<span class="metric-tooltip">{escape(desc)}</span></span>'


def _should_render_detail_block(key: str, value_text: str) -> bool:
    return key in DETAIL_BLOCK_VALUE_KEYS or "\n" in value_text or len(value_text) > DETAIL_BLOCK_VALUE_LENGTH


def _render_case_detail_metrics(metrics: Dict[str, Any]) -> str:
    if not metrics:
        return '<div class="empty-block">暂无数据</div>'

    items: List[str] = []
    for key, value in metrics.items():
        value_text = pretty_value_for_key(key, value)
        info_html = _metric_info_html(key)
        block_class = " case-param-card--wide" if _should_render_detail_block(key, value_text) else ""
        items.append(
            f'<div class="case-param-card{block_class}">'
            f'<div class="case-param-card-key">{escape(key)}{info_html}</div>'
            f'<div class="case-param-card-value">{escape(value_text)}</div>'
            "</div>"
        )
    return f'<div class="case-param-card-grid">{"".join(items)}</div>'


def _render_case_metrics(case: Dict[str, Any]) -> Tuple[str, str, str]:
    metrics = _case_metrics_for_display(case)
    if not metrics:
        return description_list(metrics, compact=True), "", ""

    summary_keys = _summary_keys_for_case(case, metrics)
    summary_metrics = _subset_metrics(metrics, summary_keys)
    detail_metrics = _remaining_metrics(metrics, summary_keys)
    summary_html = description_list(summary_metrics, compact=True)
    if not detail_metrics:
        return summary_html, "", ""

    detail_html = _render_case_detail_metrics(detail_metrics)
    detail_count = len(detail_metrics)
    expand_summary_html = (
        '<details class="case-expand-details case-param-details">'
        f"<summary>参数详情 {detail_count} 项</summary>"
        "</details>"
    )
    expand_body_html = f'<div class="case-param-detail-body">{detail_html}</div>'
    return summary_html, expand_summary_html, expand_body_html


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


def _render_case_time(case: Dict[str, Any]) -> str:
    started_at = escape(format_display_text(case.get("started_at", "-")))
    finished_at = escape(format_display_text(case.get("finished_at", "-")))
    return f'<div class="case-time">{started_at} -> {finished_at}</div>'


def _render_case_status_meta(
    case: Dict[str, Any],
    result: str,
    grade_thresholds: List[Dict[str, Any]],
) -> str:
    return (
        '<div class="case-status-meta">'
        '<div class="case-status-meta-row">'
        f'<div class="case-status-meta-item case-status-meta-category">{escape(case.get("category", "-"))}</div>'
        f'<div class="case-status-meta-item">{status_badge(result)}</div>'
        f'<div class="case-status-meta-item case-status-meta-score">{score_display(case, grade_thresholds)}</div>'
        "</div>"
        f'{_render_case_time(case)}'
        "</div>"
    )


def _case_title_cell_class(case_name: str) -> str:
    title = str(case_name or "").strip()
    if title.startswith(LONG_CASE_TITLE_PREFIXES) or len(title) > 8:
        return 'case-title-cell case-title-cell--wrap'
    return 'case-title-cell case-title-cell--nowrap'


def _case_title_style(case_name: str) -> str:
    title = str(case_name or "").strip()
    if title.startswith(LONG_CASE_TITLE_PREFIXES) or len(title) > 8:
        return "white-space: normal; word-break: break-word; overflow-wrap: anywhere;"
    return "white-space: nowrap; word-break: normal; overflow-wrap: normal;"


def render_case_rows(
    cases: List[Dict[str, Any]],
    flight_sections: List[Dict[str, Any]],
    grade_thresholds: List[Dict[str, Any]],
) -> str:
    case_flight_map = _build_case_flight_map(cases, flight_sections)
    rows: List[str] = []
    for index, case in enumerate(cases, start=1):
        result = _display_result(case, grade_thresholds)
        metrics_html, param_expand_html, param_detail_html = _render_case_metrics(case)
        case_id = str(case.get("id", ""))
        case_name = case.get("name", case.get("id", "-"))
        case_title_class = _case_title_cell_class(case_name)
        case_title_style = _case_title_style(case_name)
        flight_section = case_flight_map.get(case_id)
        expand_parts: List[str] = []
        detail_row = ""
        if flight_section:
            section_title = escape(str(flight_section.get("title", "飞行指标")))
            detail_id = f"flight-detail-{index}"
            expand_parts.append(
                f'<details class="case-expand-details" id="{detail_id}" data-case-detail-group="case-{index}">'
                f"<summary>{section_title}</summary>"
                "</details>"
            )
            detail_row = (
                f'<tr class="case-detail-row" id="{detail_id}-row" style="display:none"><td colspan="7">'
                f'<div class="case-flight-body">{render_flight_section_content(flight_section)}</div>'
                "</td></tr>"
            )
        if param_expand_html:
            detail_id = f"case-param-detail-{index}"
            param_expand_html = param_expand_html.replace(
                '<details class="case-expand-details case-param-details">',
                f'<details class="case-expand-details case-param-details" id="{detail_id}" data-case-detail-group="case-{index}">',
                1,
            )
            detail_row = (
                f"{detail_row}"
                f'<tr class="case-detail-row case-param-detail-row" id="{detail_id}-row" style="display:none">'
                f'<td colspan="7"><div class="case-param-row-body">{param_detail_html}</div></td></tr>'
            )
        expand_html = "".join(expand_parts)
        case_actions_html = f'<div class="case-actions">{expand_html}</div>' if expand_html else ""
        detail_metrics_html = f'<div class="case-detail-summary">{metrics_html}</div>'
        if param_expand_html:
            detail_metrics_html = f'{detail_metrics_html}<div class="case-param-actions">{param_expand_html}</div>'
        status_meta_html = _render_case_status_meta(case, result, grade_thresholds)
        rows.append(
            f'<tr class="case-row result-{result or "unknown"}">'
            f"<td>{index}</td>"
            f"<td class=\"{case_title_class}\" style=\"{case_title_style}\"><div class=\"case-title\">{escape(case_name)}</div>{case_actions_html}</td>"
            f'<td colspan="3">{status_meta_html}</td>'
            f"<td>{detail_metrics_html}</td>"
            f"<td>{escape(format_duration(case.get('started_at'), case.get('finished_at')))}</td>"
            "</tr>"
            f"{detail_row}"
        )
    return "".join(rows)

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
    "视觉降落指标": ("visual_landing",),
}


def _build_case_flight_map(cases: List[Dict[str, Any]], flight_sections: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
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


def render_case_rows(cases: List[Dict[str, Any]], flight_sections: List[Dict[str, Any]], grade_thresholds: List[Dict[str, Any]]) -> str:
    case_flight_map = _build_case_flight_map(cases, flight_sections)
    rows: List[str] = []
    for index, case in enumerate(cases, start=1):
        result = normalize_status(case.get("result"))
        metrics_html = description_list(case.get("metrics", {}), compact=True)
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
            f"<td>{status_badge(case.get('result'))}</td>"
            f"<td>{score_display(case, grade_thresholds)}</td>"
            f"<td><div class=\"case-time\">{escape(format_display_text(case.get('started_at', '-')))} -> {escape(format_display_text(case.get('finished_at', '-')))}</div>{metrics_html}</td>"
            f"<td>{escape(format_duration(case.get('started_at'), case.get('finished_at')))}</td>"
            "</tr>"
            f"{detail_row}"
        )
    return "".join(rows)

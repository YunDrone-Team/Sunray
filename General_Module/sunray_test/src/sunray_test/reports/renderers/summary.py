from typing import Any, Dict, List

from sunray_test.reports.renderers.common import (
    description_list,
    escape,
    format_time_short,
    normalize_status,
    pretty_value,
)


SCORE_SECTION_LABELS = {
    "hover": "悬停",
    "waypoint": "航点飞行",
    "visual_landing": "视觉降落",
}


def render_watermark_layer(text: str) -> str:
    positions = [
        ("6%", "10%"),
        ("10%", "32%"),
        ("14%", "54%"),
        ("18%", "76%"),
        ("22%", "94%"),
        ("30%", "14%"),
        ("34%", "36%"),
        ("38%", "58%"),
        ("42%", "80%"),
        ("46%", "96%"),
        ("54%", "8%"),
        ("58%", "28%"),
        ("62%", "50%"),
        ("66%", "72%"),
        ("70%", "92%"),
        ("78%", "16%"),
        ("82%", "38%"),
        ("86%", "60%"),
        ("90%", "82%"),
        ("94%", "96%"),
    ]
    items = []
    for top, left in positions:
        items.append(f'<span class="watermark-item" style="top:{top};left:{left};">{escape(text)}</span>')
    return f'<div class="watermark-layer" aria-hidden="true">{"".join(items)}</div>'


def render_summary_cards(summary: Dict[str, Any], pass_rate: float) -> str:
    summary_cards = [
        ("总用例", max(int(summary.get("total", 0)), 0), "neutral"),
        ("通过", summary.get("pass", 0), "pass"),
        ("失败", summary.get("fail", 0), "fail"),
        ("异常", summary.get("error", 0), "error"),
        ("不支持", summary.get("unsupported", 0), "unsupported"),
        ("通过率", f"{pass_rate:.1f}%", "accent"),
    ]
    return "".join(
        "<div class=\"summary-card summary-{tone}\">"
        f"<div class=\"summary-label\">{escape(label)}</div>"
        f"<div class=\"summary-value\">{escape(value)}</div>"
        "</div>".format(tone=tone)
        for label, value, tone in summary_cards
    )


def render_score_cards(payload: Dict[str, Any]) -> str:
    scores = payload.get("flight_metrics", {}).get("scores")
    if not scores:
        return ""
    overall = scores.get("overall", {})
    overall_score = overall.get("score")
    if overall_score is None:
        return ""

    overall_grade = escape(str(overall.get("grade", "-")))
    overall_color = overall.get("grade_color", "#69758a")

    sub_cards: List[str] = []
    for key in ("hover", "waypoint", "visual_landing"):
        sec = scores.get(key)
        if not isinstance(sec, dict):
            continue
        sec_score = sec.get("score")
        if sec_score is None:
            continue
        sec_grade = escape(str(sec.get("grade", "-")))
        sec_color = sec.get("grade_color", "#69758a")
        label = SCORE_SECTION_LABELS.get(key, key)
        sub_cards.append(
            f'<div class="score-sub-card">'
            f'<div class="score-sub-label">{escape(label)}</div>'
            f'<div class="score-sub-value" style="color:{sec_color}">{sec_score:.1f}</div>'
            f'<div class="score-sub-grade" style="color:{sec_color}">{sec_grade}</div>'
            "</div>"
        )

    return (
        '<section class="section">'
        '<div class="section-header"><h2 class="section-title">综合评分</h2></div>'
        '<div class="section-body">'
        '<div class="score-overview">'
        f'<div class="score-main-card">'
        '<div class="score-main-label">总分</div>'
        f'<div class="score-main-value" style="color:{overall_color}">{overall_score:.1f}</div>'
        f'<div class="score-main-grade" style="background:{overall_color}">{overall_grade}</div>'
        "</div>"
        f'<div class="score-sub-cards">{"".join(sub_cards)}</div>'
        "</div>"
        "</div>"
        "</section>"
    )


def _case_status(case: Dict[str, Any]) -> str:
    result = normalize_status(case.get("result"))
    if result in {"pass", "unsupported"}:
        return "completed"
    return "failed"


def _find_event_time(event_log: List[Dict[str, Any]], event_name: str) -> str:
    for row in event_log:
        if row.get("event") == event_name:
            return escape(format_time_short(row.get("time_str") or row.get("timestamp") or "-"))
    return ""


def build_stage_timeline(payload: Dict[str, Any]) -> List[Dict[str, str]]:
    run_info = payload.get("run_info", {})
    cases = payload.get("cases", [])
    phase_log = payload.get("phase_log", [])
    event_log = payload.get("event_log", [])
    defaults = payload.get("config", {}).get("defaults", {})

    stages: List[Dict[str, str]] = []

    hardware_cases = [case for case in cases if case.get("category") == "hardware"]
    if hardware_cases:
        stage_status = "completed" if all(_case_status(case) == "completed" for case in hardware_cases) else "failed"
        stages.append(
            {
                "name": "硬件检测",
                "detail": f"{len(hardware_cases)} 项硬件检查",
                "time": (
                    f"{escape(format_time_short(hardware_cases[0].get('started_at', '-')))}"
                    f" -> "
                    f"{escape(format_time_short(hardware_cases[-1].get('finished_at', '-')))}"
                ),
                "status": stage_status,
            }
        )

    takeoff_phase = next((item for item in phase_log if item.get("phase") == "arm_and_takeoff"), None)
    if takeoff_phase:
        takeoff_status = normalize_status(takeoff_phase.get("status")) or "completed"
        if takeoff_status == "failed":
            stages.append(
                {
                    "name": "起飞阶段",
                    "detail": escape(takeoff_phase.get("detail") or "起飞阶段失败"),
                    "time": escape(format_time_short(takeoff_phase.get("timestamp", "-"))),
                    "status": "failed",
                }
            )
        else:
            arm_start_time = _find_event_time(event_log, "arm_start") or escape(format_time_short(takeoff_phase.get("timestamp", "-")))
            arm_end_time = _find_event_time(event_log, "arm_end") or escape(format_time_short(takeoff_phase.get("timestamp", "-")))
            stages.append(
                {
                    "name": "解锁",
                    "detail": "已切换命令模式并完成解锁",
                    "time": f"{arm_start_time} -> {arm_end_time}",
                    "status": "completed",
                }
            )
            target_pos = defaults.get("takeoff_target_pos")
            target_text = pretty_value(target_pos) if target_pos else "-"
            takeoff_start_time = _find_event_time(event_log, "takeoff_start") or escape(format_time_short(takeoff_phase.get("timestamp", "-")))
            takeoff_end_time = _find_event_time(event_log, "takeoff_end") or escape(format_time_short(takeoff_phase.get("timestamp", "-")))
            stages.append(
                {
                    "name": "起飞",
                    "detail": f"目标起飞点 {target_text}",
                    "time": f"{takeoff_start_time} -> {takeoff_end_time}",
                    "status": "completed",
                }
            )

    stage_case_map = [
        ("悬停", {"hover_stability", "hover"}, "悬停稳定性检查"),
        ("指点飞行", {"waypoint_flight", "waypoint"}, "航点任务执行"),
        ("视觉降落", {"visual_landing"}, "视觉引导降落"),
    ]
    for stage_name, keywords, fallback_detail in stage_case_map:
        matched_case = next(
            (
                case
                for case in cases
                if case.get("id") in keywords or any(keyword in str(case.get("id", "")) for keyword in keywords)
            ),
            None,
        )
        if matched_case:
            stages.append(
                {
                    "name": stage_name,
                    "detail": escape(matched_case.get("name") or fallback_detail),
                    "time": (
                        f"{escape(format_time_short(matched_case.get('started_at', '-')))}"
                        f" -> "
                        f"{escape(format_time_short(matched_case.get('finished_at', '-')))}"
                    ),
                    "status": _case_status(matched_case),
                }
            )

    if run_info.get("finished_at"):
        summary = payload.get("summary", {})
        if summary.get("fail", 0) or summary.get("error", 0):
            status = "failed"
            detail = "测试结束，存在失败或异常项"
        elif run_info.get("interrupted"):
            status = "failed"
            detail = "测试中断结束"
        else:
            status = "completed"
            detail = "测试正常结束"
        stages.append(
            {
                "name": "测试结束",
                "detail": detail,
                "time": escape(format_time_short(run_info.get("finished_at"))),
                "status": status,
            }
        )

    return stages


def render_stage_timeline(payload: Dict[str, Any]) -> str:
    stage_timeline = build_stage_timeline(payload)
    phase_items: List[str] = []
    for stage in stage_timeline:
        status = stage.get("status", "completed")
        phase_items.append(
            f"<div class=\"timeline-step timeline-{status}\">"
            "<div class=\"timeline-dot\"></div>"
            f"<div class=\"timeline-phase\">{escape(stage.get('name', '-'))}</div>"
            f"<div class=\"timeline-state\">{escape(stage.get('detail', '-'))}</div>"
            f"<div class=\"timeline-time\">{escape(stage.get('time', '-'))}</div>"
            "</div>"
        )
    timeline_content = "".join(phase_items) or '<div class="empty-block">本次执行没有记录 phase 数据</div>'
    return (
        f'<div class="timeline" style="--timeline-count: {max(len(stage_timeline), 1)};">'
        f"{timeline_content}"
        "</div>"
    )


def render_report_meta(run_info: Dict[str, Any], duration: str) -> str:
    report_meta = {
        "机型": run_info.get("platform"),
        "环境": run_info.get("environment"),
        "测试方案": run_info.get("suite"),
        "SN": run_info.get("sn") or "unknown",
        "测试人员": run_info.get("tester") or "unknown",
        "开始时间": run_info.get("started_at"),
        "结束时间": run_info.get("finished_at"),
        "总时长": duration,
        "异常中断": "True" if run_info.get("interrupted") else "False",
        "中断原因": run_info.get("interruption_reason") or "-",
    }
    return description_list(report_meta)

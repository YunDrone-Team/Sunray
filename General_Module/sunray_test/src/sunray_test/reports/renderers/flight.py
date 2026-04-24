from typing import Any, Dict, Iterable, List

from sunray_test.reports.renderers.common import (
    FLIGHT_SECTION_LABELS,
    METRIC_DESCRIPTIONS,
    description_list,
    escape,
    pretty_value,
    render_labeled_value,
    render_metric_blocks,
)


def _preferred_metric_order(section_title: str, category_name: str) -> Iterable[str]:
    if section_title == "视觉降落指标" and category_name == "stability":
        return (
            "target_acquisition_time_s",
            "target_tracking_continuity_rate",
            "target_loss_count",
            "max_target_loss_duration_s",
            "descent_stability_level",
            "target_tracking_continuity_rate_exempt_terminal_loss",
        )
    return ()


def _filter_metrics_for_display(section_title: str, category_name: str, metrics: Dict[str, Any]) -> Dict[str, Any]:
    if section_title == "视觉降落指标" and category_name == "stability":
        return {
            key: value
            for key, value in metrics.items()
            if key != "low_altitude_correction_pass"
        }
    return metrics


def _render_missions_snapshot(missions: Dict[str, Any]) -> str:
    if not missions:
        return '<div class="empty-block">暂无数据</div>'

    mission_cards: List[str] = []
    for mission_key, mission_value in missions.items():
        if isinstance(mission_value, dict):
            mission_name = mission_value.get("name") or mission_key
            mission_meta = {k: v for k, v in mission_value.items() if k not in {"name", "waypoints", "mission_key"}}
            waypoints = mission_value.get("waypoints")
        else:
            mission_name = mission_key
            mission_meta = {}
            waypoints = None

        meta_html = description_list(mission_meta) if mission_meta else ""

        waypoint_html = ""
        if isinstance(waypoints, list) and waypoints:
            waypoint_items: List[str] = []
            for index, point in enumerate(waypoints, start=1):
                point_text = pretty_value(point)
                waypoint_items.append(
                    '<div style="display:flex; align-items:center; gap:10px; '
                    'background:#f8fafc; border:1px solid #edf2f7; border-radius:10px; '
                    'padding:8px 10px;">'
                    f'<span style="display:inline-flex; align-items:center; justify-content:center; '
                    f'width:22px; height:22px; border-radius:999px; background:var(--primary); '
                    f'color:#fff; font-size:11px; font-weight:700; flex-shrink:0;">{index}</span>'
                    f'<span style="font-family: var(--mono); font-size: 12px;">{escape(point_text)}</span>'
                    "</div>"
                )
            waypoint_html = (
                '<div style="margin-top: 12px;">'
                '<div style="font-size: 12px; font-weight: 700; color: var(--muted); margin-bottom: 8px;">航点列表</div>'
                f'<div style="display:flex; flex-direction:column; gap:8px;">{"".join(waypoint_items)}</div>'
                "</div>"
            )

        mission_cards.append(
            '<div style="border: 1px solid var(--line); background: #f8fafc; border-radius: 14px; padding: 14px;">'
            f'<div style="font-size: 14px; font-weight: 700; color: var(--text);">{escape(mission_name)}</div>'
            f'<div style="font-size: 12px; color: var(--muted); margin-top: 4px; font-family: var(--mono);">{escape(str(mission_key))}</div>'
            f"{meta_html}"
            f"{waypoint_html}"
            "</div>"
        )

    return '<div style="display:flex; flex-direction:column; gap:12px;">' + "".join(mission_cards) + "</div>"


def render_config_snapshot(config: Dict[str, Any]) -> str:
    defaults = config.get("defaults", {}) if isinstance(config.get("defaults"), dict) else {}
    topics = config.get("topics", {}) if isinstance(config.get("topics"), dict) else {}
    missions = config.get("missions", {}) if isinstance(config.get("missions"), dict) else {}

    defaults_html = "".join(render_labeled_value(key, value) for key, value in defaults.items()) or '<div class="empty-block">暂无数据</div>'
    topics_items: List[str] = []
    for key, value in topics.items():
        desc = METRIC_DESCRIPTIONS.get(key)
        info_html = (
            f'<span class="metric-info">!<span class="metric-tooltip">{escape(desc)}</span></span>'
            if desc else ""
        )
        topics_items.append(
            '<div style="font-size: 11px; display: flex; justify-content: space-between; gap: 8px; '
            'background: #f8fafc; padding: 6px 8px; border-radius: 4px;">'
            f'<span style="font-weight: 700; color: var(--muted);">{escape(key)}{info_html}</span>'
            f'<span style="font-family: var(--mono);">{escape(pretty_value(value))}</span>'
            "</div>"
        )
    topics_html = "".join(topics_items) or '<div class="empty-block">暂无数据</div>'

    missions_html = _render_missions_snapshot(missions)

    return (
        '<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px;">'
        '<div class="flight-card">'
        '<div class="flight-card-title" style="margin-bottom: 12px; color: var(--primary); border-bottom: 1px solid var(--line); padding-bottom: 8px;">Defaults (基础配置)</div>'
        f'<div style="display: flex; flex-direction: column; gap: 8px;">{defaults_html}</div>'
        "</div>"
        '<div class="flight-card">'
        '<div class="flight-card-title" style="margin-bottom: 12px; color: var(--primary); border-bottom: 1px solid var(--line); padding-bottom: 8px;">Topics (话题映射)</div>'
        f'<div style="display: flex; flex-direction: column; gap: 6px;">{topics_html}</div>'
        "</div>"
        '<div class="flight-card">'
        '<div class="flight-card-title" style="margin-bottom: 12px; color: var(--primary); border-bottom: 1px solid var(--line); padding-bottom: 8px;">Missions (任务详情)</div>'
        f"{missions_html}"
        "</div>"
        "</div>"
    )


def render_artifacts(artifacts: Dict[str, Any]) -> str:
    cards: List[str] = []
    for key in ("run_dir", "report_html", "event_log_jsonl", "bag_file"):
        if key not in artifacts:
            continue
        value = artifacts.get(key)
        display_value = value.split("/")[-1] if key == "bag_file" and isinstance(value, str) else value
        cards.append(
            '<div class="flight-card">'
            '<div style="font-size: 12px;">'
            f'<span style="font-weight: 700; color: var(--muted);">{escape(key)}</span>'
            f'<div style="background: #f8fafc; border-radius: 4px; padding: 6px 8px; font-family: var(--mono); margin-top: 4px; border: 1px solid #edf2f7; font-size: 11px; word-break: break-all;">{escape(pretty_value(display_value))}</div>'
            "</div>"
            "</div>"
        )

    recording_topics = artifacts.get("recording_topics")
    if recording_topics:
        topic_list = recording_topics if isinstance(recording_topics, list) else [recording_topics]
        topic_cards = "".join(
            '<div style="display: flex; align-items: center; gap: 8px; background: #f8fafc; '
            'border: 1px solid #edf2f7; border-radius: 8px; padding: 6px 10px;">'
            '<span style="display: inline-block; width: 6px; height: 6px; border-radius: 50%; '
            'background: var(--accent); flex-shrink: 0;"></span>'
            f'<span style="font-family: var(--mono); font-size: 11px; word-break: break-all;">{escape(topic)}</span>'
            "</div>"
            for topic in topic_list
        )
        cards.append(
            '<div class="flight-card" style="grid-column: 1 / -1;">'
            '<div style="font-size: 12px;">'
            '<span style="font-weight: 700; color: var(--muted);">recording_topics</span>'
            f'<div style="display: flex; flex-wrap: wrap; gap: 8px; margin-top: 6px;">{topic_cards}</div>'
            "</div>"
            "</div>"
        )

    cards_html = "".join(cards) or '<div class="empty-block">暂无数据</div>'
    return f'<div class="flight-grid" style="grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));">{cards_html}</div>'


def render_flight_section_content(section: Dict[str, Any]) -> str:
    raw_title = str(section.get("title", "飞行指标"))
    content_parts: List[str] = []

    summary_by_category = section.get("summary_by_category")
    if isinstance(summary_by_category, dict) and summary_by_category:
        content_parts.append('<div class="flight-subtitle">汇总指标</div>')
        for category_name, metrics in summary_by_category.items():
            display_name = FLIGHT_SECTION_LABELS.get(category_name, category_name)
            content_parts.append(f'<div class="flight-subtitle">{escape(display_name)}</div>')
            content_parts.append(render_metric_blocks(metrics))
    else:
        summary = section.get("summary")
        if isinstance(summary, dict) and summary:
            content_parts.append('<div class="flight-subtitle">汇总指标</div>')
            content_parts.append(render_metric_blocks(summary))

    metrics_by_category = section.get("metrics_by_category")
    if isinstance(metrics_by_category, dict) and metrics_by_category:
        for category_name, metrics in metrics_by_category.items():
            if not isinstance(metrics, dict):
                continue
            if raw_title == "悬停指标" and category_name == "analysis_info":
                continue
            display_name = FLIGHT_SECTION_LABELS.get(category_name, category_name)
            display_metrics = _filter_metrics_for_display(raw_title, category_name, metrics)
            if not display_metrics:
                continue
            content_parts.append(f'<div class="flight-subtitle">{escape(display_name)}</div>')
            content_parts.append(render_metric_blocks(display_metrics, _preferred_metric_order(raw_title, category_name)))
    else:
        flat_metrics = section.get("metrics")
        if isinstance(flat_metrics, dict) and flat_metrics:
            content_parts.append(render_metric_blocks(flat_metrics))

    waypoints = section.get("waypoints")
    if isinstance(waypoints, list) and waypoints:
        cards: List[str] = []
        for index, waypoint in enumerate(waypoints, start=1):
            waypoint_label = escape(pretty_value(waypoint.get("waypoint", [])))
            wp_metrics = waypoint.get("metrics", {})
            wp_metrics_by_category = waypoint.get("metrics_by_category", {})
            metric_html: List[str] = []
            if isinstance(wp_metrics_by_category, dict) and wp_metrics_by_category:
                for category_name, category_metrics in wp_metrics_by_category.items():
                    display_name = FLIGHT_SECTION_LABELS.get(category_name, category_name)
                    metric_html.append(f'<div class="flight-subtitle">{escape(display_name)}</div>')
                    metric_html.append(render_metric_blocks(category_metrics))
            else:
                metric_html.append(render_metric_blocks(wp_metrics))
            cards.append(
                '<div class="flight-card">'
                f'<div class="flight-card-title">航点 {index}</div>'
                f'<div class="flight-card-subtitle">{waypoint_label}</div>'
                f'{"".join(metric_html)}'
                "</div>"
            )
        content_parts.append('<div class="flight-subtitle">航点明细</div>')
        content_parts.append(f'<div class="flight-grid">{"".join(cards)}</div>')

    meta = {}
    for key in (
        "case_id",
        "pose_topic",
        "detection_topic",
        "target_xyz",
        "landing_target_xy",
        "landing_target_xy_source",
        "landing_target_xy_sample_count",
        "threshold_m",
        "stable_time_s",
        "target_zone_radius_m",
        "touchdown_window_s",
        "window",
        "artifacts",
    ):
        if key in section and section.get(key) not in ({}, [], "", None):
            meta[key] = section.get(key)
    if raw_title == "悬停指标":
        meta = {}
    elif raw_title == "视觉降落指标":
        allowed_meta_keys = ("landing_target_xy", "target_zone_radius_m", "touchdown_window_s")
        meta = {key: meta[key] for key in allowed_meta_keys if key in meta}
    if meta:
        content_parts.append('<div class="flight-subtitle">分析信息</div>')
        content_parts.append(render_metric_blocks(meta))

    limitations = section.get("limitations")
    if isinstance(limitations, list) and limitations:
        items = "".join(f"<li>{escape(item)}</li>" for item in limitations)
        content_parts.append('<div class="flight-subtitle">限制说明</div>')
        content_parts.append(f'<ul class="plain-list">{items}</ul>')

    return "".join(content_parts) or '<div class="empty-block">暂无数据</div>'


def render_flight_metrics(payload: Dict[str, Any]) -> str:
    flight_metrics = payload.get("flight_metrics", {})
    sections = flight_metrics.get("sections", [])
    errors = flight_metrics.get("errors", [])
    if not sections and not errors:
        return ""

    blocks: List[str] = []
    for section in sections:
        title = escape(str(section.get("title", "飞行指标")))
        content_html = render_flight_section_content(section)
        blocks.append(
            '<div class="flight-section">'
            f'<h3 class="flight-title">{title}</h3>'
            f"{content_html}"
            "</div>"
        )

    if errors:
        error_items = "".join(f"<li>{escape(item)}</li>" for item in errors)
        blocks.append(
            '<div class="flight-section">'
            '<h3 class="flight-title">分析提示</h3>'
            f'<ul class="plain-list">{error_items}</ul>'
            "</div>"
        )

    return (
        "<section class=\"section\">"
        "<div class=\"section-header\"><h2 class=\"section-title\">飞行指标</h2></div>"
        f"<div class=\"section-body\">{''.join(blocks)}</div>"
        "</section>"
    )

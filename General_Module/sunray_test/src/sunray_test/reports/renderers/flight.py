from typing import Any, Dict, Iterable, List

from sunray_test.reports.renderers.common import (
    FLIGHT_SECTION_LABELS,
    description_list,
    escape,
    pretty_value,
    render_metric_blocks,
)


def _preferred_metric_order(section_title: str, category_name: str) -> Iterable[str]:
    if section_title == "视觉降落指标" and category_name == "trajectory":
        return (
            "landing_target_zone_radius_m",
            "landing_target_center_xy_m",
            "landing_target_source",
            "landing_target_estimate_count",
            "final_landing_position_error_m",
            "horizontal_alignment_error_m",
            "final_descent_trigger_xy_error_m",
            "landing_start_xy_m",
            "landing_end_xy_m",
            "landing_start_to_end_xy_displacement_m",
            "horizontal_travel_m",
            "landing_vertical_drop_m",
            "landing_final_altitude_m",
            "target_acquisition_time_s",
            "target_tracking_continuity_rate",
            "target_tracking_continuity_rate_exempt_terminal_loss",
            "descent_stability_level",
        )
    if section_title == "视觉降落指标" and category_name == "completion":
        return (
            "visual_landing_success",
            "visual_landing_aborted",
            "landed_within_target_zone",
            "landing_duration_s",
            "landing_case_duration_s",
            "landing_duration_source",
            "landing_duration_fallback_reason",
            "landing_effective_start_offset_s",
            "landing_effective_end_offset_s",
            "landing_effective_start_altitude_m",
            "landing_effective_end_altitude_m",
            "landing_terminal_altitude_threshold_m",
            "landing_safety_pass",
        )
    return ()


def _filter_metrics_for_display(section_title: str, category_name: str, metrics: Dict[str, Any]) -> Dict[str, Any]:
    return metrics


def _should_show_mission(mission_key: str, platform_name: str) -> bool:
    if platform_name == "sunray150_basic" and str(mission_key).startswith("lidar_"):
        return False
    return True


def _render_point_list(title: str, points: List[Any]) -> str:
    items: List[str] = []
    for index, point in enumerate(points, start=1):
        items.append(
            '<div class="mission-point-item">'
            f'<span class="mission-point-index">{index}</span>'
            f'<span class="mission-point-value">{_render_point_value(point)}</span>'
            "</div>"
        )
    return (
        '<div class="mission-point-section">'
        f'<div class="mission-point-title">{escape(title)}</div>'
        f'<div class="mission-point-grid">{"".join(items)}</div>'
        "</div>"
    )


def _render_point_value(point: Any) -> str:
    if isinstance(point, (list, tuple)):
        axis_labels = ("X", "Y", "Z", "Yaw")
        chips: List[str] = []
        for idx, value in enumerate(point):
            label = axis_labels[idx] if idx < len(axis_labels) else f"P{idx + 1}"
            axis_class = label.lower()
            axis_text = label.upper()
            chips.append(
                f'<span class="mission-point-chip mission-point-chip--{escape(axis_class)}">'
                f'<span class="mission-point-axis">{escape(axis_text)}</span>'
                f'<span class="mission-point-number">{escape(pretty_value(value))}</span>'
                "</span>"
            )
        return '<span class="mission-point-coords">' + "".join(chips) + "</span>"
    return escape(pretty_value(point))


def _render_flight_card_title(title: str) -> str:
    return (
        '<div class="flight-card-title" style="margin-bottom: 12px; '
        'color: var(--primary); border-bottom: 1px solid var(--line); '
        f'padding-bottom: 8px;">{escape(title)}</div>'
    )


CONFIG_DEFAULT_GROUPS = (
    (
        "硬件与相机",
        (
            "hardware_check_timeout_s",
            "camera_sample_duration_s",
            "camera_min_messages",
            "camera_min_rate_hz",
            "camera_max_gap_s",
            "camera_max_identical_frame_ratio",
            "camera_black_mean_threshold",
            "camera_black_dynamic_range_threshold",
            "camera_max_black_frame_ratio",
            "camera_require_timestamp_progress",
            "camera_require_frame_content_change",
            "battery_pass_threshold_v",
        ),
    ),
    (
        "起飞流程",
        (
            "takeoff_target_z_m",
            "takeoff_reach_radius_m",
            "takeoff_stable_time_s",
            "takeoff_timeout_s",
            "takeoff_command_rate_hz",
            "post_takeoff_settle_time_s",
        ),
    ),
    ("悬停", ("hover_duration_s",)),
    (
        "航点飞行",
        (
            "waypoint_source",
            "waypoint_reach_radius_m",
            "waypoint_stable_time_s",
            "waypoint_hold_time_s",
            "waypoint_timeout_s",
            "waypoint_analysis_use_xy_only",
        ),
    ),
    (
        "EGO 自主规划",
        (
            "ego_goal_source",
            "ego_goal_z_m",
            "ego_goal_reach_radius_m",
            "ego_goal_stable_time_s",
            "ego_goal_hold_time_s",
            "ego_goal_timeout_s",
            "ego_goal_publish_burst_count",
            "ego_goal_publish_burst_interval_s",
            "ego_goal_use_xy_only",
            "ego_keepalive_enabled",
            "ego_keepalive_rate_hz",
            "ego_keepalive_stale_timeout_s",
            "ego_keepalive_zero_velocity_epsilon",
            "ego_post_transition_enabled",
            "ego_post_transition_target_yaw_rad",
            "ego_post_transition_yaw_rate_rad_s",
            "ego_post_transition_hold_after_s",
            "ego_post_transition_target_z_m",
        ),
    ),
    (
        "视觉降落",
        (
            "visual_landing_auto_takeoff",
            "visual_landing_height_m",
            "visual_landing_target_zone_radius_m",
        ),
    ),
)


def _render_config_group(title: str, data: Dict[str, Any], preferred_order: Iterable[str] = ()) -> str:
    if not data:
        return ""
    return (
        '<div class="config-snapshot-group">'
        f'<div class="flight-subtitle">{escape(title)}</div>'
        f"{render_metric_blocks(data, preferred_order)}"
        "</div>"
    )


def _render_defaults_snapshot(defaults: Dict[str, Any], analysis: Dict[str, Any]) -> str:
    groups: List[str] = []
    used_keys = set()
    for title, keys in CONFIG_DEFAULT_GROUPS:
        group_data = {key: defaults[key] for key in keys if key in defaults}
        if group_data:
            groups.append(_render_config_group(title, group_data, keys))
            used_keys.update(group_data)

    if analysis:
        groups.append(_render_config_group("分析输入", analysis))

    other_defaults = {key: value for key, value in defaults.items() if key not in used_keys}
    if other_defaults:
        groups.append(_render_config_group("其他参数", other_defaults))

    return "".join(groups) or '<div class="empty-block">暂无数据</div>'


def _render_topics_snapshot(topics: Dict[str, Any]) -> str:
    return _render_config_group("话题映射", topics) or '<div class="empty-block">暂无数据</div>'


def _render_config_section(title: str, content_html: str) -> str:
    return (
        '<details class="config-snapshot-section">'
        '<summary class="config-snapshot-summary">'
        f'<span class="config-snapshot-title">{escape(title)}</span>'
        "</summary>"
        f'<div class="config-snapshot-section-body">{content_html}</div>'
        "</details>"
    )


def _render_missions_snapshot(missions: Dict[str, Any], platform_name: str) -> str:
    if not missions:
        return '<div class="empty-block">暂无数据</div>'

    mission_cards: List[str] = []
    for mission_key, mission_value in missions.items():
        if not _should_show_mission(str(mission_key), platform_name):
            continue
        if isinstance(mission_value, dict):
            mission_name = mission_value.get("name") or mission_key
            mission_meta = {
                key: value
                for key, value in mission_value.items()
                if key not in {"name", "waypoints", "goals", "mission_key"}
            }
            waypoints = mission_value.get("waypoints")
            goals = mission_value.get("goals")
        else:
            mission_name = mission_key
            mission_meta = {}
            waypoints = None
            goals = None

        meta_html = description_list(mission_meta) if mission_meta else ""

        waypoint_html = ""
        if isinstance(waypoints, list) and waypoints:
            waypoint_html = _render_point_list("航点列表", waypoints)
        elif isinstance(goals, list) and goals:
            waypoint_html = _render_point_list("目标点列表", goals)

        mission_cards.append(
            '<div style="border: 1px solid var(--line); background: #f8fafc; border-radius: 14px; padding: 14px;">'
            f'<div style="font-size: 14px; font-weight: 700; color: var(--text);">{escape(mission_name)}</div>'
            + (
                '<div style="font-size: 12px; color: var(--muted); '
                f'margin-top: 4px; font-family: var(--mono);">{escape(str(mission_key))}</div>'
                if str(mission_name) != str(mission_key)
                else ""
            )
            + f"{meta_html}"
            f"{waypoint_html}"
            "</div>"
        )

    return '<div style="display:flex; flex-direction:column; gap:12px;">' + "".join(mission_cards) + "</div>"


def render_config_snapshot(config: Dict[str, Any], platform_name: str = "") -> str:
    defaults = config.get("defaults", {}) if isinstance(config.get("defaults"), dict) else {}
    analysis = config.get("analysis", {}) if isinstance(config.get("analysis"), dict) else {}
    topics = config.get("topics", {}) if isinstance(config.get("topics"), dict) else {}
    missions = config.get("missions", {}) if isinstance(config.get("missions"), dict) else {}

    defaults_html = _render_defaults_snapshot(defaults, analysis)
    topics_html = _render_topics_snapshot(topics)
    missions_html = _render_missions_snapshot(missions, platform_name)

    return (
        '<div class="config-snapshot-layout">'
        f'{_render_config_section("基础配置", defaults_html)}'
        f'{_render_config_section("话题映射", topics_html)}'
        f'{_render_config_section("任务详情", missions_html)}'
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
            '<div style="background: #f8fafc; border-radius: 4px; padding: 6px 8px; '
            'font-family: var(--mono); margin-top: 4px; border: 1px solid #edf2f7; '
            f'font-size: 11px; word-break: break-all;">{escape(pretty_value(display_value))}</div>'
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
    return (
        '<div class="flight-grid" style="grid-template-columns: '
        f'repeat(auto-fit, minmax(280px, 1fr));">{cards_html}</div>'
    )


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
            content_parts.append(
                render_metric_blocks(display_metrics, _preferred_metric_order(raw_title, category_name))
            )
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
                f'<div class="flight-card-title">{"目标点" if raw_title == "EGO自主规划指标" else "航点"} {index}</div>'
                f'<div class="flight-card-subtitle">{waypoint_label}</div>'
                f'{"".join(metric_html)}'
                "</div>"
            )
        detail_title = "目标点明细" if raw_title == "EGO自主规划指标" else "航点明细"
        content_parts.append(f'<div class="flight-subtitle">{detail_title}</div>')
        content_parts.append(f'<div class="flight-grid">{"".join(cards)}</div>')

    meta = {}
    for key in (
        "case_id",
        "pose_topic",
        "target_xyz",
        "threshold_m",
        "stable_time_s",
        "launch_args",
        "window",
        "artifacts",
    ):
        if key in section and section.get(key) not in ({}, [], "", None):
            meta[key] = section.get(key)
    if raw_title == "悬停指标":
        meta = {}
    elif raw_title == "视觉降落指标":
        meta = {}
    if meta:
        content_parts.append('<div class="flight-subtitle">分析信息</div>')
        content_parts.append(render_metric_blocks(meta))

    limitations = section.get("limitations")
    if isinstance(limitations, list) and limitations:
        items = "".join(f"<li>{escape(item)}</li>" for item in limitations)
        content_parts.append('<div class="flight-subtitle">限制说明</div>')
        content_parts.append(f'<ul class="plain-list">{items}</ul>')

    return "".join(content_parts) or '<div class="empty-block">暂无数据</div>'

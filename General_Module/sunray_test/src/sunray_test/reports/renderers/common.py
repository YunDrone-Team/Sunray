import html
from datetime import datetime
from typing import Any, Dict, Iterable, List


STATUS_LABELS = {
    "pass": "PASS",
    "fail": "FAIL",
    "error": "ERROR",
    "unsupported": "UNSUPPORTED",
}

FLIGHT_SECTION_LABELS = {
    "accuracy": "准确性",
    "completion": "完成性",
    "stability": "稳定性",
    "precision_stability": "精度稳定性",
    "path_quality": "路径合理性",
    "response_smoothness": "响应平顺性",
    "robustness": "鲁棒性",
    "smoothness": "平顺性",
    "functional_completeness": "功能完整性",
    "planning": "规划质量",
    "analysis_info": "分析信息",
}

METRIC_DESCRIPTIONS = {
    "mean_xy_error": "悬停窗口内 xy_error 均值",
    "mean_abs_z_error": "悬停窗口内 abs_z_error 均值",
    "xy_error_rmse": "悬停窗口内 xy_error 的均方根",
    "z_error_rmse": "悬停窗口内 z_error 的均方根",
    "xy_error_p95": "xy_error 的 95 分位",
    "abs_z_error_p95": "abs_z_error 的 95 分位",
    "xy_error_max": "悬停窗口内最大水平误差",
    "abs_z_error_max": "悬停窗口内最大高度误差",
    "speed_mean": "悬停窗口内速度均值",
    "speed_p95": "速度 95 分位",
    "speed_max": "最大速度",
    "reach_success_flag": "在该段内是否曾进入到达判定窗口",
    "goal_success": "单个目标点是否成功进入目标半径并稳定",
    "reach_time": "从该段开始到首次满足到达判定的时间",
    "settling_time": "从该段开始到首次满足更严格停稳判定的时间",
    "within_threshold": "稳定判定时最终误差是否在阈值内",
    "final_xy_error": "该段结束时的水平误差",
    "final_abs_z_error": "该段结束时的高度误差",
    "hold_window_p95_xy_error": "到达后保持窗口内的 xy_error 95 分位",
    "speed_mean_mps": "目标段平均速度",
    "speed_p95_mps": "目标段速度 95 分位",
    "path_length_m": "目标段实际飞行轨迹长度",
    "path_direct_distance_m": "目标段起点到目标点的直线距离 (米)，用于作为路径长度和绕行程度的基准",
    "path_efficiency": "direct_distance / path_length_m",
    "path_detour_ratio": "实际轨迹长度 / 直线距离",
    "path_heading_change_deg": "轨迹方向变化角累计值",
    "path_backtrack_distance_m": "沿目标方向反向运动累计距离",
    "goal_completion_rate": "成功目标点数 / 计划目标点数",
    "max_lateral_deviation_m": "相对起终点连线的最大横向偏离",
    "max_speed": "该段最大速度",
    "overshoot_distance_m": "穿过目标点后沿起终点方向的最大超越量",
    "wp_success_count": "成功到达的 waypoint 数量",
    "all_wp_success_flag": "n 个 waypoint 是否全部到达",
    "total_waypoint_time_s": "n 段总时长",
    "mean_path_efficiency": "n 段 path_efficiency 均值",
    "worst_final_xy_error": "n 段 final_xy_error 的最大值",
    "horizontal_alignment_error_m": "降落过程中相对真实视觉目标的水平对准误差 (米)，仅在目标世界坐标可用时计算",
    "final_landing_position_error_m": "最终着陆点与真实视觉目标点的水平距离 (米)，仅在目标世界坐标可用时计算",
    "landing_target_zone_radius_m": "视觉降落目标区域半径 (米)",
    "landing_target_center_xy_m": "视觉降落目标区域中心点 xy 坐标 (米)，仅在目标世界坐标可用时显示",
    "landing_target_source": "视觉降落目标中心来源",
    "landing_target_estimate_count": "用于估算视觉目标世界坐标的有效检测样本数",
    "landed_within_target_zone": "是否降落在目标区域内，仅在目标世界坐标可用时判断",
    "landing_start_xy_m": "视觉降落开始时无人机 xy 坐标 (米)",
    "landing_end_xy_m": "视觉降落结束时无人机 xy 坐标 (米)",
    "landing_start_to_end_xy_displacement_m": "视觉降落开始到结束的水平位移 (米)，诊断项，不等同于落点误差",
    "horizontal_travel_m": "视觉降落过程累计水平移动距离 (米)，诊断项",
    "landing_vertical_drop_m": "视觉降落过程下降高度 (米)",
    "landing_final_altitude_m": "视觉降落结束时高度 (米)",
    "final_descent_trigger_xy_error_m": "触发最终下降时相对真实视觉目标的水平偏差 (米)，仅在目标世界坐标可用时计算",
    "target_acquisition_time_s": "首次检测到降落目标的时间 (秒)",
    "target_tracking_continuity_rate": "目标跟踪连续率 (%)，反映视觉检测的稳定性",
    "target_tracking_continuity_rate_exempt_terminal_loss": "排除末端丢失后的目标跟踪连续率 (%)",
    "descent_stability_level": "下降过程稳定性等级",
    "visual_landing_success": "视觉降落是否成功完成",
    "visual_landing_aborted": "视觉降落是否被中止",
    "landing_duration_s": "有效降落耗时 (秒)，优先按姿态高度曲线中的持续下降窗口计算",
    "landing_case_duration_s": "视觉降落 case 总耗时 (秒)，包含 launch 启动、等待、降落和退出清理",
    "landing_duration_source": "有效降落耗时的数据来源",
    "landing_duration_fallback_reason": "有效降落耗时回退到 case 总耗时的原因",
    "landing_effective_start_offset_s": "有效降落窗口开始相对视觉降落 case 开始的时间 (秒)",
    "landing_effective_end_offset_s": "有效降落窗口结束相对视觉降落 case 开始的时间 (秒)",
    "landing_effective_start_altitude_m": "有效降落窗口开始高度 (米)",
    "landing_effective_end_altitude_m": "有效降落窗口结束高度 (米)",
    "landing_terminal_altitude_threshold_m": "判定有效降落窗口结束的末端高度阈值 (米)",
    "landing_safety_pass": "降落安全性检查是否通过",
    "hardware_check_timeout_s": "硬件检查的超时时间 (秒)",
    "battery_pass_threshold_v": "电池电压通过阈值 (伏特)，低于此值判定为失败",
    "takeoff_target_z_m": "起飞目标高度 (米)",
    "post_takeoff_settle_time_s": "起飞后等待稳定的时间 (秒)",
    "hover_duration_s": "悬停测试持续时间 (秒)",
    "waypoint_source": "航点来源：list 从配置读取，input 运行时手动输入",
    "waypoint_reach_radius_m": "航点到达判定半径 (米)，进入此范围视为到达",
    "waypoint_stable_time_s": "航点稳定判定时间 (秒)，在半径内持续此时间视为稳定",
    "waypoint_hold_time_s": "到达航点后的悬停保持时间 (秒)",
    "waypoint_timeout_s": "单个航点的超时时间 (秒)，超时未到达则失败",
    "waypoint_analysis_use_xy_only": "航点评分是否只使用水平距离判定到达",
    "ego_goal_source": "EGO 目标点来源：list 从配置读取，input 运行时通过 RViz 输入",
    "ego_goal_z_m": "EGO 目标点默认高度 (米)",
    "ego_goal_reach_radius_m": "EGO 目标点到达判定半径 (米)",
    "ego_goal_stable_time_s": "EGO 目标点稳定判定时间 (秒)",
    "ego_goal_hold_time_s": "EGO 目标点到达后的保持时间 (秒)",
    "ego_goal_timeout_s": "单个 EGO 目标点超时时间 (秒)",
    "ego_goal_republish_rate_hz": "EGO 目标点重复发布频率 (Hz)",
    "ego_goal_publish_burst_count": "EGO 目标点起始发布次数",
    "ego_goal_publish_burst_interval_s": "EGO 目标点起始发布间隔 (秒)",
    "ego_goal_use_xy_only": "EGO 目标点是否只使用水平距离判定到达",
    "ego_post_transition_enabled": "EGO 结束后是否自动执行偏航过渡",
    "ego_post_transition_target_yaw_rad": "EGO 后偏航过渡目标角 (弧度)",
    "ego_post_transition_yaw_rate_rad_s": "EGO 后偏航过渡角速度上限 (弧度/秒)",
    "ego_post_transition_hold_after_s": "EGO 后偏航过渡完成后的保持时间 (秒)",
    "ego_post_transition_target_z_m": "EGO 后偏航过渡期间的目标高度 (米)",
    "post_transition_enabled": "EGO 后是否执行偏航过渡",
    "post_transition_target_yaw_rad": "EGO 后偏航过渡目标角 (弧度)",
    "post_transition_yaw_rate_rad_s": "EGO 后偏航过渡角速度上限 (弧度/秒)",
    "post_transition_hold_after_s": "EGO 后偏航过渡完成后的保持时间 (秒)",
    "post_transition_target_z_m": "EGO 后偏航过渡期间的目标高度 (米)",
    "visual_landing_auto_takeoff": "视觉降落前是否自动起飞",
    "visual_landing_height_m": "视觉降落的起始高度 (米)",
    "visual_landing_target_zone_radius_m": "视觉降落目标区域半径 (米)",
    "launch_args": "启动视觉降落时附带的 launch 参数",
    "front_camera": "前视相机图像话题",
    "down_camera": "下视相机图像话题",
    "battery": "电池状态话题",
    "uav_state": "无人机状态话题",
    "uav_control_cmd": "无人机控制指令话题",
    "uav_setup": "无人机配置/设置话题",
    "lidar_pointcloud": "激光雷达点云话题",
    "ego_global_pointcloud": "EGO-Planner 全局点云话题",
    "ego_bspline": "EGO-Planner 轨迹 B-spline 话题",
    "ego_pos_cmd": "EGO-Planner 位置指令话题",
}

def escape(value: Any) -> str:
    if value is None:
        return "-"
    return html.escape(str(value))


def normalize_status(value: Any) -> str:
    return str(value or "").strip().lower()


def format_display_text(value: Any) -> str:
    if value in (None, ""):
        return "-"
    text = str(value)
    if "T" in text and len(text) >= 19:
        try:
            dt = datetime.fromisoformat(text)
            if dt.microsecond:
                return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            return dt.strftime("%Y-%m-%d %H:%M:%S")
        except ValueError:
            return text.replace("T", " ")
    if "." in text:
        prefix, suffix = text.split(".", 1)
        digits = "".join(ch for ch in suffix if ch.isdigit())
        if len(prefix) >= 19 and digits:
            return f"{prefix}.{digits[:3]}"
    return text


def format_time_short(value: Any) -> str:
    if value in (None, ""):
        return "-"
    text = str(value)
    try:
        dt = datetime.fromisoformat(text)
        if dt.microsecond:
            return dt.strftime("%H:%M:%S.") + f"{dt.microsecond // 1000:03d}"
        return dt.strftime("%H:%M:%S")
    except ValueError:
        pass
    if " " in text and len(text) >= 19:
        return text.split(" ", 1)[1]
    return text


def status_badge(result: Any) -> str:
    normalized = normalize_status(result)
    label = STATUS_LABELS.get(normalized, escape(result).upper())
    return f'<span class="status-badge status-{normalized or "unknown"}">{label}</span>'


def _score_for_result(result: Any) -> str:
    normalized = normalize_status(result)
    if normalized == "pass":
        return "100"
    if normalized in {"fail", "error"}:
        return "0"
    return "-"


def score_display(case: Dict[str, Any], grade_thresholds: List[Dict[str, Any]]) -> str:
    if case.get("category") == "hardware":
        return "-"
    score = case.get("score")
    if score is None:
        if case.get("category") == "flight":
            return "-"
        return _score_for_result(case.get("result"))
    score_val = float(score)
    color = "#69758a"
    for grade in sorted(grade_thresholds, key=lambda g: g.get("min", 0), reverse=True):
        if score_val >= grade.get("min", 0):
            color = grade.get("color", color)
            break
    return f'<span style="color:{color};font-weight:700">{score_val:.1f}</span>'


def pretty_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        if value and all(isinstance(item, (list, tuple)) for item in value):
            lines = []
            for item in value:
                point = ",".join(str(part) for part in item)
                lines.append(f"[{point}]")
            return "\n".join(lines)
        return "[" + ",".join(str(item) for item in value) + "]"
    if isinstance(value, dict):
        lines = []
        for key, item in value.items():
            pretty_item = pretty_value(item)
            if "\n" in pretty_item:
                indented = "\n".join(f"  {line}" for line in pretty_item.splitlines())
                lines.append(f"{key}:\n{indented}")
            else:
                lines.append(f"{key}: {pretty_item}")
        return "\n".join(lines) if lines else "-"
    if value in ("", None):
        return "-"
    return format_display_text(value)


def pretty_value_for_key(key: str, value: Any) -> str:
    return pretty_value(value)


def pretty_metric_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.5f}"
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(pretty_metric_value(item) for item in value) + "]"
    if isinstance(value, dict):
        lines = []
        for key, item in value.items():
            pretty_item = pretty_metric_value(item)
            if "\n" in pretty_item:
                indented = "\n".join(f"  {line}" for line in pretty_item.splitlines())
                lines.append(f"{key}:\n{indented}")
            else:
                lines.append(f"{key}: {pretty_item}")
        return "\n".join(lines) if lines else "-"
    return pretty_value(value)


def description_list(data: Dict[str, Any], preferred_order: Iterable[str] = (), compact: bool = False) -> str:
    ordered_keys: List[str] = []
    for key in preferred_order:
        if key == "mission_key":
            continue
        if key in data:
            ordered_keys.append(key)
    for key in data:
        if key == "mission_key":
            continue
        if key not in ordered_keys:
            ordered_keys.append(key)

    items = []
    cls = "kv-chip" if compact else "kv-item"
    for key in ordered_keys:
        desc = METRIC_DESCRIPTIONS.get(key)
        info_html = (
            f'<span class="metric-info">!<span class="metric-tooltip">{escape(desc)}</span></span>'
            if desc else ""
        )
        items.append(
            f"<div class=\"{cls}\">"
            f"<span class=\"kv-key\">{escape(key)}{info_html}</span>"
            f"<span class=\"kv-value\">{escape(pretty_value_for_key(key, data.get(key)))}</span>"
            "</div>"
        )
    if not items:
        return '<div class="empty-block">暂无数据</div>'
    if compact:
        return f'<div class="kv-chip-row">{"".join(items)}</div>'
    return "".join(items)


def render_metric_blocks(data: Dict[str, Any], preferred_order: Iterable[str] = ()) -> str:
    if not data:
        return '<div class="empty-block">暂无数据</div>'
    ordered_keys: List[str] = []
    for key in preferred_order:
        if key in data:
            ordered_keys.append(key)
    for key in data:
        if key not in ordered_keys:
            ordered_keys.append(key)

    items = []
    for key in ordered_keys:
        desc = METRIC_DESCRIPTIONS.get(key)
        info_html = (
            f'<span class="metric-info">!<span class="metric-tooltip">{escape(desc)}</span></span>'
            if desc else ""
        )
        items.append(
            '<div class="kv-item">'
            f'<span class="kv-key">{escape(key)}{info_html}</span>'
            f'<span class="kv-value">{escape(pretty_metric_value(data.get(key)))}</span>'
            "</div>"
        )
    return f'<div class="meta-grid">{"".join(items)}</div>'


def render_labeled_value(label: str, value: Any) -> str:
    desc = METRIC_DESCRIPTIONS.get(label)
    info_html = (
        f'<span class="metric-info">!<span class="metric-tooltip">{escape(desc)}</span></span>'
        if desc else ""
    )
    return (
        '<div style="font-size: 12px;">'
        f'<span style="font-weight: 700; color: var(--muted);">{escape(label)}{info_html}</span>'
        f'<div style="background: #f8fafc; border-radius: 4px; padding: 4px 8px; '
        f'font-family: var(--mono); margin-top: 2px;">{escape(pretty_value(value))}</div>'
        "</div>"
    )


def format_duration(started_at: Any, finished_at: Any) -> str:
    if not started_at or not finished_at:
        return "-"
    try:
        start = datetime.fromisoformat(str(started_at))
        end = datetime.fromisoformat(str(finished_at))
    except ValueError:
        return "-"

    delta = max((end - start).total_seconds(), 0.0)
    if delta < 1:
        return f"{delta:.2f}s"
    minutes, seconds = divmod(int(delta), 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}h {minutes}m {seconds}s"
    if minutes:
        return f"{minutes}m {seconds}s"
    return f"{delta:.2f}s"

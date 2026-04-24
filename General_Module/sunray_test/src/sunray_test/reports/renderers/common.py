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
    "stability": "稳定性",
    "robustness": "鲁棒性",
    "smoothness": "平静性",
    "functional_completeness": "功能完整性",
    "analysis_info": "分析信息",
}

METRIC_DESCRIPTIONS = {
    "xy_error_mean": "水平面 (XY) 位置误差的平均值，反映悬停水平定位精度",
    "abs_z_error_mean": "高度 (Z) 误差绝对值的平均值，反映悬停高度保持精度",
    "xy_error_rmse": "水平面位置误差的均方根值 (RMSE)，综合反映误差大小和波动",
    "z_error_rmse": "高度误差的均方根值 (RMSE)，综合反映高度误差大小和波动",
    "xy_error_p95": "水平面位置误差的第 95 百分位值，表示 95% 时间内的最大水平偏差",
    "abs_z_error_p95": "高度误差绝对值的第 95 百分位值，表示 95% 时间内的最大高度偏差",
    "xy_error_max": "水平面位置误差的最大值，反映最坏情况下的水平偏移",
    "z_error_max": "高度误差的最大值，反映最坏情况下的高度偏移",
    "speed_mean": "飞行速度的平均值，悬停时越小表示越稳定",
    "speed_p95": "飞行速度的第 95 百分位值，反映速度波动上限",
    "speed_max": "飞行速度的最大值，反映最大瞬时速度",
    "reach_success": "是否成功到达目标航点（进入阈值半径并稳定）",
    "reach_time_s": "从开始飞向航点到首次进入阈值半径的时间 (秒)",
    "settling_time_s": "从开始飞向航点到稳定在阈值半径内的时间 (秒)",
    "within_threshold": "最终位置是否在阈值半径内",
    "final_xy_error_m": "到达航点后水平面的最终位置误差 (米)",
    "final_z_error_m": "到达航点后高度的最终位置误差 (米)",
    "hold_xy_mean_m": "保持阶段水平面位置误差的平均值 (米)",
    "hold_xy_rmse_m": "保持阶段水平面位置误差的均方根值 (米)",
    "hold_xy_p95_m": "保持阶段水平面位置误差的第 95 百分位值 (米)",
    "hold_z_rmse_m": "保持阶段高度误差的均方根值 (米)",
    "speed_mean_mps": "飞行段平均速度 (米/秒)",
    "speed_p95_mps": "飞行段速度的第 95 百分位值 (米/秒)",
    "speed_max_mps": "飞行段最大瞬时速度 (米/秒)",
    "path_length_m": "实际飞行路径长度 (米)",
    "path_efficiency": "路径效率 = 直线距离 / 实际路径长度，越接近 1 越高效",
    "max_lateral_deviation_m": "飞行过程中偏离直线路径的最大横向距离 (米)",
    "overshoot_distance_m": "超越目标点的最大距离 (米)，反映减速控制能力",
    "stability_level": "综合稳定性等级：excellent / pass / fail",
    "horizontal_alignment_error_m": "降落过程中水平对准误差 (米)，反映视觉引导的水平精度",
    "final_landing_position_error_m": "最终着陆点与目标点的水平距离 (米)",
    "landed_within_target_zone": "是否降落在目标区域内",
    "final_descent_trigger_xy_error_m": "触发最终下降时的水平偏差 (米)",
    "final_yaw_alignment_error_deg": "最终偏航角对准误差 (度)",
    "target_acquisition_time_s": "首次检测到降落目标的时间 (秒)",
    "target_tracking_continuity_rate": "目标跟踪连续率 (%)，反映视觉检测的稳定性",
    "target_tracking_continuity_rate_exempt_terminal_loss": "排除末端丢失后的目标跟踪连续率 (%)",
    "target_loss_count": "目标丢失次数，0 表示全程无丢失",
    "max_target_loss_duration_s": "单次目标丢失的最长持续时间 (秒)",
    "descent_stability_level": "下降过程稳定性等级",
    "low_altitude_correction_pass": "低空修正是否通过",
    "touchdown_window_max_tilt_deg": "着陆窗口内最大倾斜角 (度)，反映着陆姿态稳定性",
    "bounce_rebound_height_m": "着陆弹跳高度 (米)，0 表示无弹跳",
    "visual_landing_success": "视觉降落是否成功完成",
    "visual_landing_aborted": "视觉降落是否被中止",
    "visual_guidance_completed": "视觉引导流程是否完整执行",
    "target_reacquisition_pass": "目标重新捕获能力是否通过",
    "landing_duration_s": "降落总耗时 (秒)",
    "landing_safety_pass": "降落安全性检查是否通过",
    "landing_target_xy": "视觉降落目标点的平面坐标 [X, Y] (米)，用于衡量最终落点偏差",
    "target_zone_radius_m": "目标区域半径 (米)，最终落点落在该范围内视为命中目标区域",
    "touchdown_window_s": "触地判定时间窗口 (秒)，用于统计接地瞬间的姿态与稳定性指标",
    "hardware_check_timeout_s": "硬件检查的超时时间 (秒)",
    "battery_pass_threshold_v": "电池电压通过阈值 (伏特)，低于此值判定为失败",
    "takeoff_target_pos": "起飞目标位置 [X, Y, Z] (米)，惯性系坐标",
    "post_takeoff_settle_time_s": "起飞后等待稳定的时间 (秒)",
    "hover_duration_s": "悬停测试持续时间 (秒)",
    "waypoint_source": "航点来源：list 从配置读取，input 运行时手动输入",
    "waypoint_reach_radius_m": "航点到达判定半径 (米)，进入此范围视为到达",
    "waypoint_stable_time_s": "航点稳定判定时间 (秒)，在半径内持续此时间视为稳定",
    "waypoint_hold_time_s": "到达航点后的悬停保持时间 (秒)",
    "waypoint_timeout_s": "单个航点的超时时间 (秒)，超时未到达则失败",
    "visual_landing_auto_takeoff": "视觉降落前是否自动起飞",
    "visual_landing_height_m": "视觉降落的起始高度 (米)",
    "front_camera": "前视相机图像话题",
    "down_camera": "下视相机图像话题",
    "battery": "电池状态话题",
    "uav_state": "无人机状态话题",
    "uav_control_cmd": "无人机控制指令话题",
    "uav_setup": "无人机配置/设置话题",
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


def format_number(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.5f}"
    return str(value)


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
    score = case.get("score")
    if score is None:
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
    if key == "landing_target_xy" and isinstance(value, (list, tuple)):
        return "[" + ",".join(format_number(item) for item in value) + "]"
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
    return f'<div class="meta-grid">{description_list(data, preferred_order)}</div>'


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

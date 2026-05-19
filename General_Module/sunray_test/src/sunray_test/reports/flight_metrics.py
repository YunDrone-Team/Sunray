import ast
import csv
import json
import math
import os
from typing import Any, Dict, List, Optional

from sunray_test.reports.scoring import compute_scores, load_scoring_config


def _normalize(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if hasattr(value, "item"):
        try:
            return _normalize(value.item())
        except Exception:
            pass
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return value
    return str(value)


def _load_event_log(path: str) -> List[Dict[str, Any]]:
    if not path or not os.path.exists(path):
        return []
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _case_artifacts_for_case(payload: Dict[str, Any], case_prefixes: tuple) -> Dict[str, Any]:
    for case in payload.get("cases", []):
        case_id = str(case.get("id", ""))
        if case_id in case_prefixes or any(case_id.startswith(prefix) for prefix in case_prefixes):
            artifacts = case.get("artifacts", {})
            return artifacts if isinstance(artifacts, dict) else {}
    return {}


def _backfill_case_times(payload: Dict[str, Any], event_log: List[Dict[str, Any]]) -> None:
    if not payload.get("cases") or not event_log:
        return

    start_times: Dict[str, str] = {}
    end_times: Dict[str, str] = {}
    for row in event_log:
        event_name = str(row.get("event", ""))
        detail = str(row.get("detail", ""))
        time_str = str(row.get("time_str", ""))
        if event_name == "case_start" and detail:
            start_times[detail] = time_str
        elif event_name == "case_end" and detail:
            case_id = detail.split(":", 1)[0]
            end_times[case_id] = time_str

    for case in payload.get("cases", []):
        case_id = str(case.get("id", ""))
        if case_id in start_times:
            case["started_at"] = start_times[case_id]
        if case_id in end_times:
            case["finished_at"] = end_times[case_id]


def _remove_skip_data(payload: Dict[str, Any]) -> None:
    summary = payload.get("summary")
    if isinstance(summary, dict) and "skip" in summary:
        summary.pop("skip", None)

    for case in payload.get("cases", []):
        if case.get("result") == "skip":
            case["result"] = "unsupported"


def _build_waypoint_metric_view(result: Dict[str, Any], threshold_m: float) -> Dict[str, Any]:
    if not result["reach_success_flag"]:
        return {
            "reach_success_flag": False,
            "reach_time": None,
            "settling_time": None,
            "final_xy_error": None,
            "final_abs_z_error": None,
            "hold_window_p95_xy_error": None,
            "path_length_m": None,
            "path_efficiency": None,
            "max_lateral_deviation_m": None,
            "max_speed": None,
            "overshoot_distance_m": None,
        }

    return {
        "reach_success_flag": True,
        "reach_time": result["reach_time"],
        "settling_time": result["settling_time"],
        "final_xy_error": result["final_xy_error"],
        "final_abs_z_error": result["final_abs_z_error"],
        "hold_window_p95_xy_error": result["hold_window_p95_xy_error"],
        "path_length_m": result["path_length_m"],
        "path_efficiency": result["path_efficiency"],
        "max_lateral_deviation_m": result["max_lateral_deviation_m"],
        "max_speed": result["max_speed"],
        "overshoot_distance_m": result["overshoot_distance_m"],
    }


def _find_case_window(event_log: List[Dict[str, Any]], case_prefixes: tuple) -> Optional[Dict[str, Any]]:
    starts: Dict[str, float] = {}
    for row in event_log:
        event_name = str(row.get("event", ""))
        detail = str(row.get("detail", ""))
        if not detail:
            continue
        timestamp = float(row.get("timestamp") or row.get("ros_timestamp") or row.get("wall_timestamp"))
        if event_name == "case_start" and any(detail == prefix or detail.startswith(prefix) for prefix in case_prefixes):
            starts[detail] = timestamp
        elif event_name == "case_end":
            case_id = detail.split(":", 1)[0]
            if case_id in starts:
                return {"case_id": case_id, "start_time": starts[case_id], "end_time": timestamp}
    return None


def _write_xyz_csv(csv_path: str, rows: List[List[float]]) -> None:
    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time", "x", "y", "z"])
        writer.writerows(rows)


def _load_builtin_hover_metrics(run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    event_log = payload.get("event_log") or _load_event_log(payload.get("artifacts", {}).get("event_log_jsonl", ""))
    window_info = _find_case_window(event_log, ("hover_stability", "hover"))
    if not window_info:
        raise ValueError("event log does not contain hover_stability case_start/case_end window")

    defaults = payload.get("config", {}).get("defaults", {})
    target_z = float(defaults.get("takeoff_target_z_m", 1.2))
    case_artifacts = _case_artifacts_for_case(payload, ("hover_stability", "hover"))
    hover_reference = case_artifacts.get("hover_reference_xyz")
    bag_path = payload.get("artifacts", {}).get("bag_file")
    if bag_path and not os.path.isabs(bag_path):
        bag_path = os.path.join(run_dir, bag_path)
    if not bag_path or not os.path.exists(bag_path):
        raise ValueError(f"rosbag not found for hover metrics: {bag_path or '<empty>'}")

    import numpy as np
    import rosbag

    rows: List[List[float]] = []
    with rosbag.Bag(bag_path) as bag:
        topic_info = bag.get_type_and_topic_info().topics
        pose_topic = _select_pose_topic_from_info(topic_info, payload)
        if pose_topic is None:
            raise ValueError("unable to select pose topic from rosbag for hover metrics")
        for _, msg, t in bag.read_messages(topics=[pose_topic]):
            timestamp = t.to_sec()
            if timestamp < window_info["start_time"] or timestamp > window_info["end_time"]:
                continue
            position = _position_from_msg(msg)
            if position is None:
                continue
            rows.append([timestamp, position[0], position[1], position[2]])

    if len(rows) < 2:
        raise ValueError(f"hover window has too few pose samples on topic {pose_topic}: {len(rows)}")

    data = np.asarray(sorted(rows, key=lambda row: row[0]), dtype=float)
    pos = data[:, 1:4]
    first_samples = pos[: min(len(pos), 20)]
    if len(first_samples) > 0:
        target_array = np.asarray(
            [
                float(np.median(first_samples[:, 0])),
                float(np.median(first_samples[:, 1])),
                float(np.median(first_samples[:, 2])),
            ],
            dtype=float,
        )
    elif isinstance(hover_reference, (list, tuple)) and len(hover_reference) >= 3:
        target_array = np.asarray([float(hover_reference[0]), float(hover_reference[1]), float(hover_reference[2])], dtype=float)
    else:
        target_array = np.asarray([float(np.median(first_samples[:, 0])), float(np.median(first_samples[:, 1])), target_z], dtype=float)
    err = pos - target_array
    xy_error = np.linalg.norm(err[:, :2], axis=1)
    z_error = err[:, 2]
    abs_z_error = np.abs(z_error)

    dt = np.diff(data[:, 0])
    delta = np.diff(pos, axis=0)
    valid_dt = dt > 0.005
    speeds = np.linalg.norm(delta[valid_dt], axis=1) / dt[valid_dt] if np.any(valid_dt) else np.asarray([], dtype=float)
    duration_s = float(window_info["end_time"] - window_info["start_time"])
    sample_rate_hz = float((len(data) - 1) / (data[-1, 0] - data[0, 0])) if data[-1, 0] > data[0, 0] else None

    csv_dir = os.path.join(run_dir, "data")
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, "hover_stability_xyz.csv")
    _write_xyz_csv(csv_path, rows)

    mean_xy_error = float(np.mean(xy_error))
    mean_abs_z_error = float(np.mean(abs_z_error))
    xy_error_rmse = float(np.sqrt(np.mean(np.square(xy_error))))
    z_error_rmse = float(np.sqrt(np.mean(np.square(z_error))))
    speed_mean = float(np.mean(speeds)) if len(speeds) else None
    speed_p95 = float(np.percentile(speeds, 95)) if len(speeds) else None
    speed_max = float(np.max(speeds)) if len(speeds) else None

    return _normalize(
        {
            "title": "悬停指标",
            "case_id": window_info["case_id"],
            "pose_topic": pose_topic,
            "target_xyz": target_array.tolist(),
            "window": {
                "start_time_s": window_info["start_time"],
                "end_time_s": window_info["end_time"],
                "duration_s": duration_s,
            },
            "artifacts": {
                "hover_csv": os.path.relpath(csv_path, run_dir),
            },
            "metrics_by_category": {
                "accuracy": {
                    "mean_xy_error": mean_xy_error,
                    "mean_abs_z_error": mean_abs_z_error,
                },
                "stability": {
                    "xy_error_rmse": xy_error_rmse,
                    "z_error_rmse": z_error_rmse,
                    "xy_error_p95": float(np.percentile(xy_error, 95)),
                    "abs_z_error_p95": float(np.percentile(abs_z_error, 95)),
                },
                "robustness": {
                    "xy_error_max": float(np.max(xy_error)),
                    "abs_z_error_max": float(np.max(abs_z_error)),
                },
                "smoothness": {
                    "speed_mean": speed_mean,
                    "speed_p95": speed_p95,
                    "speed_max": speed_max,
                },
            },
            "analysis_info": {
                "filtered_sample_count": int(len(rows)),
                "mean_dt_s": float(np.mean(dt[valid_dt])) if np.any(valid_dt) else None,
                "mean_rate_hz": sample_rate_hz,
                "analysis_source": "builtin_rosbag",
            },
        }
    )


def _load_hover_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return _load_builtin_hover_metrics(run_dir, payload)


def _load_waypoint_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return _load_builtin_waypoint_metrics(run_dir, payload)


def _parse_waypoint_events(event_log: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    starts: Dict[tuple, Dict[str, Any]] = {}
    segments: List[Dict[str, Any]] = []
    for row in event_log:
        event_name = str(row.get("event", ""))
        detail = str(row.get("detail", ""))
        if event_name not in {"waypoint_start", "waypoint_end", "waypoint_fail"} or not detail:
            continue
        parts = detail.split(":", 2)
        if len(parts) < 2:
            continue
        case_id = parts[0]
        try:
            index = int(parts[1])
        except ValueError:
            continue
        timestamp = float(row.get("timestamp") or row.get("ros_timestamp") or row.get("wall_timestamp"))
        key = (case_id, index)
        if event_name == "waypoint_start":
            waypoint = None
            if len(parts) >= 3:
                try:
                    parsed = ast.literal_eval(parts[2])
                    if isinstance(parsed, (list, tuple)) and len(parsed) >= 3:
                        waypoint = [float(parsed[0]), float(parsed[1]), float(parsed[2])]
                except (ValueError, SyntaxError):
                    waypoint = None
            starts[key] = {"case_id": case_id, "index": index, "start_time": timestamp, "wp": waypoint}
        elif key in starts:
            segment = dict(starts[key])
            segment["end_time"] = timestamp
            segment["success"] = event_name == "waypoint_end"
            segments.append(segment)
    return sorted(segments, key=lambda item: (item["start_time"], item["index"]))


def _point_to_line_distance(point: Any, start: Any, end: Any) -> float:
    import numpy as np

    line = end - start
    denom = float(np.dot(line, line))
    if denom <= 1e-9:
        return float(np.linalg.norm(point - start))
    t = max(0.0, min(1.0, float(np.dot(point - start, line) / denom)))
    projection = start + t * line
    return float(np.linalg.norm(point - projection))


def _load_builtin_waypoint_metrics(run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    event_log = payload.get("event_log") or _load_event_log(payload.get("artifacts", {}).get("event_log_jsonl", ""))
    waypoints = _parse_waypoint_events(event_log)
    if not waypoints:
        case_result = _case_result_for_section(payload, "waypoint")
        if case_result == "pass":
            raise ValueError("event log does not contain waypoint_start/waypoint_end events")
        return None

    defaults = payload.get("config", {}).get("defaults", {})
    threshold_m = float(defaults.get("waypoint_reach_radius_m", 0.15))
    stable_time_s = float(defaults.get("waypoint_stable_time_s", 2.0))
    use_xy_only = bool(defaults.get("waypoint_analysis_use_xy_only", False))

    bag_path = payload.get("artifacts", {}).get("bag_file")
    if bag_path and not os.path.isabs(bag_path):
        bag_path = os.path.join(run_dir, bag_path)
    if not bag_path or not os.path.exists(bag_path):
        raise ValueError(f"rosbag not found for waypoint metrics: {bag_path or '<empty>'}")

    import numpy as np
    import rosbag

    data = []
    with rosbag.Bag(bag_path) as bag:
        topic_name = _select_pose_topic_from_info(bag.get_type_and_topic_info().topics, payload)
        if topic_name is None:
            raise ValueError("unable to select pose topic from rosbag for waypoint metrics")
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            position = _position_from_msg(msg)
            if position is None:
                continue
            data.append([t.to_sec(), position[0], position[1], position[2]])
    if not data:
        raise ValueError(f"pose topic has no readable position samples for waypoint metrics: {topic_name}")

    raw = np.asarray(sorted(data, key=lambda row: row[0]), dtype=float)
    if len(raw) < 2:
        raise ValueError(f"not enough pose samples for waypoint metrics: {len(raw)}")
    dt = np.diff(raw[:, 0])
    delta = np.diff(raw[:, 1:4], axis=0)
    valid = dt > 0.005
    if not np.any(valid):
        raise ValueError("no valid timestamp intervals in pose samples for waypoint metrics")

    traj_time = raw[1:, 0][valid]
    traj_pos = raw[1:, 1:4][valid]
    traj_delta = delta[valid]
    traj_dt = dt[valid]
    traj_speed = np.linalg.norm(traj_delta, axis=1) / traj_dt

    mean_dt_s = float(np.mean(traj_dt))
    stable_count = max(1, int(np.ceil(stable_time_s / mean_dt_s)))
    results: List[Dict[str, Any]] = []

    for index, waypoint_info in enumerate(waypoints):
        waypoint = waypoint_info.get("wp")
        if not waypoint:
            continue
        if waypoint_info.get("end_time") is not None:
            segment_end_t = waypoint_info["end_time"]
        elif index < len(waypoints) - 1:
            segment_end_t = waypoints[index + 1]["start_time"]
        else:
            segment_end_t = float(traj_time[-1])

        segment_mask = (traj_time >= waypoint_info["start_time"]) & (traj_time <= segment_end_t)
        segment_time = traj_time[segment_mask]
        segment_pos = traj_pos[segment_mask]
        segment_delta = traj_delta[segment_mask]
        segment_speed = traj_speed[segment_mask]
        if len(segment_time) < 5:
            results.append(
                {
                    "waypoint": waypoint,
                    "metrics": _build_waypoint_metric_view({"reach_success_flag": False}, threshold_m),
                }
            )
            continue

        wp_array = np.asarray(waypoint, dtype=float)
        wp_delta = segment_pos - wp_array
        distance = np.linalg.norm(wp_delta[:, :2], axis=1) if use_xy_only else np.linalg.norm(wp_delta, axis=1)
        inside = distance <= threshold_m
        first_entry_time = float(segment_time[np.argmax(inside)]) if np.any(inside) else None
        settled_time = None
        if np.any(inside) and len(inside) >= stable_count:
            rolling_sum = np.convolve(inside.astype(int), np.ones(stable_count, dtype=int), mode="valid")
            stable_indices = np.flatnonzero(rolling_sum >= stable_count)
            if len(stable_indices):
                settled_time = float(segment_time[stable_indices[0] + stable_count - 1])

        reach_success_flag = bool(waypoint_info.get("success")) and settled_time is not None
        if not reach_success_flag:
            result = {"waypoint": waypoint, "reach_success_flag": False}
        else:
            flight_mask = segment_time <= settled_time
            hold_mask = segment_time > settled_time
            flight_pos = segment_pos[flight_mask]
            flight_delta = segment_delta[flight_mask]
            flight_speed = segment_speed[flight_mask]
            hold_pos = segment_pos[hold_mask]

            final_error = flight_pos[-1] - wp_array
            final_xy_error = float(np.linalg.norm(final_error[:2]))
            final_abs_z_error = float(abs(final_error[2]))
            max_speed = float(np.max(flight_speed))
            path_length_m = float(np.linalg.norm(flight_delta, axis=1).sum())
            start_pos = flight_pos[0]
            direct_distance = float(np.linalg.norm(wp_array - start_pos))
            path_efficiency = float(direct_distance / path_length_m) if path_length_m > 1e-9 else None
            max_lateral_deviation_m = float(max(_point_to_line_distance(point, start_pos, wp_array) for point in flight_pos))
            overshoot_distance_m = _compute_overshoot_distance(flight_pos, start_pos, wp_array)
            reach_time = float(first_entry_time - waypoint_info["start_time"]) if first_entry_time is not None else None
            settling_time = float(settled_time - waypoint_info["start_time"])

            if len(hold_pos) > 3:
                hold_error = hold_pos - wp_array
                hold_xy = np.linalg.norm(hold_error[:, :2], axis=1)
                hold_z = hold_error[:, 2]
                hold_window_p95_xy_error = float(np.percentile(hold_xy, 95))
            else:
                hold_window_p95_xy_error = final_xy_error

            result = {
                "waypoint": waypoint,
                "final_xy_error": final_xy_error,
                "final_abs_z_error": final_abs_z_error,
                "max_speed": max_speed,
                "overshoot_distance_m": overshoot_distance_m,
                "path_length_m": path_length_m,
                "path_efficiency": path_efficiency,
                "max_lateral_deviation_m": max_lateral_deviation_m,
                "reach_success_flag": reach_success_flag,
                "reach_time": reach_time,
                "settling_time": settling_time,
                "hold_window_p95_xy_error": hold_window_p95_xy_error,
            }

        results.append({"waypoint": waypoint, "metrics": _build_waypoint_metric_view(result, threshold_m)})

    if not results:
        raise ValueError("no waypoint segments had enough pose samples for metrics")

    metric_views = [item["metrics"] for item in results]
    successful_views = [item for item in metric_views if item.get("reach_success_flag")]
    path_efficiencies = [
        item["path_efficiency"]
        for item in successful_views
        if isinstance(item.get("path_efficiency"), (int, float)) and not isinstance(item.get("path_efficiency"), bool)
    ]
    final_xy_errors = [
        item["final_xy_error"]
        for item in successful_views
        if isinstance(item.get("final_xy_error"), (int, float)) and not isinstance(item.get("final_xy_error"), bool)
    ]
    segment_end_times = [item.get("end_time") for item in waypoints if item.get("end_time") is not None]
    summary: Dict[str, Any] = {
        "wp_success_count": len(successful_views),
        "all_wp_success_flag": len(successful_views) == len(results),
        "total_waypoint_time_s": float(max(segment_end_times) - waypoints[0]["start_time"]) if segment_end_times else None,
        "mean_path_efficiency": float(sum(path_efficiencies) / len(path_efficiencies)) if path_efficiencies else None,
        "worst_final_xy_error": float(max(final_xy_errors)) if final_xy_errors else None,
    }

    return _normalize(
        {
            "title": "航点飞行指标",
            "pose_topic": topic_name,
            "threshold_m": threshold_m,
            "stable_time_s": stable_time_s,
            "use_xy_only": use_xy_only,
            "summary": summary,
            "waypoints": results,
        }
    )


def _parse_ego_goal_events(event_log: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    starts: Dict[tuple, Dict[str, Any]] = {}
    segments: List[Dict[str, Any]] = []
    for row in event_log:
        event_name = str(row.get("event", ""))
        detail = str(row.get("detail", ""))
        if event_name not in {"ego_goal_start", "ego_goal_end", "ego_goal_fail"} or not detail:
            continue
        parts = detail.split(":", 2)
        if len(parts) < 2:
            continue
        case_id = parts[0]
        try:
            index = int(parts[1])
        except ValueError:
            continue
        timestamp = float(row.get("timestamp") or row.get("ros_timestamp") or row.get("wall_timestamp"))
        key = (case_id, index)
        if event_name == "ego_goal_start":
            goal = None
            if len(parts) >= 3:
                try:
                    parsed = ast.literal_eval(parts[2])
                    if isinstance(parsed, (list, tuple)) and len(parsed) >= 2:
                        goal = [float(parsed[0]), float(parsed[1]), float(parsed[2]) if len(parsed) >= 3 else 0.0]
                except (ValueError, SyntaxError):
                    goal = None
            starts[key] = {"case_id": case_id, "index": index, "start_time": timestamp, "goal": goal}
        elif key in starts:
            segment = dict(starts[key])
            segment["end_time"] = timestamp
            segment["success"] = event_name == "ego_goal_end"
            segments.append(segment)
    return sorted(segments, key=lambda item: (item["start_time"], item["index"]))


def _select_pose_topic_from_info(topic_info: Dict[str, Any], payload: Dict[str, Any]) -> Optional[str]:
    analysis_config = payload.get("config", {}).get("analysis", {})
    configured = analysis_config.get("pose_topic")
    fallback_topics = analysis_config.get("pose_topic_fallbacks") or []
    candidate_topics: List[str] = []
    for topic in [configured] + list(fallback_topics):
        if topic and topic not in candidate_topics:
            candidate_topics.append(topic)
    if candidate_topics:
        for topic in candidate_topics:
            if topic in topic_info:
                return topic
        available = ", ".join(sorted(topic_info.keys())[:20])
        raise ValueError(
            "configured analysis pose topics not found in rosbag: "
            f"{candidate_topics}; available topics: {available}"
        )

    legacy_configured = (
        payload.get("config", {})
        .get("topics", {})
        .get("local_position_pose")
    )
    if legacy_configured:
        if legacy_configured in topic_info:
            return legacy_configured
        available = ", ".join(sorted(topic_info.keys())[:20])
        raise ValueError(f"configured topics.local_position_pose not found in rosbag: {legacy_configured}; available topics: {available}")
    preferred_suffixes = (
        "/mavros/local_position/pose",
        "/sunray/uav_state",
    )
    for suffix in preferred_suffixes:
        for topic in topic_info:
            if topic.endswith(suffix):
                return topic
    for topic, info in topic_info.items():
        msg_type = getattr(info, "msg_type", "")
        if msg_type in {"geometry_msgs/PoseStamped", "nav_msgs/Odometry"}:
            return topic
    return None


def _position_from_msg(msg: Any) -> Optional[List[float]]:
    pose = getattr(msg, "pose", None)
    if pose is not None and hasattr(pose, "pose"):
        pose = pose.pose
    position = getattr(pose, "position", None)
    if position is not None:
        return [float(position.x), float(position.y), float(position.z)]
    position = getattr(msg, "position", None)
    if position is not None:
        return [float(position.x), float(position.y), float(position.z)]
    return None


def _distance_to_goal(row: Any, goal: List[float], use_xy_only: bool) -> float:
    dx = float(row["x"] - goal[0])
    dy = float(row["y"] - goal[1])
    if use_xy_only:
        return float((dx ** 2 + dy ** 2) ** 0.5)
    dz = float(row["z"] - goal[2])
    return float((dx ** 2 + dy ** 2 + dz ** 2) ** 0.5)


def _compute_overshoot_distance(points: Any, start: Any, end: Any) -> float:
    import numpy as np

    direction = end - start
    denom = float(np.dot(direction, direction))
    if denom <= 1e-9:
        return 0.0
    projections = np.dot(points - start, direction) / denom
    overshoot = projections - 1.0
    if len(overshoot) == 0:
        return 0.0
    return float(max(0.0, float(np.max(overshoot)) * np.linalg.norm(direction)))


def _compute_path_heading_change_deg(points: Any, min_step_m: float = 0.02) -> Optional[float]:
    import numpy as np

    if len(points) < 3:
        return None
    reduced_points = [points[0]]
    last_point = points[0]
    for point in points[1:]:
        if float(np.linalg.norm(point - last_point)) >= min_step_m:
            reduced_points.append(point)
            last_point = point
    if len(reduced_points) < 2 or not np.array_equal(reduced_points[-1], points[-1]):
        reduced_points.append(points[-1])
    if len(reduced_points) < 3:
        return None

    vectors = np.diff(np.asarray(reduced_points), axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    if len(vectors) < 2:
        return None

    heading_change = 0.0
    for prev_vec, next_vec, prev_len, next_len in zip(vectors[:-1], vectors[1:], lengths[:-1], lengths[1:]):
        denom = float(prev_len * next_len)
        if denom <= 1e-9:
            continue
        cos_angle = float(np.dot(prev_vec, next_vec) / denom)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        heading_change += float(np.degrees(np.arccos(cos_angle)))
    return heading_change


def _compute_path_backtrack_distance_m(points: Any, start: Any, end: Any, min_backtrack_m: float = 0.01) -> Optional[float]:
    import numpy as np

    if len(points) < 2:
        return None
    direction = end - start
    direct_distance = float(np.linalg.norm(direction))
    if direct_distance <= 1e-9:
        return None
    unit_direction = direction / direct_distance
    deltas = np.diff(points, axis=0)
    projections = np.dot(deltas, unit_direction)
    backtracks = [-float(value) for value in projections if value < -min_backtrack_m]
    return float(sum(backtracks))


def _build_ego_goal_metric_groups(metrics: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    groups = {
        "completion": ("goal_success", "within_threshold", "goal_completion_rate"),
        "precision_stability": ("final_xy_error_m", "hold_xy_mean_m", "hold_xy_p95_m"),
        "path_quality": (
            "path_length_m",
            "path_efficiency",
            "path_detour_ratio",
            "path_heading_change_deg",
            "path_backtrack_distance_m",
        ),
        "response_smoothness": ("reach_time_s", "settling_time_s", "overshoot_distance_m", "speed_mean_mps", "speed_p95_mps"),
    }
    return {
        group_name: {key: metrics.get(key) for key in keys if key in metrics}
        for group_name, keys in groups.items()
    }


def _goal_metric_view(
    result: Dict[str, Any],
    threshold_m: float,
    completion_rate: float,
    use_xy_only: bool = True,
) -> Dict[str, Any]:
    if not result["goal_success"]:
        return {
            "goal_success": False,
            "reach_time_s": None,
            "settling_time_s": None,
            "within_threshold": False,
            "final_xy_error_m": None,
            "hold_xy_mean_m": None,
            "hold_xy_p95_m": None,
            "speed_mean_mps": None,
            "speed_p95_mps": None,
            "path_length_m": None,
            "path_efficiency": None,
            "path_detour_ratio": None,
            "path_heading_change_deg": None,
            "path_backtrack_distance_m": None,
            "overshoot_distance_m": None,
            "goal_completion_rate": completion_rate,
        }
    within_threshold = result["final_xy_error"] <= threshold_m
    if not use_xy_only:
        within_threshold = within_threshold and result["final_abs_z_error"] <= threshold_m

    return {
        "goal_success": True,
        "reach_time_s": result["reach_time"],
        "settling_time_s": result["settling_time"],
        "within_threshold": bool(within_threshold),
        "final_xy_error_m": result["final_xy_error"],
        "hold_xy_mean_m": result["xy_mean"],
        "hold_xy_p95_m": result["hold_window_p95_xy_error"],
        "speed_mean_mps": result["speed_mean"],
        "speed_p95_mps": result["speed_95"],
        "path_length_m": result["path_length_m"],
        "path_efficiency": result["path_efficiency"],
        "path_detour_ratio": result["path_detour_ratio"],
        "path_heading_change_deg": result["path_heading_change_deg"],
        "path_backtrack_distance_m": result["path_backtrack_distance_m"],
        "overshoot_distance_m": result["overshoot_distance_m"],
        "goal_completion_rate": completion_rate,
    }


def _load_ego_goal_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    event_log = payload.get("event_log") or _load_event_log(payload.get("artifacts", {}).get("event_log_jsonl", ""))
    goal_segments = _parse_ego_goal_events(event_log)
    if not goal_segments:
        case_result = _case_result_for_section(payload, "ego_goal")
        if case_result == "pass":
            raise ValueError("event log does not contain ego_goal_start/ego_goal_end events")
        return None

    defaults = payload.get("config", {}).get("defaults", {})
    reach_radius_m = float(defaults.get("ego_goal_reach_radius_m", defaults.get("waypoint_reach_radius_m", 0.15)))
    stable_time_s = float(defaults.get("ego_goal_stable_time_s", defaults.get("waypoint_stable_time_s", 2.0)))
    use_xy_only = bool(defaults.get("ego_goal_use_xy_only", True))

    bag_path = payload.get("artifacts", {}).get("bag_file")
    if bag_path and not os.path.isabs(bag_path):
        bag_path = os.path.join(run_dir, bag_path)
    if not bag_path or not os.path.exists(bag_path):
        raise ValueError(f"rosbag not found for EGO goal metrics: {bag_path or '<empty>'}")

    import numpy as np
    import rosbag

    data = []
    with rosbag.Bag(bag_path) as bag:
        topic_name = _select_pose_topic_from_info(bag.get_type_and_topic_info().topics, payload)
        if topic_name is None:
            raise ValueError("unable to select pose topic from rosbag for EGO goal metrics")
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            position = _position_from_msg(msg)
            if position is None:
                continue
            data.append([t.to_sec(), position[0], position[1], position[2]])
    if not data:
        raise ValueError(f"pose topic has no readable position samples for EGO goal metrics: {topic_name}")

    raw = np.asarray(sorted(data, key=lambda row: row[0]), dtype=float)
    if len(raw) < 2:
        raise ValueError(f"not enough pose samples for EGO goal metrics: {len(raw)}")
    dt = np.diff(raw[:, 0])
    delta = np.diff(raw[:, 1:4], axis=0)
    valid = dt > 0.005
    if not np.any(valid):
        raise ValueError("no valid timestamp intervals in pose samples for EGO goal metrics")

    traj_time = raw[1:, 0][valid]
    traj_pos = raw[1:, 1:4][valid]
    traj_delta = delta[valid]
    traj_dt = dt[valid]
    traj_speed = np.linalg.norm(traj_delta, axis=1) / traj_dt

    mean_dt_s = float(np.mean(traj_dt))
    stable_count = max(1, int(np.ceil(stable_time_s / mean_dt_s)))
    planned_count = max(len(goal_segments), 1)
    success_count = len([item for item in goal_segments if item.get("success")])
    completion_rate = float(success_count / planned_count)

    results: List[Dict[str, Any]] = []
    for segment_info in goal_segments:
        goal = segment_info.get("goal")
        if not goal:
            continue
        mask = (traj_time >= segment_info["start_time"]) & (traj_time <= segment_info["end_time"])
        segment_time = traj_time[mask]
        segment_pos = traj_pos[mask]
        segment_delta = traj_delta[mask]
        segment_speed = traj_speed[mask]
        if len(segment_time) < 5:
            results.append(
                {
                    "waypoint": goal,
                    "metrics": _goal_metric_view(
                        {"goal_success": False},
                        reach_radius_m,
                        completion_rate,
                        use_xy_only,
                    ),
                }
            )
            results[-1]["metrics_by_category"] = _build_ego_goal_metric_groups(results[-1]["metrics"])
            continue

        goal_array = np.asarray(goal, dtype=float)
        goal_delta = segment_pos - goal_array
        if use_xy_only:
            goal_distance = np.linalg.norm(goal_delta[:, :2], axis=1)
        else:
            goal_distance = np.linalg.norm(goal_delta, axis=1)
        inside = goal_distance <= reach_radius_m
        first_entry_time = float(segment_time[np.argmax(inside)]) if np.any(inside) else None
        settled_time = None
        if np.any(inside) and len(inside) >= stable_count:
            inside_int = inside.astype(int)
            rolling_sum = np.convolve(inside_int, np.ones(stable_count, dtype=int), mode="valid")
            stable_indices = np.flatnonzero(rolling_sum >= stable_count)
            if len(stable_indices):
                settled_time = float(segment_time[stable_indices[0] + stable_count - 1])

        goal_success = bool(segment_info.get("success")) and settled_time is not None
        if not goal_success:
            result = {"goal_success": False}
        else:
            flight_mask = segment_time <= settled_time
            hold_mask = segment_time > settled_time
            flight_pos = segment_pos[flight_mask]
            flight_delta = segment_delta[flight_mask]
            flight_speed = segment_speed[flight_mask]
            hold_pos = segment_pos[hold_mask]

            final_error = flight_pos[-1] - goal_array
            final_xy_error = float(np.linalg.norm(final_error[:2]))
            final_abs_z_error = float(abs(final_error[2]))
            speed_mean = float(np.mean(flight_speed))
            speed_95 = float(np.percentile(flight_speed, 95))
            path_length_m = float(np.linalg.norm(flight_delta, axis=1).sum())
            start_pos = flight_pos[0]
            end_pos = goal_array
            direct_distance = float(np.linalg.norm(end_pos - start_pos))
            path_efficiency = float(direct_distance / path_length_m) if path_length_m > 1e-9 else None
            path_detour_ratio = float(path_length_m / direct_distance) if direct_distance > 1e-9 else None
            path_heading_change_deg = _compute_path_heading_change_deg(flight_pos)
            path_backtrack_distance_m = _compute_path_backtrack_distance_m(flight_pos, start_pos, end_pos)
            overshoot_distance_m = _compute_overshoot_distance(flight_pos, start_pos, end_pos)
            reach_time = float(first_entry_time - segment_info["start_time"]) if first_entry_time is not None else None
            settling_time = float(settled_time - segment_info["start_time"])

            if len(hold_pos) > 3:
                hold_xy = np.linalg.norm((hold_pos - goal_array)[:, :2], axis=1)
                xy_mean = float(np.mean(hold_xy))
                hold_window_p95_xy_error = float(np.percentile(hold_xy, 95))
            else:
                xy_mean = final_xy_error
                hold_window_p95_xy_error = final_xy_error

            result = {
                "goal_success": True,
                "final_xy_error": final_xy_error,
                "final_abs_z_error": final_abs_z_error,
                "speed_mean": speed_mean,
                "speed_95": speed_95,
                "overshoot_distance_m": overshoot_distance_m,
                "path_length_m": path_length_m,
                "path_efficiency": path_efficiency,
                "path_detour_ratio": path_detour_ratio,
                "path_heading_change_deg": path_heading_change_deg,
                "path_backtrack_distance_m": path_backtrack_distance_m,
                "reach_time": reach_time,
                "settling_time": settling_time,
                "xy_mean": xy_mean,
                "hold_window_p95_xy_error": hold_window_p95_xy_error,
            }

        metrics = _goal_metric_view(result, reach_radius_m, completion_rate, use_xy_only)
        results.append(
            {
                "waypoint": goal,
                "metrics": metrics,
                "metrics_by_category": _build_ego_goal_metric_groups(metrics),
            }
        )

    if not results:
        raise ValueError("no EGO goal segments had enough pose samples for metrics")

    summary: Dict[str, Any] = {}
    metric_views = [item["metrics"] for item in results]
    for key in metric_views[0].keys():
        values = [item[key] for item in metric_views]
        numeric_values = [v for v in values if isinstance(v, (int, float)) and not isinstance(v, bool)]
        bool_values = [v for v in values if isinstance(v, bool)]
        text_values = [v for v in values if isinstance(v, str)]
        if numeric_values:
            summary[key] = float(sum(numeric_values) / len(numeric_values))
        elif bool_values:
            summary[key] = all(bool_values)
        elif text_values:
            summary[key] = text_values[-1]
        else:
            summary[key] = None

    return _normalize(
        {
            "title": "EGO自主规划指标",
            "pose_topic": topic_name,
            "threshold_m": reach_radius_m,
            "stable_time_s": stable_time_s,
            "summary_by_category": _build_ego_goal_metric_groups(summary),
            "waypoints": results,
            "metrics_by_category": {},
        }
    )


def _find_case(payload: Dict[str, Any], case_prefixes: tuple) -> Optional[Dict[str, Any]]:
    for case in payload.get("cases", []):
        case_id = str(case.get("id", ""))
        if case_id in case_prefixes or any(case_id.startswith(prefix) for prefix in case_prefixes):
            return case
    return None


def _load_builtin_landing_metrics(run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    event_log = payload.get("event_log") or _load_event_log(payload.get("artifacts", {}).get("event_log_jsonl", ""))
    window_info = _find_case_window(event_log, ("visual_landing",))
    case = _find_case(payload, ("visual_landing",))
    if not window_info and not case:
        return None

    case_metrics = case.get("metrics", {}) if isinstance(case, dict) else {}
    defaults = payload.get("config", {}).get("defaults", {})
    target_zone_radius_m = float(defaults.get("visual_landing_target_zone_radius_m", 0.15))
    return_code = case_metrics.get("return_code")
    matched_failure_patterns = case_metrics.get("matched_failure_patterns") or []
    case_result = str(case.get("result", "") if case else "").strip().lower()
    duration_s = (
        float(window_info["end_time"] - window_info["start_time"])
        if window_info
        else None
    )

    analysis: Dict[str, Any] = {
        "title": "视觉降落指标",
        "window": {
            "start_time_s": window_info["start_time"] if window_info else None,
            "end_time_s": window_info["end_time"] if window_info else None,
            "duration_s": duration_s,
        },
        "metrics_by_category": {
            "completion": {
                "visual_landing_success": case_result == "pass",
                "visual_landing_aborted": case_result in {"fail", "error"} or bool(matched_failure_patterns),
                "landing_safety_pass": case_result == "pass" and not matched_failure_patterns,
                "landed_within_target_zone": None,
                "landing_duration_s": duration_s,
            }
        },
        "artifacts": {},
        "limitations": [],
    }

    bag_path = payload.get("artifacts", {}).get("bag_file")
    if bag_path and not os.path.isabs(bag_path):
        bag_path = os.path.join(run_dir, bag_path)
    if not window_info or not bag_path or not os.path.exists(bag_path):
        return _normalize(analysis)

    import numpy as np
    import rosbag

    rows: List[List[float]] = []
    with rosbag.Bag(bag_path) as bag:
        pose_topic = _select_pose_topic_from_info(bag.get_type_and_topic_info().topics, payload)
        if pose_topic is None:
            return _normalize(analysis)
        for _, msg, t in bag.read_messages(topics=[pose_topic]):
            timestamp = t.to_sec()
            if timestamp < window_info["start_time"] or timestamp > window_info["end_time"]:
                continue
            position = _position_from_msg(msg)
            if position is None:
                continue
            rows.append([timestamp, position[0], position[1], position[2]])

    if len(rows) < 2:
        return _normalize(analysis)

    rows = sorted(rows, key=lambda row: row[0])
    data = np.asarray(rows, dtype=float)
    pos = data[:, 1:4]
    dt = np.diff(data[:, 0])
    delta = np.diff(pos, axis=0)
    valid_dt = dt > 0.005
    speeds = np.linalg.norm(delta[valid_dt], axis=1) / dt[valid_dt] if np.any(valid_dt) else np.asarray([], dtype=float)
    horizontal_travel = float(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1).sum()) if len(pos) > 1 else 0.0
    z_drop = float(pos[0, 2] - pos[-1, 2])
    touchdown_xy_offset = float(np.linalg.norm(pos[-1, :2] - pos[0, :2]))
    horizontal_alignment_error = touchdown_xy_offset
    final_descent_trigger_xy_error = touchdown_xy_offset
    if len(pos) > 1:
        descent_trigger_indices = np.flatnonzero(pos[:, 2] <= 0.5)
        if len(descent_trigger_indices):
            trigger_pos = pos[int(descent_trigger_indices[0]), :2]
            final_descent_trigger_xy_error = float(np.linalg.norm(trigger_pos - pos[0, :2]))
    landed_within_target_zone = touchdown_xy_offset <= target_zone_radius_m
    target_acquisition_time_s = 0.0 if case_result == "pass" else None
    target_tracking_continuity_rate = 100.0 if case_result == "pass" and not matched_failure_patterns else 0.0
    descent_stability_level = "excellent"
    if len(speeds):
        speed_p95 = float(np.percentile(speeds, 95))
        if speed_p95 > 0.6 or horizontal_travel > 0.6:
            descent_stability_level = "fail"
        elif speed_p95 > 0.35 or horizontal_travel > 0.35:
            descent_stability_level = "pass"

    csv_dir = os.path.join(run_dir, "data")
    os.makedirs(csv_dir, exist_ok=True)
    pose_csv = os.path.join(csv_dir, "visual_landing_pose.csv")
    _write_xyz_csv(pose_csv, rows)

    analysis["pose_topic"] = pose_topic
    analysis["artifacts"]["pose_window_csv"] = os.path.relpath(pose_csv, run_dir)
    analysis["metrics_by_category"]["trajectory"] = {
        "horizontal_alignment_error_m": horizontal_alignment_error,
        "final_landing_position_error_m": touchdown_xy_offset,
        "landing_target_zone_radius_m": target_zone_radius_m,
        "landing_target_center_xy_m": [0.0, 0.0],
        "final_descent_trigger_xy_error_m": final_descent_trigger_xy_error,
        "target_acquisition_time_s": target_acquisition_time_s,
        "target_tracking_continuity_rate": target_tracking_continuity_rate,
        "target_tracking_continuity_rate_exempt_terminal_loss": target_tracking_continuity_rate,
        "descent_stability_level": descent_stability_level,
    }
    analysis["metrics_by_category"]["completion"]["landed_within_target_zone"] = landed_within_target_zone
    analysis["metrics_by_category"]["completion"]["landing_safety_pass"] = (
        case_result == "pass"
        and not matched_failure_patterns
        and float(pos[-1, 2]) <= 0.25
        and z_drop > 0.5
    )
    return _normalize(analysis)


def _load_landing_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return _load_builtin_landing_metrics(run_dir, payload)


def _resolve_analysis_root(run_dir: str, workspace_root: Optional[str]) -> str:
    if workspace_root:
        return os.path.abspath(workspace_root)
    return os.path.abspath(os.path.join(run_dir, "..", "..", ".."))


def _case_result_for_section(payload: Dict[str, Any], section_key: str) -> Optional[str]:
    prefixes = {
        "hover": ("hover_stability", "hover"),
        "waypoint": ("waypoint_flight", "waypoint"),
        "ego_goal": ("ego_goal_flight", "ego_goal"),
        "visual_landing": ("visual_landing",),
    }.get(section_key, ())
    for case in payload.get("cases", []):
        case_id = str(case.get("id", ""))
        if case_id in prefixes or any(case_id.startswith(prefix) for prefix in prefixes):
            return str(case.get("result", "")).strip().lower()
    return None


def enrich_report_payload(payload: Dict[str, Any], workspace_root: Optional[str] = None) -> Dict[str, Any]:
    run_dir = payload.get("run_info", {}).get("run_dir") or payload.get("artifacts", {}).get("run_dir")
    _remove_skip_data(payload)
    if not run_dir or not os.path.isdir(run_dir):
        return payload

    analysis_root = _resolve_analysis_root(run_dir, workspace_root)

    event_log_path = payload.get("artifacts", {}).get("event_log_jsonl", "")
    if event_log_path and not os.path.isabs(event_log_path):
        event_log_path = os.path.join(run_dir, event_log_path)
    payload["event_log"] = _load_event_log(event_log_path)
    _backfill_case_times(payload, payload["event_log"])

    sections = []
    errors = []
    loader_specs = (
        ("hover", "悬停指标", _load_hover_metrics),
        ("waypoint", "航点飞行指标", _load_waypoint_metrics),
        ("ego_goal", "EGO自主规划指标", _load_ego_goal_metrics),
        ("visual_landing", "视觉降落指标", _load_landing_metrics),
    )
    for section_key, section_title, loader in loader_specs:
        loader_failed = False
        try:
            result = loader(analysis_root, run_dir, payload)
        except Exception as exc:
            errors.append(f"{section_title}: {exc}")
            loader_failed = True
            result = None
        if result:
            sections.append(result)
        elif not loader_failed and _case_result_for_section(payload, section_key) == "pass":
            errors.append(f"{section_title}: metrics not generated")

    payload["flight_metrics"] = {
        "title": "飞行指标",
        "sections": sections,
        "errors": errors,
    }

    scoring_roots = [run_dir, analysis_root]
    if workspace_root:
        scoring_roots.append(workspace_root)
    scoring_config = None
    for root in scoring_roots:
        scoring_config = load_scoring_config(root)
        if scoring_config:
            break
    if scoring_config:
        compute_scores(payload, scoring_config)

    return payload

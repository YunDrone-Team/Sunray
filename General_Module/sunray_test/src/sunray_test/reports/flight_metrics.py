import importlib.util
import json
import math
import os
from typing import Any, Dict, List, Optional

from sunray_test.reports.scoring import compute_scores, load_scoring_config


def _load_module(module_name: str, file_path: str):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"unable to load module from {file_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
            "reach_success": False,
            "reach_time_s": None,
            "settling_time_s": None,
            "within_threshold": False,
            "final_xy_error_m": None,
            "final_z_error_m": None,
            "hold_xy_mean_m": None,
            "hold_xy_rmse_m": None,
            "hold_xy_p95_m": None,
            "hold_z_rmse_m": None,
            "speed_mean_mps": None,
            "speed_p95_mps": None,
            "speed_max_mps": None,
            "path_length_m": None,
            "path_efficiency": None,
            "max_lateral_deviation_m": None,
            "overshoot_distance_m": None,
            "stability_level": "fail",
        }

    hold_xy_mean_m = result["xy_mean"] if result["xy_mean"] is not None else result["final_xy_error"]
    stability_level = "unknown"
    if result["speed_95"] is not None and hold_xy_mean_m is not None:
        if result["speed_95"] <= 0.8 and hold_xy_mean_m <= 0.08:
            stability_level = "excellent"
        elif result["speed_95"] <= 1.5 and hold_xy_mean_m <= 0.15:
            stability_level = "pass"
        else:
            stability_level = "fail"

    return {
        "reach_success": True,
        "reach_time_s": result["reach_time"],
        "settling_time_s": result["settling_time"],
        "within_threshold": bool(result["final_xy_error"] <= threshold_m and result["final_abs_z_error"] <= threshold_m),
        "final_xy_error_m": result["final_xy_error"],
        "final_z_error_m": result["final_abs_z_error"],
        "hold_xy_mean_m": result["xy_mean"],
        "hold_xy_rmse_m": result["xy_rmse"],
        "hold_xy_p95_m": result["hold_window_p95_xy_error"],
        "hold_z_rmse_m": result["z_rmse"],
        "speed_mean_mps": result["speed_mean"],
        "speed_p95_mps": result["speed_95"],
        "speed_max_mps": result["speed_max"],
        "path_length_m": result["path_length_m"],
        "path_efficiency": result["path_efficiency"],
        "max_lateral_deviation_m": result["max_lateral_deviation_m"],
        "overshoot_distance_m": result["overshoot_distance_m"],
        "stability_level": stability_level,
    }


def _load_hover_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    module = _load_module("sunray_hover_analyze", os.path.join(workspace_root, "hover_analyze.py"))
    log_path = module.resolve_log_path(run_dir)
    test_result = payload
    bag_path = module.resolve_bag_path(run_dir, test_result)
    log_df = module.load_event_log(log_path)

    case_id, hover_duration_s = module.infer_hover_case(test_result, log_df)
    if case_id is None:
        return None

    hover_duration_s = hover_duration_s or test_result.get("config", {}).get("defaults", {}).get(
        "hover_duration_s",
        module.DEFAULT_HOVER_DURATION_S,
    )
    target = test_result.get("config", {}).get("defaults", {}).get("takeoff_target_pos", module.DEFAULT_TARGET)
    target = [float(v) for v in target[:3]]
    hover_start_time, hover_end_time = module.find_hover_window(log_df, case_id, float(hover_duration_s))
    if hover_start_time is None:
        return None

    import rosbag

    with rosbag.Bag(bag_path) as bag:
        topic_info = bag.get_type_and_topic_info().topics
        pose_topic = module.select_pose_topic(topic_info, test_result)

    if pose_topic is None:
        return None

    result_df = module.extract_hover_data(bag_path, pose_topic, hover_start_time, hover_end_time)
    if result_df.empty:
        return None

    csv_dir = os.path.join(run_dir, "data")
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, "hover_stability_xyz.csv")
    result_df.to_csv(csv_path, index=False)
    accuracy, stability, robustness, smoothness, analysis_info = module.analyze_hover(result_df, target)
    return _normalize(
        {
            "title": "悬停指标",
            "case_id": case_id,
            "pose_topic": pose_topic,
            "target_xyz": target,
            "window": {
                "start_time_s": hover_start_time,
                "end_time_s": hover_end_time,
                "duration_s": float(hover_end_time - hover_start_time),
            },
            "artifacts": {
                "hover_csv": os.path.relpath(csv_path, run_dir),
            },
            "metrics_by_category": {
                "accuracy": accuracy,
                "stability": stability,
                "robustness": robustness,
                "smoothness": smoothness,
                "analysis_info": analysis_info,
            },
        }
    )


def _load_waypoint_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    module = _load_module("sunray_waypoint_analyze", os.path.join(workspace_root, "waypoint_analyze.py"))
    log_path = module.resolve_log_path(run_dir)
    test_result = payload
    bag_path = module.resolve_bag_path(run_dir, test_result)
    log_df = module.load_event_log(log_path)
    waypoints = module.parse_waypoint_events(log_df)
    if not waypoints:
        return None

    defaults = test_result.get("config", {}).get("defaults", {})
    threshold_m = float(defaults.get("waypoint_reach_radius_m", module.DEFAULT_THRESHOLD_M))
    stable_time_s = float(defaults.get("waypoint_stable_time_s", module.DEFAULT_STABLE_TIME_S))

    import numpy as np
    import pandas as pd
    import rosbag

    data = []
    with rosbag.Bag(bag_path) as bag:
        topic_name = module.select_pose_topic(bag.get_type_and_topic_info().topics, test_result)
        if topic_name is None:
            return None
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            position = module.get_position_from_msg(msg)
            if position is None:
                continue
            data.append([t.to_sec(), position.x, position.y, position.z])

    if not data:
        return None

    traj = pd.DataFrame(data, columns=["time", "x", "y", "z"]).sort_values("time")
    traj["dt"] = traj["time"].diff()
    traj = traj[traj["dt"] > 0.005]
    traj["dx"] = traj["x"].diff()
    traj["dy"] = traj["y"].diff()
    traj["dz"] = traj["z"].diff()
    traj["speed"] = np.sqrt(traj["dx"] ** 2 + traj["dy"] ** 2 + traj["dz"] ** 2) / traj["dt"]
    traj = traj.dropna()
    if traj.empty:
        return None

    mean_dt_s = traj["dt"].mean()
    stable_count = max(1, int(np.ceil(stable_time_s / mean_dt_s)))

    results: List[Dict[str, Any]] = []
    for index, waypoint_info in enumerate(waypoints):
        start_t = waypoint_info["time"]
        if waypoint_info["end_time"] is not None:
            segment_end_t = waypoint_info["end_time"]
        elif index < len(waypoints) - 1:
            segment_end_t = waypoints[index + 1]["time"]
        else:
            segment_end_t = traj["time"].max()

        wp = waypoint_info["wp"]
        segment = traj[(traj["time"] >= start_t) & (traj["time"] <= segment_end_t)].copy()
        if len(segment) < 20:
            continue

        settled_time = module.find_reached_time(segment, wp, threshold_m, stable_count)
        first_entry_time = module.find_first_entry_time(segment, wp, threshold_m)
        reach_success_flag = settled_time is not None

        if settled_time is None:
            result = {"waypoint": wp, "reach_success_flag": False}
        else:
            flight = segment[segment["time"] <= settled_time].copy()
            hold = segment[segment["time"] > settled_time].copy()
            flight["ex"] = flight["x"] - wp[0]
            flight["ey"] = flight["y"] - wp[1]
            flight["ez"] = flight["z"] - wp[2]
            final_xy_error = float(np.sqrt(flight["ex"].iloc[-1] ** 2 + flight["ey"].iloc[-1] ** 2))
            final_abs_z_error = float(np.abs(flight["ez"].iloc[-1]))
            speed_mean = float(flight["speed"].mean())
            speed_95 = float(np.percentile(flight["speed"], 95))
            speed_max = float(flight["speed"].max())
            path_length_m = float(np.sqrt(flight["dx"] ** 2 + flight["dy"] ** 2 + flight["dz"] ** 2).sum())
            start_pos = flight[["x", "y", "z"]].iloc[0].to_numpy(dtype=float)
            end_pos = np.array(wp, dtype=float)
            direct_distance = np.linalg.norm(end_pos - start_pos)
            path_efficiency = float(direct_distance / path_length_m) if path_length_m > 1e-9 else None
            max_lateral_deviation_m = float(
                max(
                    module.point_to_line_distance(row[["x", "y", "z"]].to_numpy(dtype=float), start_pos, end_pos)
                    for _, row in flight.iterrows()
                )
            )
            overshoot_distance_m = float(
                module.compute_overshoot_distance(
                    flight[["x", "y", "z"]].to_numpy(dtype=float),
                    start_pos,
                    end_pos,
                )
            )
            reach_time = float(first_entry_time - start_t) if first_entry_time is not None else None
            settling_time = float(settled_time - start_t)

            if len(hold) > 10:
                hold["ex"] = hold["x"] - wp[0]
                hold["ey"] = hold["y"] - wp[1]
                hold["ez"] = hold["z"] - wp[2]
                hold["xy"] = np.sqrt(hold["ex"] ** 2 + hold["ey"] ** 2)
                xy_mean = float(hold["xy"].mean())
                xy_rmse = float(module.rmse(hold["xy"]))
                hold_window_p95_xy_error = float(np.percentile(hold["xy"], 95))
                z_rmse = float(module.rmse(hold["ez"]))
            else:
                xy_mean = None
                xy_rmse = None
                hold_window_p95_xy_error = None
                z_rmse = None

            result = {
                "waypoint": wp,
                "final_xy_error": final_xy_error,
                "final_abs_z_error": final_abs_z_error,
                "speed_mean": speed_mean,
                "speed_95": speed_95,
                "speed_max": speed_max,
                "overshoot_distance_m": overshoot_distance_m,
                "path_length_m": path_length_m,
                "path_efficiency": path_efficiency,
                "max_lateral_deviation_m": max_lateral_deviation_m,
                "reach_success_flag": reach_success_flag,
                "reach_time": reach_time,
                "settling_time": settling_time,
                "xy_mean": xy_mean,
                "xy_rmse": xy_rmse,
                "hold_window_p95_xy_error": hold_window_p95_xy_error,
                "z_rmse": z_rmse,
            }

        results.append(
            {
                "waypoint": wp,
                "metrics": _build_waypoint_metric_view(result, threshold_m),
            }
        )

    if not results:
        return None

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
            "title": "航点飞行指标",
            "pose_topic": topic_name,
            "threshold_m": threshold_m,
            "stable_time_s": stable_time_s,
            "summary": summary,
            "waypoints": results,
        }
    )


def _load_landing_metrics(workspace_root: str, run_dir: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    module = _load_module("sunray_landing_analyze", os.path.join(workspace_root, "landing_analyze.py"))
    log_path = module.resolve_log_path(run_dir)
    if log_path is None:
        return None

    bag_path = module.find_file_with_suffix(run_dir, ".bag")
    if bag_path is None:
        return None

    log_df = module.load_log_csv(log_path)
    test_result = payload
    landing_window = module.get_visual_landing_window(log_df, test_result)

    import rosbag

    with rosbag.Bag(bag_path) as bag:
        pose_topic = module.choose_pose_topic(bag, test_result)
        detection_topic = module.choose_detection_topic(bag)

    pose_df = module.load_pose_df(
        bag_path,
        pose_topic,
        landing_window["start_time"],
        landing_window["end_time"],
    )
    detection_df = module.load_detection_df(
        bag_path,
        detection_topic,
        landing_window["start_time"],
        landing_window["end_time"],
    )

    landing_target_xy, landing_target_xy_source, landing_target_xy_sample_count = (
        module.estimate_landing_target_xy_from_pose_and_detection(
            pose_df,
            detection_df,
            module.DEFAULT_POSE_DETECTION_SYNC_TOLERANCE_S,
        )
    )
    if landing_target_xy is None:
        landing_target_xy, landing_target_xy_source = module.infer_landing_target_xy(
            log_df,
            test_result,
            landing_window["start_time"],
        )
        landing_target_xy_sample_count = 0

    pose_metrics = module.analyze_pose_metrics(
        pose_df,
        landing_target_xy,
        module.DEFAULT_TARGET_ZONE_RADIUS_M,
        module.DEFAULT_TOUCHDOWN_WINDOW_S,
    )
    detection_metrics = module.analyze_detection_metrics(detection_df, landing_window, pose_df, pose_metrics)
    functional_metrics = module.infer_functional_metrics(landing_window, pose_metrics, detection_metrics)

    csv_dir = os.path.join(run_dir, "data")
    os.makedirs(csv_dir, exist_ok=True)
    pose_csv = os.path.join(csv_dir, "visual_landing_pose.csv")
    pose_df.to_csv(pose_csv, index=False)
    detection_csv = None
    if not detection_df.empty:
        detection_csv = os.path.join(csv_dir, "visual_landing_detection.csv")
        detection_df.to_csv(detection_csv, index=False, na_rep="NaN")

    analysis = {
        "title": "视觉降落指标",
        "pose_topic": pose_topic,
        "detection_topic": detection_topic,
        "landing_target_xy": landing_target_xy,
        "landing_target_xy_source": landing_target_xy_source,
        "landing_target_xy_sample_count": landing_target_xy_sample_count,
        "target_zone_radius_m": module.DEFAULT_TARGET_ZONE_RADIUS_M,
        "touchdown_window_s": module.DEFAULT_TOUCHDOWN_WINDOW_S,
        "window": {
            "start_time_s": landing_window["start_time"],
            "end_time_s": landing_window["end_time"],
            "duration_s": landing_window["duration_s"],
        },
        "artifacts": {
            "pose_window_csv": os.path.relpath(pose_csv, run_dir),
            "detection_window_csv": os.path.relpath(detection_csv, run_dir) if detection_csv else None,
        },
        "limitations": [],
        "metrics": {},
        "metrics_by_category": {},
    }
    if detection_topic is None:
        analysis["limitations"].append(
            "No detection topic recorded in rosbag. Detection-side visual metrics are unavailable."
        )
    analysis["metrics"].update(functional_metrics)
    analysis["metrics"].update(detection_metrics)
    analysis["metrics"].update(pose_metrics)
    analysis["metrics_by_category"] = module.build_ordered_metric_groups(analysis["metrics"])
    return _normalize(analysis)


def enrich_report_payload(payload: Dict[str, Any], workspace_root: Optional[str] = None) -> Dict[str, Any]:
    run_dir = payload.get("run_info", {}).get("run_dir") or payload.get("artifacts", {}).get("run_dir")
    _remove_skip_data(payload)
    if not run_dir or not os.path.isdir(run_dir):
        return payload

    if workspace_root is None:
        candidate = os.path.abspath(run_dir)
        while True:
            if os.path.exists(os.path.join(candidate, "hover_analyze.py")):
                workspace_root = candidate
                break
            parent = os.path.dirname(candidate)
            if parent == candidate:
                workspace_root = os.path.abspath(os.path.join(run_dir, "..", "..", ".."))
                break
            candidate = parent

    if not os.path.exists(os.path.join(workspace_root, "hover_analyze.py")):
        return payload

    event_log_path = payload.get("artifacts", {}).get("event_log_jsonl", "")
    if event_log_path and not os.path.isabs(event_log_path):
        event_log_path = os.path.join(run_dir, event_log_path)
    payload["event_log"] = _load_event_log(event_log_path)
    _backfill_case_times(payload, payload["event_log"])

    sections = []
    errors = []
    for loader in (_load_hover_metrics, _load_waypoint_metrics, _load_landing_metrics):
        try:
            result = loader(workspace_root, run_dir, payload)
        except Exception as exc:
            errors.append(str(exc))
            result = None
        if result:
            sections.append(result)

    payload["flight_metrics"] = {
        "title": "飞行指标",
        "sections": sections,
        "errors": errors,
    }

    scoring_config = load_scoring_config(workspace_root)
    if scoring_config and sections:
        compute_scores(payload, scoring_config)

    return payload

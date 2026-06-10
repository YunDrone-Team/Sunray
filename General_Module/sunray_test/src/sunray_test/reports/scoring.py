import math
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml


SECTION_KEY_MAP = {
    "悬停指标": "hover",
    "航点飞行指标": "waypoint",
    "EGO自主规划指标": "ego_goal",
    "视觉降落指标": "visual_landing",
}

CASE_ID_TO_SECTION = {
    "hover_stability": "hover",
    "hover": "hover",
    "waypoint_flight": "waypoint",
    "waypoint": "waypoint",
    "ego_goal_flight": "ego_goal",
    "ego_goal": "ego_goal",
    "visual_landing": "visual_landing",
}

CASE_NAME_TO_SECTION = {
    "悬停": "hover",
    "指点飞行": "waypoint",
    "航点飞行": "waypoint",
    "EGO-Planner自主规划": "ego_goal",
    "EGO自主规划": "ego_goal",
    "视觉降落": "visual_landing",
}


def _case_section_key(case: Dict[str, Any]) -> Optional[str]:
    case_id = str(case.get("id") or case.get("case_id") or "").strip()
    if case_id:
        for prefix, key in CASE_ID_TO_SECTION.items():
            if case_id == prefix or case_id.startswith(prefix):
                return key

    case_name = str(case.get("name", "")).strip()
    if case_name in CASE_NAME_TO_SECTION:
        return CASE_NAME_TO_SECTION[case_name]
    case_type = str(case.get("type", "")).strip()
    if case_type.startswith("flight."):
        return CASE_ID_TO_SECTION.get(case_type.split(".", 1)[1])
    return None


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


def load_scoring_config(workspace_root: str) -> Optional[Dict[str, Any]]:
    config_path = None
    candidate = workspace_root
    while True:
        path = os.path.join(candidate, "General_Module", "sunray_test", "config", "scoring", "scoring.yaml")
        if os.path.isfile(path):
            config_path = path
            break
        path = os.path.join(candidate, "config", "scoring", "scoring.yaml")
        if os.path.isfile(path):
            config_path = path
            break
        parent = os.path.dirname(candidate)
        if parent == candidate:
            break
        candidate = parent
    if config_path is None:
        return None
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or None


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _resolve_platform_scoring_config(payload: Dict[str, Any], scoring_config: Dict[str, Any]) -> Dict[str, Any]:
    platform_name = str(payload.get("run_info", {}).get("platform", "")).strip()
    platform_profiles = scoring_config.get("platform_profiles") or {}
    profile = platform_profiles.get(platform_name)
    if not platform_name:
        raise ValueError("missing run_info.platform for scoring profile")
    if not isinstance(profile, dict):
        available = ", ".join(sorted(str(key) for key in platform_profiles.keys())) or "<none>"
        raise ValueError(f"missing scoring platform profile: {platform_name}; available: {available}")

    effective_config = _deep_merge(
        {"grades": scoring_config.get("grades", [])},
        profile,
    )
    payload.setdefault("flight_metrics", {})["scoring_profile"] = platform_name
    return effective_config


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _score_single_metric(value: Any, thresholds: List[float], higher_is_better: bool = False) -> Optional[float]:
    if value is None or not isinstance(value, (int, float, bool)):
        return None
    v = 1.0 if value is True else 0.0 if value is False else float(value)
    t100, t75, t60, t0 = [float(t) for t in thresholds]
    if higher_is_better:
        if v >= t100:
            return 100.0
        if v >= t75:
            return _lerp(75.0, 100.0, (v - t75) / (t100 - t75)) if t100 != t75 else 75.0
        if v >= t60:
            return _lerp(60.0, 75.0, (v - t60) / (t75 - t60)) if t75 != t60 else 60.0
        if v > t0:
            return _lerp(0.0, 60.0, (v - t0) / (t60 - t0)) if t60 != t0 else 0.0
        return 0.0

    if v <= t100:
        return 100.0
    if v <= t75:
        return _lerp(100.0, 75.0, (v - t100) / (t75 - t100)) if t75 != t100 else 75.0
    if v <= t60:
        return _lerp(75.0, 60.0, (v - t75) / (t60 - t75)) if t60 != t75 else 60.0
    if v < t0:
        return _lerp(60.0, 0.0, (v - t60) / (t0 - t60)) if t0 != t60 else 0.0
    return 0.0


def _collect_flat_metrics(section: Dict[str, Any]) -> Dict[str, Any]:
    flat: Dict[str, Any] = {}
    summary_by_category = section.get("summary_by_category")
    if isinstance(summary_by_category, dict):
        for category_metrics in summary_by_category.values():
            if isinstance(category_metrics, dict):
                flat.update(category_metrics)
    metrics_by_category = section.get("metrics_by_category")
    if isinstance(metrics_by_category, dict):
        for category_metrics in metrics_by_category.values():
            if isinstance(category_metrics, dict):
                flat.update(category_metrics)
    metrics = section.get("metrics")
    if isinstance(metrics, dict):
        flat.update(metrics)
    summary = section.get("summary")
    if isinstance(summary, dict):
        flat.update(summary)
    return flat


def _failed_gates(flat_metrics: Dict[str, Any], gates: List[str]) -> List[str]:
    failed = []
    for gate_key in gates:
        value = flat_metrics.get(gate_key)
        if value is None or value is False:
            failed.append(gate_key)
        elif isinstance(value, (int, float)) and not isinstance(value, bool) and float(value) <= 0:
            failed.append(gate_key)
    return failed


def _score_configured_metric(metric_key: str, value: Any, cfg: Dict[str, Any]) -> Tuple[float, float, Dict[str, Any]]:
    thresholds = cfg.get("thresholds", [])
    if len(thresholds) != 4:
        raise ValueError(f"invalid scoring thresholds for {metric_key}: expected four values")

    weight = float(cfg.get("weight", 0))
    score = _score_single_metric(value, thresholds, cfg.get("higher_is_better", False))
    if score is None:
        detail = {"value": value, "score": 0.0}
        detail["reason"] = "missing_metric" if value is None else "invalid_metric_value"
        return 0.0, weight, detail
    return score, weight, {"value": value, "score": round(score, 1)}


def _score_metric_set(flat_metrics: Dict[str, Any], metric_configs: Dict[str, Any]) -> Tuple[Optional[float], Dict[str, Any]]:
    weighted_sum = 0.0
    total_weight = 0.0
    details: Dict[str, Any] = {}
    for metric_key, cfg in metric_configs.items():
        score, weight, detail = _score_configured_metric(metric_key, flat_metrics.get(metric_key), cfg)
        details[metric_key] = detail
        if weight > 0:
            weighted_sum += score * weight
            total_weight += weight

    section_score = round(weighted_sum / total_weight, 1) if total_weight > 0 else None
    return section_score, details


def _score_section(section: Dict[str, Any], section_config: Dict[str, Any]) -> Dict[str, Any]:
    flat = _collect_flat_metrics(section)
    gates = section_config.get("gates", [])
    metric_configs = section_config.get("metrics", {})
    details: Dict[str, Any] = {}

    failed_gates = _failed_gates(flat, gates)
    if failed_gates:
        for metric_key in metric_configs:
            details[metric_key] = {"value": flat.get(metric_key), "score": 0.0, "reason": "gate_failed"}
        return {"score": 0.0, "gate_failed": True, "failed_gates": failed_gates, "details": details}

    section_score, details = _score_metric_set(flat, metric_configs)
    return {"score": section_score, "gate_failed": False, "details": details}


def _score_waypoint_section(section: Dict[str, Any], section_config: Dict[str, Any]) -> Dict[str, Any]:
    waypoints = section.get("waypoints", [])
    if not waypoints:
        return _score_section(section, section_config)

    flat = _collect_flat_metrics(section)
    gates = section_config.get("gates", [])
    section_gates = [gate for gate in gates if gate in flat]
    waypoint_gates = [gate for gate in gates if gate not in flat]
    metric_configs = section_config.get("metrics", {})
    section_metric_keys = {key for key in metric_configs if key in flat}
    waypoint_metric_configs = {
        key: value
        for key, value in metric_configs.items()
        if key not in section_metric_keys
    }
    waypoint_scores: List[Dict[str, Any]] = []

    failed_section_gates = _failed_gates(flat, section_gates)
    if failed_section_gates:
        details = {
            metric_key: {"value": flat.get(metric_key), "score": 0.0, "reason": "gate_failed"}
            for metric_key in metric_configs
        }
        return {
            "score": 0.0,
            "gate_failed": True,
            "failed_gates": failed_section_gates,
            "details": details,
            "waypoint_scores": [],
        }

    for waypoint_data in waypoints:
        waypoint_metrics = waypoint_data.get("metrics", {})
        failed_waypoint_gates = _failed_gates(waypoint_metrics, waypoint_gates)
        if failed_waypoint_gates:
            waypoint_scores.append(
                {
                    "score": 0.0,
                    "gate_failed": True,
                    "failed_gates": failed_waypoint_gates,
                    "waypoint": waypoint_data.get("waypoint"),
                }
            )
            continue

        waypoint_score, details = _score_metric_set(waypoint_metrics, waypoint_metric_configs)
        waypoint_scores.append(
            {
                "score": waypoint_score,
                "gate_failed": False,
                "waypoint": waypoint_data.get("waypoint"),
                "details": details,
            }
        )

    valid_scores = [item["score"] for item in waypoint_scores if item["score"] is not None]
    waypoint_average_score = round(sum(valid_scores) / len(valid_scores), 1) if valid_scores else None
    summary_config = {
        **section_config,
        "gates": section_gates,
        "metrics": {
            key: value
            for key, value in metric_configs.items()
            if key in section_metric_keys
        },
    }
    summary_details = _score_section(section, summary_config)
    summary_score = summary_details.get("score")
    if summary_score is not None and waypoint_average_score is not None:
        average_score = round((float(summary_score) + float(waypoint_average_score)) / 2.0, 1)
    elif summary_score is not None:
        average_score = summary_score
    else:
        average_score = waypoint_average_score
    return {
        "score": average_score,
        "gate_failed": average_score == 0.0,
        "details": summary_details.get("details", {}),
        "waypoint_scores": waypoint_scores,
    }


def _grade_for_score(score: Optional[float], grade_config: List[Dict[str, Any]]) -> Tuple[str, str]:
    if score is None:
        return "-", "#69758a"
    sorted_grades = sorted(grade_config, key=lambda g: g.get("min", 0), reverse=True)
    for grade in sorted_grades:
        if score >= grade.get("min", 0):
            return grade.get("label", "-"), grade.get("color", "#69758a")
    last = sorted_grades[-1] if sorted_grades else {}
    return last.get("label", "-"), last.get("color", "#69758a")


def compute_scores(payload: Dict[str, Any], scoring_config: Dict[str, Any]) -> None:
    scoring_config = _resolve_platform_scoring_config(payload, scoring_config)
    flight_metrics = payload.get("flight_metrics", {})
    sections = flight_metrics.get("sections", [])
    grade_config = scoring_config.get("grades", [])
    cases = payload.get("cases", [])

    scores: Dict[str, Any] = {"grade_thresholds": grade_config}
    section_results: List[Tuple[str, float, float]] = []
    case_result_by_section: Dict[str, str] = {}

    for case in cases:
        result = str(case.get("result", "")).strip().lower()
        config_key = _case_section_key(case)
        if config_key:
            case_result_by_section[config_key] = result

    planned_scored_sections: List[str] = []
    for key in ("hover", "waypoint", "ego_goal", "visual_landing"):
        if key in case_result_by_section and key in scoring_config:
            planned_scored_sections.append(key)

    for section in sections:
        title = section.get("title", "")
        config_key = SECTION_KEY_MAP.get(title)
        if not config_key or config_key not in scoring_config:
            continue

        section_config = scoring_config[config_key]
        result = _score_waypoint_section(section, section_config) if config_key in {"waypoint", "ego_goal"} else _score_section(section, section_config)

        case_result = case_result_by_section.get(config_key)
        if case_result and case_result != "pass":
            result["score"] = 0.0
            result["gate_failed"] = True
            result["forced_by_case_result"] = case_result

        label, color = _grade_for_score(result["score"], grade_config)
        result["grade"] = label
        result["grade_color"] = color
        scores[config_key] = result

    for config_key in planned_scored_sections:
        section_config = scoring_config[config_key]
        score_entry = scores.get(config_key)
        if not isinstance(score_entry, dict):
            case_result = case_result_by_section.get(config_key, "")
            score_entry = {
                "score": 0.0,
                "gate_failed": True,
                "details": {},
                "reason": "missing_metrics_section",
            }
            if case_result and case_result != "pass":
                score_entry["forced_by_case_result"] = case_result
            label, color = _grade_for_score(score_entry["score"], grade_config)
            score_entry["grade"] = label
            score_entry["grade_color"] = color
            scores[config_key] = score_entry
        section_results.append(
            (
                config_key,
                float(score_entry.get("score", 0.0) or 0.0),
                float(section_config.get("weight", 0)),
            )
        )

    if section_results:
        total_weight = sum(weight for _, _, weight in section_results)
        overall_score = round(sum(score * weight for _, score, weight in section_results) / total_weight, 1) if total_weight > 0 else None
    else:
        overall_score = None

    label, color = _grade_for_score(overall_score, grade_config)
    scores["overall"] = {"score": overall_score, "grade": label, "grade_color": color}
    flight_metrics["scores"] = _normalize(scores)

    for case in payload.get("cases", []):
        if case.get("category") == "hardware":
            case.pop("score", None)
            continue
        config_key = _case_section_key(case)
        if config_key and config_key in scores and isinstance(scores[config_key], dict):
            if str(case.get("result", "")).strip().lower() == "pass":
                case["score"] = scores[config_key].get("score")
            elif case.get("result") in ("fail", "error"):
                case["score"] = 0
        elif config_key and config_key in scoring_config:
            case.pop("score", None)
        elif case.get("result") == "pass":
            case["score"] = 100
        elif case.get("result") in ("fail", "error"):
            case["score"] = 0

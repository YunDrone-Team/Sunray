import math
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml


SECTION_KEY_MAP = {
    "悬停指标": "hover",
    "航点飞行指标": "waypoint",
    "视觉降落指标": "visual_landing",
}

CASE_ID_TO_SECTION = {
    "hover_stability": "hover",
    "hover": "hover",
    "waypoint_flight": "waypoint",
    "waypoint": "waypoint",
    "visual_landing": "visual_landing",
}


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


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _score_single_metric(value: Any, thresholds: List[float], higher_is_better: bool = False) -> Optional[float]:
    if value is None or not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    v = float(value)
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


def _check_gates(flat_metrics: Dict[str, Any], gates: List[str]) -> bool:
    for gate_key in gates:
        val = flat_metrics.get(gate_key)
        if val is None or val is False:
            return False
    return True


def _score_section(section: Dict[str, Any], section_config: Dict[str, Any]) -> Dict[str, Any]:
    flat = _collect_flat_metrics(section)
    gates = section_config.get("gates", [])
    metric_configs = section_config.get("metrics", {})
    details: Dict[str, Any] = {}

    if gates and not _check_gates(flat, gates):
        for metric_key in metric_configs:
            details[metric_key] = {"value": flat.get(metric_key), "score": 0.0}
        return {"score": 0.0, "gate_failed": True, "details": details}

    weighted_sum = 0.0
    total_weight = 0.0
    for metric_key, cfg in metric_configs.items():
        value = flat.get(metric_key)
        thresholds = cfg.get("thresholds", [])
        higher_is_better = cfg.get("higher_is_better", False)
        weight = float(cfg.get("weight", 0))
        score = _score_single_metric(value, thresholds, higher_is_better)
        if score is not None:
            weighted_sum += score * weight
            total_weight += weight
            details[metric_key] = {"value": value, "score": round(score, 1)}
        else:
            details[metric_key] = {"value": value, "score": None}

    section_score = round(weighted_sum / total_weight, 1) if total_weight > 0 else None
    return {"score": section_score, "gate_failed": False, "details": details}


def _score_waypoint_section(section: Dict[str, Any], section_config: Dict[str, Any]) -> Dict[str, Any]:
    waypoints = section.get("waypoints", [])
    if not waypoints:
        return _score_section(section, section_config)

    gates = section_config.get("gates", [])
    metric_configs = section_config.get("metrics", {})
    waypoint_scores: List[Dict[str, Any]] = []

    for waypoint_data in waypoints:
        waypoint_metrics = waypoint_data.get("metrics", {})
        if gates and not _check_gates(waypoint_metrics, gates):
            waypoint_scores.append({"score": 0.0, "gate_failed": True, "waypoint": waypoint_data.get("waypoint")})
            continue

        weighted_sum = 0.0
        total_weight = 0.0
        details: Dict[str, Any] = {}
        for metric_key, cfg in metric_configs.items():
            value = waypoint_metrics.get(metric_key)
            thresholds = cfg.get("thresholds", [])
            higher_is_better = cfg.get("higher_is_better", False)
            weight = float(cfg.get("weight", 0))
            score = _score_single_metric(value, thresholds, higher_is_better)
            if score is not None:
                weighted_sum += score * weight
                total_weight += weight
                details[metric_key] = {"value": value, "score": round(score, 1)}
            else:
                details[metric_key] = {"value": value, "score": None}

        waypoint_score = round(weighted_sum / total_weight, 1) if total_weight > 0 else None
        waypoint_scores.append(
            {
                "score": waypoint_score,
                "gate_failed": False,
                "waypoint": waypoint_data.get("waypoint"),
                "details": details,
            }
        )

    valid_scores = [item["score"] for item in waypoint_scores if item["score"] is not None]
    average_score = round(sum(valid_scores) / len(valid_scores), 1) if valid_scores else None
    summary_details = _score_section(section, section_config)
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
    flight_metrics = payload.get("flight_metrics", {})
    sections = flight_metrics.get("sections", [])
    grade_config = scoring_config.get("grades", [])
    cases = payload.get("cases", [])

    scores: Dict[str, Any] = {"grade_thresholds": grade_config}
    section_results: List[Tuple[str, float, float]] = []
    case_result_by_section: Dict[str, str] = {}

    for case in cases:
        case_id = str(case.get("id", ""))
        result = str(case.get("result", "")).strip().lower()
        for prefix, key in CASE_ID_TO_SECTION.items():
            if case_id == prefix or case_id.startswith(prefix):
                case_result_by_section[key] = result
                break

    planned_scored_sections: List[str] = []
    for key in ("hover", "waypoint", "visual_landing"):
        if key in case_result_by_section and key in scoring_config:
            planned_scored_sections.append(key)

    for section in sections:
        title = section.get("title", "")
        config_key = SECTION_KEY_MAP.get(title)
        if not config_key or config_key not in scoring_config:
            continue

        section_config = scoring_config[config_key]
        result = _score_waypoint_section(section, section_config) if config_key == "waypoint" else _score_section(section, section_config)

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
            score_entry = {
                "score": 0.0,
                "gate_failed": True,
                "details": {},
                "forced_by_case_result": case_result_by_section.get(config_key, "fail"),
            }
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
        case_id = str(case.get("id", ""))
        config_key = None
        for prefix, key in CASE_ID_TO_SECTION.items():
            if case_id == prefix or case_id.startswith(prefix):
                config_key = key
                break
        if config_key and config_key in scores and isinstance(scores[config_key], dict):
            if str(case.get("result", "")).strip().lower() == "pass":
                case["score"] = scores[config_key].get("score")
            elif case.get("result") in ("fail", "error"):
                case["score"] = 0
        elif case.get("result") == "pass":
            case["score"] = 100
        elif case.get("result") in ("fail", "error"):
            case["score"] = 0

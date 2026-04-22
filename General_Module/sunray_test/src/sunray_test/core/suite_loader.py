import copy
import os
from typing import Any, Dict, Iterable, List, Optional

import yaml


class ConfigValidationError(ValueError):
    pass


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data


def load_yaml_dir(directory: str) -> Dict[str, Dict[str, Any]]:
    loaded: Dict[str, Dict[str, Any]] = {}
    if not os.path.isdir(directory):
        return loaded
    for filename in sorted(os.listdir(directory)):
        if not filename.endswith(".yaml"):
            continue
        path = os.path.join(directory, filename)
        loaded[os.path.splitext(filename)[0]] = load_yaml(path)
    return loaded


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def render_template(value: Any, variables: Dict[str, Any]) -> Any:
    if isinstance(value, str):
        return value.format(**variables)
    if isinstance(value, list):
        return [render_template(item, variables) for item in value]
    if isinstance(value, dict):
        return {key: render_template(item, variables) for key, item in value.items()}
    return value


def merge_unique_lists(*lists: Iterable[Any]) -> List[Any]:
    merged: List[Any] = []
    for current in lists:
        for item in current:
            if item not in merged:
                merged.append(item)
    return merged


def _ensure(condition: bool, message: str) -> None:
    if not condition:
        raise ConfigValidationError(message)


def _ensure_dict(value: Any, path: str) -> Dict[str, Any]:
    _ensure(isinstance(value, dict), f"{path} must be a mapping")
    return value


def _ensure_list(value: Any, path: str) -> List[Any]:
    _ensure(isinstance(value, list), f"{path} must be a list")
    return value


def _ensure_string(value: Any, path: str, allow_empty: bool = False) -> str:
    _ensure(isinstance(value, str), f"{path} must be a string")
    if not allow_empty:
        _ensure(value.strip() != "", f"{path} must not be empty")
    return value


def _ensure_bool(value: Any, path: str) -> bool:
    _ensure(isinstance(value, bool), f"{path} must be a boolean")
    return value


def _ensure_number(value: Any, path: str) -> float:
    _ensure(isinstance(value, (int, float)) and not isinstance(value, bool), f"{path} must be a number")
    return float(value)


def _ensure_allowed_keys(data: Dict[str, Any], path: str, allowed: Iterable[str], required: Iterable[str] = ()) -> None:
    allowed_set = set(allowed)
    required_set = set(required)
    unknown_keys = sorted(set(data.keys()) - allowed_set)
    missing_keys = sorted(required_set - set(data.keys()))
    _ensure(not unknown_keys, f"{path} contains unknown keys: {', '.join(unknown_keys)}")
    _ensure(not missing_keys, f"{path} is missing required keys: {', '.join(missing_keys)}")


def _ensure_point_list(value: Any, path: str) -> None:
    points = _ensure_list(value, path)
    for index, point in enumerate(points):
        point_path = f"{path}[{index}]"
        _ensure(isinstance(point, (list, tuple)), f"{point_path} must be a list or tuple")
        _ensure(len(point) >= 3, f"{point_path} must contain at least 3 numbers")
        for axis_index, axis_value in enumerate(point[:3]):
            _ensure_number(axis_value, f"{point_path}[{axis_index}]")


def _validate_report_config(report: Any, path: str) -> None:
    if report is None:
        return
    report_dict = _ensure_dict(report, path)
    _ensure_allowed_keys(report_dict, path, {"title"})
    if "title" in report_dict:
        _ensure_string(report_dict["title"], f"{path}.title")


def _validate_recording_config(recording: Any, path: str) -> None:
    recording_dict = _ensure_dict(recording, path)
    _ensure_allowed_keys(recording_dict, path, {"bag_prefix", "topic_templates"})
    if "bag_prefix" in recording_dict:
        _ensure_string(recording_dict["bag_prefix"], f"{path}.bag_prefix")
    if "topic_templates" in recording_dict:
        for index, topic in enumerate(_ensure_list(recording_dict["topic_templates"], f"{path}.topic_templates")):
            _ensure_string(topic, f"{path}.topic_templates[{index}]")


def _validate_defaults(defaults: Any, path: str) -> None:
    defaults_dict = _ensure_dict(defaults, path)
    numeric_keys = {
        "hardware_check_timeout_s",
        "battery_pass_threshold_v",
        "post_takeoff_settle_time_s",
        "hover_duration_s",
        "waypoint_reach_radius_m",
        "waypoint_stable_time_s",
        "waypoint_hold_time_s",
        "waypoint_timeout_s",
        "visual_landing_height_m",
    }
    for key in numeric_keys:
        if key in defaults_dict:
            _ensure_number(defaults_dict[key], f"{path}.{key}")
    if "takeoff_target_pos" in defaults_dict:
        point = defaults_dict["takeoff_target_pos"]
        _ensure(isinstance(point, (list, tuple)), f"{path}.takeoff_target_pos must be a list or tuple")
        _ensure(len(point) >= 3, f"{path}.takeoff_target_pos must contain at least 3 numbers")
        for index, value in enumerate(point[:3]):
            _ensure_number(value, f"{path}.takeoff_target_pos[{index}]")
    if "waypoint_source" in defaults_dict:
        source = _ensure_string(defaults_dict["waypoint_source"], f"{path}.waypoint_source")
        _ensure(source in {"list", "input"}, f"{path}.waypoint_source must be 'list' or 'input'")
    if "visual_landing_auto_takeoff" in defaults_dict:
        _ensure_bool(defaults_dict["visual_landing_auto_takeoff"], f"{path}.visual_landing_auto_takeoff")


def _validate_topics(topics: Any, path: str, allow_empty_values: bool) -> None:
    topics_dict = _ensure_dict(topics, path)
    for key, value in topics_dict.items():
        _ensure_string(key, f"{path}.{key}")
        _ensure_string(value, f"{path}.{key}", allow_empty=allow_empty_values)


def _validate_capabilities(capabilities: Any, path: str) -> None:
    capabilities_dict = _ensure_dict(capabilities, path)
    for key, value in capabilities_dict.items():
        _ensure_string(key, f"{path}.{key}")
        _ensure_bool(value, f"{path}.{key}")


def _validate_platform_config(platform: Dict[str, Any], path: str) -> None:
    _ensure_allowed_keys(
        platform,
        path,
        {"name", "vehicle_type", "report", "capabilities", "topics", "recording", "defaults"},
        {"name", "vehicle_type", "topics", "recording", "defaults"},
    )
    _ensure_string(platform["name"], f"{path}.name")
    _ensure_string(platform["vehicle_type"], f"{path}.vehicle_type")
    _validate_topics(platform["topics"], f"{path}.topics", allow_empty_values=True)
    _validate_recording_config(platform["recording"], f"{path}.recording")
    _validate_defaults(platform["defaults"], f"{path}.defaults")
    if "report" in platform:
        _validate_report_config(platform["report"], f"{path}.report")
    if "capabilities" in platform:
        _validate_capabilities(platform["capabilities"], f"{path}.capabilities")


def _validate_environment_config(environment: Dict[str, Any], path: str) -> None:
    _ensure_allowed_keys(
        environment,
        path,
        {"name", "recording", "defaults", "topic_overrides", "missions"},
        {"name"},
    )
    _ensure_string(environment["name"], f"{path}.name")
    if "recording" in environment:
        _validate_recording_config(environment["recording"], f"{path}.recording")
    if "defaults" in environment:
        _validate_defaults(environment["defaults"], f"{path}.defaults")
    if "topic_overrides" in environment:
        _validate_topics(environment["topic_overrides"], f"{path}.topic_overrides", allow_empty_values=False)
    if "missions" in environment:
        _ensure_dict(environment["missions"], f"{path}.missions")


def _validate_missions(missions: Dict[str, Any], path: str) -> None:
    for mission_key, mission_value in missions.items():
        mission_path = f"{path}.{mission_key}"
        if isinstance(mission_value, dict):
            if "name" in mission_value:
                _ensure_string(mission_value["name"], f"{mission_path}.name")
            if "waypoints" in mission_value:
                _ensure_point_list(mission_value["waypoints"], f"{mission_path}.waypoints")
        elif isinstance(mission_value, list):
            _ensure_point_list(mission_value, mission_path)
        else:
            raise ConfigValidationError(f"{mission_path} must be a mapping or waypoint list")


def _validate_suite_step(
    step: Dict[str, Any],
    path: str,
    merged_topics: Dict[str, Any],
    merged_missions: Dict[str, Any],
    merged_defaults: Dict[str, Any],
) -> None:
    has_phase = "phase" in step
    has_case = "case" in step
    _ensure(has_phase != has_case, f"{path} must define exactly one of 'phase' or 'case'")

    if has_phase:
        _ensure_allowed_keys(step, path, {"phase"})
        phase_name = _ensure_string(step["phase"], f"{path}.phase")
        from sunray_test.phases.registry import PHASE_REGISTRY

        _ensure(phase_name in PHASE_REGISTRY, f"{path}.phase references unsupported phase: {phase_name}")
        return

    _ensure_allowed_keys(
        step,
        path,
        {"case", "name", "type", "params", "category", "required_state", "resulting_state"},
        {"case", "type"},
    )
    _ensure_string(step["case"], f"{path}.case")
    case_type = _ensure_string(step["type"], f"{path}.type")
    if "name" in step:
        _ensure_string(step["name"], f"{path}.name")
    if "category" in step:
        _ensure_string(step["category"], f"{path}.category")
    if "required_state" in step:
        _ensure_string(step["required_state"], f"{path}.required_state")
    if "resulting_state" in step:
        _ensure_string(step["resulting_state"], f"{path}.resulting_state")

    from sunray_test.cases.registry import CASE_REGISTRY

    _ensure(case_type in CASE_REGISTRY, f"{path}.type references unsupported case type: {case_type}")

    params = step.get("params", {})
    _ensure(isinstance(params, dict), f"{path}.params must be a mapping")

    if "topic_key" in params:
        topic_key = _ensure_string(params["topic_key"], f"{path}.params.topic_key")
        _ensure(topic_key in merged_topics, f"{path}.params.topic_key references unknown topic key: {topic_key}")
        _ensure_string(merged_topics[topic_key], f"merged.topics.{topic_key}", allow_empty=False)

    if case_type == "hardware.camera_alive":
        _ensure("topic_key" in params, f"{path}.params.topic_key is required for hardware.camera_alive")
        if "timeout_s" in params:
            _ensure_number(params["timeout_s"], f"{path}.params.timeout_s")
        if "require_non_uniform_frame" in params:
            _ensure_bool(params["require_non_uniform_frame"], f"{path}.params.require_non_uniform_frame")
        if "device_path" in params:
            _ensure_string(params["device_path"], f"{path}.params.device_path")
    elif case_type == "hardware.battery_voltage":
        _ensure("topic_key" in params, f"{path}.params.topic_key is required for hardware.battery_voltage")
        if "timeout_s" in params:
            _ensure_number(params["timeout_s"], f"{path}.params.timeout_s")
        if "pass_threshold_v" in params:
            _ensure_number(params["pass_threshold_v"], f"{path}.params.pass_threshold_v")
    elif case_type == "flight.hover":
        if "duration_s" in params:
            _ensure_number(params["duration_s"], f"{path}.params.duration_s")
    elif case_type == "flight.visual_landing":
        if "launch_file" in params:
            _ensure_string(params["launch_file"], f"{path}.params.launch_file")
        if "auto_takeoff" in params:
            _ensure_bool(params["auto_takeoff"], f"{path}.params.auto_takeoff")
        if "height_m" in params:
            _ensure_number(params["height_m"], f"{path}.params.height_m")
    elif case_type == "flight.waypoint":
        waypoint_source = params.get("waypoint_source", merged_defaults.get("waypoint_source", "list"))
        waypoint_source = _ensure_string(waypoint_source, f"{path}.params.waypoint_source")
        _ensure(waypoint_source in {"list", "input"}, f"{path}.params.waypoint_source must be 'list' or 'input'")
        if waypoint_source == "list":
            mission_key = params.get("mission_key")
            _ensure(mission_key is not None, f"{path}.params.mission_key is required when waypoint_source=list")
            mission_key = _ensure_string(mission_key, f"{path}.params.mission_key")
            _ensure(mission_key in merged_missions, f"{path}.params.mission_key references unknown mission: {mission_key}")
        for number_key in ("reach_radius_m", "stable_time_s", "hold_time_s", "timeout_s"):
            if number_key in params:
                _ensure_number(params[number_key], f"{path}.params.{number_key}")


def _validate_suite_config(
    suite: Dict[str, Any],
    path: str,
    merged_topics: Dict[str, Any],
    merged_missions: Dict[str, Any],
    merged_defaults: Dict[str, Any],
) -> None:
    _ensure_allowed_keys(
        suite,
        path,
        {"name", "description", "record_rosbag", "stop_on_failure", "report", "steps"},
        {"name", "steps"},
    )
    _ensure_string(suite["name"], f"{path}.name")
    if "description" in suite:
        _ensure_string(suite["description"], f"{path}.description")
    if "record_rosbag" in suite:
        _ensure_bool(suite["record_rosbag"], f"{path}.record_rosbag")
    if "stop_on_failure" in suite:
        _ensure_bool(suite["stop_on_failure"], f"{path}.stop_on_failure")
    if "report" in suite:
        _validate_report_config(suite["report"], f"{path}.report")

    steps = _ensure_list(suite["steps"], f"{path}.steps")
    _ensure(len(steps) > 0, f"{path}.steps must not be empty")
    for index, step in enumerate(steps):
        step_dict = _ensure_dict(step, f"{path}.steps[{index}]")
        _validate_suite_step(step_dict, f"{path}.steps[{index}]", merged_topics, merged_missions, merged_defaults)


def validate_config_triplet(
    package_root: str,
    platform_name: str,
    environment_name: str,
    suite_name: str,
    uav_id: int,
    loaded: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    _ensure(isinstance(uav_id, int) and uav_id > 0, "uav_id must be a positive integer")
    _ensure_string(platform_name, "platform_name")
    _ensure_string(environment_name, "environment_name")
    _ensure_string(suite_name, "suite_name")

    if loaded is None:
        loaded = load_config_triplet(
            package_root=package_root,
            platform_name=platform_name,
            environment_name=environment_name,
            suite_name=suite_name,
            uav_id=uav_id,
        )

    platform = _ensure_dict(loaded.get("platform"), "resolved.platform")
    environment = _ensure_dict(loaded.get("environment"), "resolved.environment")
    suite = _ensure_dict(loaded.get("suite"), "resolved.suite")
    defaults = _ensure_dict(loaded.get("defaults"), "resolved.defaults")
    report = _ensure_dict(loaded.get("report"), "resolved.report")
    topics = _ensure_dict(loaded.get("topics"), "resolved.topics")
    missions = _ensure_dict(loaded.get("missions"), "resolved.missions")
    recording = _ensure_dict(loaded.get("recording"), "resolved.recording")

    _validate_platform_config(platform, "resolved.platform")
    _validate_environment_config(environment, "resolved.environment")
    _validate_defaults(defaults, "resolved.defaults")
    _validate_topics(topics, "resolved.topics", allow_empty_values=False)
    _validate_report_config(report, "resolved.report")
    _validate_recording_config(recording, "resolved.recording")
    _validate_missions(missions, "resolved.missions")
    _validate_suite_config(suite, "resolved.suite", topics, missions, defaults)

    for required_topic_key in ("uav_state", "uav_control_cmd", "uav_setup"):
        _ensure(required_topic_key in topics, f"resolved.topics is missing required key: {required_topic_key}")
        _ensure_string(topics[required_topic_key], f"resolved.topics.{required_topic_key}", allow_empty=False)

    return loaded


def show_effective_config(
    package_root: str,
    platform_name: str,
    environment_name: str,
    suite_name: str,
    uav_id: int,
) -> Dict[str, Any]:
    loaded = load_config_triplet(
        package_root=package_root,
        platform_name=platform_name,
        environment_name=environment_name,
        suite_name=suite_name,
        uav_id=uav_id,
    )
    return {
        "input": {
            "platform": platform_name,
            "environment": environment_name,
            "suite": suite_name,
            "uav_id": uav_id,
            "uav_name": f"/uav{uav_id}",
        },
        "platform": loaded["platform"],
        "environment": loaded["environment"],
        "suite": loaded["suite"],
        "defaults": loaded["defaults"],
        "report": loaded["report"],
        "topics": loaded["topics"],
        "recording": loaded["recording"],
        "missions": loaded["missions"],
    }


def load_config_triplet(
    package_root: str,
    platform_name: str,
    environment_name: str,
    suite_name: str,
    uav_id: int,
) -> Dict[str, Any]:
    config_root = os.path.join(package_root, "config")
    platform_path = os.path.join(config_root, "platforms", f"{platform_name}.yaml")
    environment_path = os.path.join(config_root, "environments", f"{environment_name}.yaml")
    suite_path = os.path.join(config_root, "suites", f"{suite_name}.yaml")
    mission_dir = os.path.join(config_root, "missions")

    _ensure(os.path.isfile(platform_path), f"platform config not found: {platform_path}")
    _ensure(os.path.isfile(environment_path), f"environment config not found: {environment_path}")
    _ensure(os.path.isfile(suite_path), f"suite config not found: {suite_path}")

    platform = load_yaml(platform_path)
    environment = load_yaml(environment_path)
    suite = load_yaml(suite_path)
    mission_files = load_yaml_dir(mission_dir)

    uav_name = f"/uav{uav_id}"
    variables = {
        "uav_id": uav_id,
        "uav_name": uav_name,
        "workspace_root": os.path.abspath(os.path.join(package_root, "..", "..")),
        "package_root": package_root,
    }

    resolved_platform = render_template(platform, variables)
    resolved_environment = render_template(environment, variables)
    resolved_suite = render_template(suite, variables)
    resolved_mission_files = render_template(mission_files, variables)

    defaults = deep_merge(resolved_platform.get("defaults", {}), resolved_environment.get("defaults", {}))
    report = deep_merge(resolved_platform.get("report", {}), resolved_suite.get("report", {}))
    recording = deep_merge(resolved_platform.get("recording", {}), resolved_environment.get("recording", {}))
    missions: Dict[str, Any] = {}
    for filename, mission_data in resolved_mission_files.items():
        mission_name = mission_data.get("name", filename) if isinstance(mission_data, dict) else filename
        missions[mission_name] = mission_data
    missions = deep_merge(missions, resolved_environment.get("missions", {}))

    topics: Dict[str, Any] = {}
    topics.update(resolved_platform.get("topics", {}))
    topics.update(resolved_environment.get("topic_overrides", {}))

    recording_topics = merge_unique_lists(
        resolved_platform.get("recording", {}).get("topic_templates", []),
        resolved_environment.get("recording", {}).get("topic_templates", []),
    )
    recording["topic_templates"] = recording_topics

    loaded = {
        "platform": resolved_platform,
        "environment": resolved_environment,
        "suite": resolved_suite,
        "defaults": defaults,
        "report": report,
        "topics": topics,
        "missions": missions,
        "recording": recording,
    }
    return validate_config_triplet(
        package_root=package_root,
        platform_name=platform_name,
        environment_name=environment_name,
        suite_name=suite_name,
        uav_id=uav_id,
        loaded=loaded,
    )

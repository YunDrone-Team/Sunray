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


def _merge_platform_config(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = deep_merge(base, override)
    base_recording = base.get("recording", {}) if isinstance(base.get("recording"), dict) else {}
    override_recording = override.get("recording", {}) if isinstance(override.get("recording"), dict) else {}
    merged_recording = merged.get("recording", {}) if isinstance(merged.get("recording"), dict) else {}

    if "topic_templates" in base_recording or "topic_templates" in override_recording:
        merged_recording["topic_templates"] = merge_unique_lists(
            base_recording.get("topic_templates", []),
            override_recording.get("topic_templates", []),
        )
    if "exclude_topic_templates" in base_recording or "exclude_topic_templates" in override_recording:
        merged_recording["exclude_topic_templates"] = merge_unique_lists(
            base_recording.get("exclude_topic_templates", []),
            override_recording.get("exclude_topic_templates", []),
        )
    if merged_recording:
        merged["recording"] = merged_recording
    merged.pop("extends", None)
    return merged


def load_platform_config(config_root: str, platform_name: str) -> Dict[str, Any]:
    platform_path = os.path.join(config_root, "platforms", f"{platform_name}.yaml")
    _ensure(os.path.isfile(platform_path), f"platform config not found: {platform_path}")
    platform = load_yaml(platform_path)
    extends = platform.get("extends")
    if not extends:
        return platform

    base_name = _ensure_string(extends, f"{platform_path}.extends")
    base_platform = load_platform_config(config_root, base_name)
    return _merge_platform_config(base_platform, platform)


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
    _ensure(
        isinstance(value, (int, float)) and not isinstance(value, bool),
        f"{path} must be a number",
    )
    return float(value)


def _ensure_integer(value: Any, path: str, minimum: int, description: str) -> float:
    number = _ensure_number(value, path)
    _ensure(
        number >= minimum and float(number).is_integer(),
        f"{path} must be a {description} integer",
    )
    return number


def _validate_optional_numbers(data: Dict[str, Any], path: str, keys: Iterable[str]) -> None:
    for key in keys:
        if key in data:
            _ensure_number(data[key], f"{path}.{key}")


def _validate_optional_strings(data: Dict[str, Any], path: str, keys: Iterable[str]) -> None:
    for key in keys:
        if key in data:
            _ensure_string(data[key], f"{path}.{key}")


def _validate_optional_bools(data: Dict[str, Any], path: str, keys: Iterable[str]) -> None:
    for key in keys:
        if key in data:
            _ensure_bool(data[key], f"{path}.{key}")


def _validate_string_list(
    data: Dict[str, Any],
    path: str,
    key: str,
    allow_empty: bool = False,
) -> None:
    if key not in data:
        return
    for index, item in enumerate(_ensure_list(data[key], f"{path}.{key}")):
        _ensure_string(item, f"{path}.{key}[{index}]", allow_empty=allow_empty)


def _ensure_allowed_keys(
    data: Dict[str, Any],
    path: str,
    allowed: Iterable[str],
    required: Iterable[str] = (),
) -> None:
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


def _ensure_ego_goal_list(value: Any, path: str) -> None:
    points = _ensure_list(value, path)
    _ensure(len(points) > 0, f"{path} must not be empty")
    for index, point in enumerate(points):
        point_path = f"{path}[{index}]"
        _ensure(isinstance(point, (list, tuple)), f"{point_path} must be a list or tuple")
        _ensure(2 <= len(point) <= 3, f"{point_path} must contain 2 or 3 numbers")
        for axis_index, axis_value in enumerate(point):
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
    _ensure_allowed_keys(recording_dict, path, {"bag_prefix", "topic_templates", "exclude_topic_templates"})
    if "bag_prefix" in recording_dict:
        _ensure_string(recording_dict["bag_prefix"], f"{path}.bag_prefix")
    _validate_string_list(recording_dict, path, "topic_templates")
    _validate_string_list(recording_dict, path, "exclude_topic_templates")


def _validate_defaults(defaults: Any, path: str) -> None:
    defaults_dict = _ensure_dict(defaults, path)
    numeric_keys = {
        "hardware_check_timeout_s",
        "camera_sample_duration_s",
        "camera_min_messages",
        "camera_min_rate_hz",
        "camera_max_gap_s",
        "camera_max_identical_frame_ratio",
        "camera_black_mean_threshold",
        "camera_black_dynamic_range_threshold",
        "camera_max_black_frame_ratio",
        "battery_pass_threshold_v",
        "takeoff_reach_radius_m",
        "takeoff_stable_time_s",
        "takeoff_timeout_s",
        "takeoff_command_rate_hz",
        "post_takeoff_settle_time_s",
        "hover_duration_s",
        "waypoint_reach_radius_m",
        "waypoint_stable_time_s",
        "waypoint_hold_time_s",
        "waypoint_timeout_s",
        "ego_goal_z_m",
        "ego_goal_reach_radius_m",
        "ego_goal_stable_time_s",
        "ego_goal_hold_time_s",
        "ego_goal_timeout_s",
        "ego_goal_republish_rate_hz",
        "ego_goal_publish_burst_count",
        "ego_goal_publish_burst_interval_s",
        "ego_keepalive_rate_hz",
        "ego_keepalive_stale_timeout_s",
        "ego_keepalive_zero_velocity_epsilon",
        "ego_post_transition_target_yaw_rad",
        "ego_post_transition_yaw_rate_rad_s",
        "ego_post_transition_hold_after_s",
        "ego_post_transition_target_z_m",
        "visual_landing_height_m",
        "visual_landing_target_zone_radius_m",
    }
    _validate_optional_numbers(defaults_dict, path, numeric_keys)
    for key in ("takeoff_reach_radius_m", "takeoff_timeout_s", "takeoff_command_rate_hz"):
        if key in defaults_dict:
            value = _ensure_number(defaults_dict[key], f"{path}.{key}")
            _ensure(value > 0, f"{path}.{key} must be positive")
    if "takeoff_stable_time_s" in defaults_dict:
        value = _ensure_number(defaults_dict["takeoff_stable_time_s"], f"{path}.takeoff_stable_time_s")
        _ensure(value >= 0, f"{path}.takeoff_stable_time_s must be non-negative")
    if "ego_goal_publish_burst_count" in defaults_dict:
        _ensure_integer(
            defaults_dict["ego_goal_publish_burst_count"],
            f"{path}.ego_goal_publish_burst_count",
            minimum=1,
            description="positive",
        )
    if "ego_goal_publish_burst_interval_s" in defaults_dict:
        value = _ensure_number(
            defaults_dict["ego_goal_publish_burst_interval_s"],
            f"{path}.ego_goal_publish_burst_interval_s",
        )
        _ensure(value >= 0, f"{path}.ego_goal_publish_burst_interval_s must be non-negative")
    if "takeoff_target_z_m" in defaults_dict:
        value = _ensure_number(defaults_dict["takeoff_target_z_m"], f"{path}.takeoff_target_z_m")
        _ensure(value > 0, f"{path}.takeoff_target_z_m must be positive")
    if "waypoint_source" in defaults_dict:
        source = _ensure_string(defaults_dict["waypoint_source"], f"{path}.waypoint_source")
        _ensure(source in {"list", "input"}, f"{path}.waypoint_source must be 'list' or 'input'")
    if "visual_landing_auto_takeoff" in defaults_dict:
        _ensure_bool(defaults_dict["visual_landing_auto_takeoff"], f"{path}.visual_landing_auto_takeoff")
    if "visual_landing_target_zone_radius_m" in defaults_dict:
        value = _ensure_number(
            defaults_dict["visual_landing_target_zone_radius_m"],
            f"{path}.visual_landing_target_zone_radius_m",
        )
        _ensure(value > 0, f"{path}.visual_landing_target_zone_radius_m must be positive")
    _validate_optional_bools(
        defaults_dict,
        path,
        (
            "waypoint_analysis_use_xy_only",
            "ego_goal_use_xy_only",
            "ego_keepalive_enabled",
            "ego_post_transition_enabled",
        ),
    )
    if "ego_post_transition_yaw_rate_rad_s" in defaults_dict:
        value = _ensure_number(
            defaults_dict["ego_post_transition_yaw_rate_rad_s"],
            f"{path}.ego_post_transition_yaw_rate_rad_s",
        )
        _ensure(value > 0, f"{path}.ego_post_transition_yaw_rate_rad_s must be positive")
    if "ego_post_transition_hold_after_s" in defaults_dict:
        value = _ensure_number(
            defaults_dict["ego_post_transition_hold_after_s"],
            f"{path}.ego_post_transition_hold_after_s",
        )
        _ensure(value >= 0, f"{path}.ego_post_transition_hold_after_s must be non-negative")
    if "camera_min_messages" in defaults_dict:
        _ensure_integer(
            defaults_dict["camera_min_messages"],
            f"{path}.camera_min_messages",
            minimum=1,
            description="positive",
        )
    for positive_key in ("camera_sample_duration_s", "camera_min_rate_hz", "camera_max_gap_s"):
        if positive_key in defaults_dict:
            value = _ensure_number(defaults_dict[positive_key], f"{path}.{positive_key}")
            _ensure(value > 0, f"{path}.{positive_key} must be positive")
    if "camera_max_identical_frame_ratio" in defaults_dict:
        value = _ensure_number(
            defaults_dict["camera_max_identical_frame_ratio"],
            f"{path}.camera_max_identical_frame_ratio",
        )
        _ensure(0.0 <= value <= 1.0, f"{path}.camera_max_identical_frame_ratio must be in [0, 1]")
    if "camera_black_mean_threshold" in defaults_dict:
        value = _ensure_number(defaults_dict["camera_black_mean_threshold"], f"{path}.camera_black_mean_threshold")
        _ensure(value >= 0, f"{path}.camera_black_mean_threshold must be non-negative")
    if "camera_black_dynamic_range_threshold" in defaults_dict:
        value = _ensure_number(
            defaults_dict["camera_black_dynamic_range_threshold"],
            f"{path}.camera_black_dynamic_range_threshold",
        )
        _ensure(value >= 0, f"{path}.camera_black_dynamic_range_threshold must be non-negative")
    if "camera_max_black_frame_ratio" in defaults_dict:
        value = _ensure_number(
            defaults_dict["camera_max_black_frame_ratio"],
            f"{path}.camera_max_black_frame_ratio",
        )
        _ensure(0.0 <= value <= 1.0, f"{path}.camera_max_black_frame_ratio must be in [0, 1]")
    _validate_optional_bools(
        defaults_dict,
        path,
        ("camera_require_timestamp_progress", "camera_require_frame_content_change"),
    )


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


def _validate_lidar_config(lidar: Any, path: str) -> None:
    lidar_dict = _ensure_dict(lidar, path)
    _ensure_allowed_keys(
        lidar_dict,
        path,
        {"mid360_auto_check", "mid360_iface", "mid360_config_path", "mid360_timeout_s"},
    )
    if "mid360_auto_check" in lidar_dict:
        _ensure_bool(lidar_dict["mid360_auto_check"], f"{path}.mid360_auto_check")
    if "mid360_iface" in lidar_dict:
        _ensure_string(lidar_dict["mid360_iface"], f"{path}.mid360_iface")
    if "mid360_config_path" in lidar_dict:
        _ensure_string(lidar_dict["mid360_config_path"], f"{path}.mid360_config_path")
    if "mid360_timeout_s" in lidar_dict:
        value = _ensure_number(lidar_dict["mid360_timeout_s"], f"{path}.mid360_timeout_s")
        _ensure(value > 0, f"{path}.mid360_timeout_s must be positive")


def _validate_analysis_config(analysis: Any, path: str) -> None:
    analysis_dict = _ensure_dict(analysis, path)
    _ensure_allowed_keys(analysis_dict, path, {"pose_topic", "pose_topic_fallbacks"})
    if "pose_topic" in analysis_dict:
        _ensure_string(analysis_dict["pose_topic"], f"{path}.pose_topic", allow_empty=False)
    _validate_string_list(analysis_dict, path, "pose_topic_fallbacks")


def _validate_external_sources_config(external_sources: Any, path: str) -> None:
    external_sources_dict = _ensure_dict(external_sources, path)
    for source_key, source_config in external_sources_dict.items():
        _ensure_string(str(source_key), f"{path}.{source_key}")
        source_dict = _ensure_dict(source_config, f"{path}.{source_key}")
        _ensure_allowed_keys(source_dict, f"{path}.{source_key}", {"analysis", "recording"})
        if "analysis" in source_dict:
            _validate_analysis_config(source_dict["analysis"], f"{path}.{source_key}.analysis")
        if "recording" in source_dict:
            _validate_recording_config(source_dict["recording"], f"{path}.{source_key}.recording")


def _validate_platform_config(platform: Dict[str, Any], path: str) -> None:
    _ensure_allowed_keys(
        platform,
        path,
        {"name", "vehicle_type", "report", "capabilities", "lidar", "topics", "recording", "defaults", "analysis"},
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
    if "lidar" in platform:
        _validate_lidar_config(platform["lidar"], f"{path}.lidar")
    if "analysis" in platform:
        _validate_analysis_config(platform["analysis"], f"{path}.analysis")


def _validate_environment_config(environment: Dict[str, Any], path: str) -> None:
    _ensure_allowed_keys(
        environment,
        path,
        {"name", "recording", "defaults", "analysis", "external_sources", "topic_overrides", "missions"},
        {"name"},
    )
    _ensure_string(environment["name"], f"{path}.name")
    if "recording" in environment:
        _validate_recording_config(environment["recording"], f"{path}.recording")
    if "defaults" in environment:
        _validate_defaults(environment["defaults"], f"{path}.defaults")
    if "analysis" in environment:
        _validate_analysis_config(environment["analysis"], f"{path}.analysis")
    if "external_sources" in environment:
        _validate_external_sources_config(environment["external_sources"], f"{path}.external_sources")
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
            if "goals" in mission_value:
                _ensure_ego_goal_list(mission_value["goals"], f"{mission_path}.goals")
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
        _validate_optional_numbers(
            params,
            f"{path}.params",
            (
                "timeout_s",
                "sample_duration_s",
                "min_rate_hz",
                "max_gap_s",
                "max_identical_frame_ratio",
                "black_mean_threshold",
                "black_dynamic_range_threshold",
                "max_black_frame_ratio",
            ),
        )
        if "min_messages" in params:
            _ensure_integer(
                params["min_messages"],
                f"{path}.params.min_messages",
                minimum=0,
                description="non-negative",
            )
        _validate_optional_bools(
            params,
            f"{path}.params",
            (
                "require_non_uniform_frame",
                "require_timestamp_progress",
                "require_frame_content_change",
            ),
        )
        _validate_optional_strings(params, f"{path}.params", ("device_path",))
    elif case_type == "hardware.battery_voltage":
        _ensure("topic_key" in params, f"{path}.params.topic_key is required for hardware.battery_voltage")
        _validate_optional_numbers(params, f"{path}.params", ("timeout_s", "pass_threshold_v"))
    elif case_type == "hardware.topic_alive":
        _ensure("topic_key" in params, f"{path}.params.topic_key is required for hardware.topic_alive")
        _validate_optional_numbers(params, f"{path}.params", ("timeout_s",))
    elif case_type == "hardware.lidar_health":
        _validate_optional_strings(params, f"{path}.params", ("imu_topic_pattern", "lidar_topic_pattern"))
        _validate_optional_numbers(
            params,
            f"{path}.params",
            ("timeout_s", "sample_duration_s", "min_rate_hz", "max_gap_s"),
        )
        for integer_key in ("min_messages", "min_points_per_cloud", "min_valid_clouds"):
            if integer_key in params:
                _ensure_integer(
                    params[integer_key],
                    f"{path}.params.{integer_key}",
                    minimum=0,
                    description="non-negative",
                )
    elif case_type == "flight.hover":
        _validate_optional_numbers(params, f"{path}.params", ("duration_s",))
    elif case_type == "flight.ego_goal":
        _ensure("mission_key" in params, f"{path}.params.mission_key is required for flight.ego_goal")
        mission_key = _ensure_string(params["mission_key"], f"{path}.params.mission_key")
        _ensure(mission_key in merged_missions, f"{path}.params.mission_key references unknown mission: {mission_key}")
        mission = merged_missions[mission_key]
        if isinstance(mission, dict):
            _ensure(
                "goals" in mission or "waypoints" in mission,
                f"merged.missions.{mission_key} must define goals or waypoints",
            )
            if "goals" in mission:
                _ensure_ego_goal_list(mission["goals"], f"merged.missions.{mission_key}.goals")
            if "waypoints" in mission:
                _ensure_ego_goal_list(mission["waypoints"], f"merged.missions.{mission_key}.waypoints")
        elif isinstance(mission, list):
            _ensure_ego_goal_list(mission, f"merged.missions.{mission_key}")
        else:
            raise ConfigValidationError(f"merged.missions.{mission_key} must be a mapping or goal list")
        _validate_optional_strings(
            params,
            f"{path}.params",
            ("goal_topic", "frame_id", "goal_source", "pos_cmd_topic", "control_cmd_topic"),
        )
        _validate_optional_numbers(
            params,
            f"{path}.params",
            (
                "z_m",
                "reach_radius_m",
                "stable_time_s",
                "hold_time_s",
                "timeout_s",
                "republish_rate_hz",
                "publish_burst_count",
                "publish_burst_interval_s",
                "keepalive_rate_hz",
                "keepalive_stale_timeout_s",
                "keepalive_zero_velocity_epsilon",
                "post_transition_target_yaw_rad",
                "post_transition_yaw_rate_rad_s",
                "post_transition_hold_after_s",
                "post_transition_target_z_m",
            ),
        )
        if "publish_burst_count" in params:
            _ensure_integer(
                params["publish_burst_count"],
                f"{path}.params.publish_burst_count",
                minimum=1,
                description="positive",
            )
        if "publish_burst_interval_s" in params:
            value = _ensure_number(
                params["publish_burst_interval_s"],
                f"{path}.params.publish_burst_interval_s",
            )
            _ensure(value >= 0, f"{path}.params.publish_burst_interval_s must be non-negative")
        _validate_optional_bools(
            params,
            f"{path}.params",
            ("use_xy_only", "keepalive_enabled", "post_transition_enabled"),
        )
    elif case_type == "flight.visual_landing":
        _validate_optional_strings(params, f"{path}.params", ("launch_file",))
        _validate_optional_bools(params, f"{path}.params", ("auto_takeoff",))
        _validate_optional_numbers(params, f"{path}.params", ("height_m",))
        if "launch_args" in params:
            launch_args = _ensure_dict(params["launch_args"], f"{path}.params.launch_args")
            for key, value in launch_args.items():
                _ensure_string(key, f"{path}.params.launch_args.{key}")
                _ensure(
                    isinstance(value, (str, int, float, bool)),
                    f"{path}.params.launch_args.{key} must be a scalar",
                )
        if "remaps" in params:
            remaps = _ensure_list(params["remaps"], f"{path}.params.remaps")
            for index, remap in enumerate(remaps):
                remap_path = f"{path}.params.remaps[{index}]"
                remap_dict = _ensure_dict(remap, remap_path)
                _ensure("from" in remap_dict, f"{remap_path}.from is required")
                _ensure("to" in remap_dict, f"{remap_path}.to is required")
                _ensure_string(remap_dict["from"], f"{remap_path}.from")
                _ensure_string(remap_dict["to"], f"{remap_path}.to")
        _validate_string_list(params, f"{path}.params", "failure_patterns")
        _validate_string_list(params, f"{path}.params", "pre_stop_nodes")
    elif case_type == "flight.waypoint":
        waypoint_source = params.get("waypoint_source", merged_defaults.get("waypoint_source", "list"))
        waypoint_source = _ensure_string(waypoint_source, f"{path}.params.waypoint_source")
        _ensure(waypoint_source in {"list", "input"}, f"{path}.params.waypoint_source must be 'list' or 'input'")
        if waypoint_source == "list":
            mission_key = params.get("mission_key")
            _ensure(mission_key is not None, f"{path}.params.mission_key is required when waypoint_source=list")
            mission_key = _ensure_string(mission_key, f"{path}.params.mission_key")
            _ensure(
                mission_key in merged_missions,
                f"{path}.params.mission_key references unknown mission: {mission_key}",
            )
        _validate_optional_numbers(
            params,
            f"{path}.params",
            ("reach_radius_m", "stable_time_s", "hold_time_s", "timeout_s"),
        )


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
        {"name", "description", "record_rosbag", "stop_on_failure", "report", "platform_requirements", "steps"},
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
    if "platform_requirements" in suite:
        requirements = _ensure_dict(suite["platform_requirements"], f"{path}.platform_requirements")
        _ensure_allowed_keys(requirements, f"{path}.platform_requirements", {"capabilities", "topics"})
        if "capabilities" in requirements:
            capabilities = _ensure_list(
                requirements["capabilities"],
                f"{path}.platform_requirements.capabilities",
            )
            for index, capability in enumerate(capabilities):
                _ensure_string(capability, f"{path}.platform_requirements.capabilities[{index}]")
        if "topics" in requirements:
            topic_keys = _ensure_list(requirements["topics"], f"{path}.platform_requirements.topics")
            for index, topic_key in enumerate(topic_keys):
                _ensure_string(topic_key, f"{path}.platform_requirements.topics[{index}]")

    steps = _ensure_list(suite["steps"], f"{path}.steps")
    _ensure(len(steps) > 0, f"{path}.steps must not be empty")
    for index, step in enumerate(steps):
        step_dict = _ensure_dict(step, f"{path}.steps[{index}]")
        _validate_suite_step(step_dict, f"{path}.steps[{index}]", merged_topics, merged_missions, merged_defaults)


def _validate_platform_requirements(
    suite: Dict[str, Any],
    platform: Dict[str, Any],
    topics: Dict[str, Any],
    path: str,
) -> None:
    requirements = suite.get("platform_requirements")
    if not requirements:
        return

    requirements_dict = _ensure_dict(requirements, path)
    required_topics = [str(item) for item in requirements_dict.get("topics", [])]
    required_capabilities = [str(item) for item in requirements_dict.get("capabilities", [])]
    capabilities = platform.get("capabilities") or {}

    for topic_key in required_topics:
        _ensure(topic_key in topics, f"{path}.topics requires missing topic key: {topic_key}")
        _ensure_string(topics[topic_key], f"resolved.topics.{topic_key}", allow_empty=False)

    for capability in required_capabilities:
        _ensure(
            bool(capabilities.get(capability, False)),
            f"{path}.capabilities requires platform capability '{capability}'",
        )


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
    analysis = _ensure_dict(loaded.get("analysis"), "resolved.analysis")
    report = _ensure_dict(loaded.get("report"), "resolved.report")
    topics = _ensure_dict(loaded.get("topics"), "resolved.topics")
    missions = _ensure_dict(loaded.get("missions"), "resolved.missions")
    recording = _ensure_dict(loaded.get("recording"), "resolved.recording")

    _validate_platform_config(platform, "resolved.platform")
    _validate_environment_config(environment, "resolved.environment")
    _validate_defaults(defaults, "resolved.defaults")
    _validate_analysis_config(analysis, "resolved.analysis")
    _validate_topics(topics, "resolved.topics", allow_empty_values=False)
    _validate_report_config(report, "resolved.report")
    _validate_recording_config(recording, "resolved.recording")
    _validate_missions(missions, "resolved.missions")
    _validate_suite_config(suite, "resolved.suite", topics, missions, defaults)
    _validate_platform_requirements(suite, platform, topics, "resolved.suite.platform_requirements")

    for required_topic_key in ("uav_state", "uav_control_cmd", "uav_setup"):
        _ensure(required_topic_key in topics, f"resolved.topics is missing required key: {required_topic_key}")
        _ensure_string(topics[required_topic_key], f"resolved.topics.{required_topic_key}", allow_empty=False)

    return loaded


def load_config_triplet(
    package_root: str,
    platform_name: str,
    environment_name: str,
    suite_name: str,
    uav_id: int,
    external_source: Optional[int] = None,
    suite_override: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    config_root = os.path.join(package_root, "config")
    environment_path = os.path.join(config_root, "environments", f"{environment_name}.yaml")
    mission_dir = os.path.join(config_root, "missions")

    _ensure(os.path.isfile(environment_path), f"environment config not found: {environment_path}")
    _ensure(suite_override is not None, "runtime suite is required; use the dashboard generated suite")

    platform = load_platform_config(config_root, platform_name)
    environment = load_yaml(environment_path)
    suite = copy.deepcopy(suite_override)
    mission_files = load_yaml_dir(mission_dir)

    uav_name = f"/uav{uav_id}"
    variables = {
        "uav_id": uav_id,
        "uav_name": uav_name,
        "external_source": external_source if external_source is not None else "",
        "workspace_root": os.path.abspath(os.path.join(package_root, "..", "..")),
        "package_root": package_root,
    }

    resolved_platform = render_template(platform, variables)
    resolved_environment = render_template(environment, variables)
    resolved_suite = render_template(suite, variables)
    resolved_mission_files = render_template(mission_files, variables)

    defaults = deep_merge(resolved_platform.get("defaults", {}), resolved_environment.get("defaults", {}))
    analysis = deep_merge(resolved_platform.get("analysis", {}), resolved_environment.get("analysis", {}))
    report = deep_merge(resolved_platform.get("report", {}), resolved_suite.get("report", {}))
    recording = deep_merge(resolved_platform.get("recording", {}), resolved_environment.get("recording", {}))

    external_source_config: Dict[str, Any] = {}
    external_sources = resolved_environment.get("external_sources", {})
    if isinstance(external_sources, dict) and external_source is not None:
        external_source_config = external_sources.get(str(external_source)) or external_sources.get("default") or {}
        if isinstance(external_source_config, dict):
            analysis = deep_merge(analysis, external_source_config.get("analysis", {}))
            recording = deep_merge(recording, external_source_config.get("recording", {}))
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
        external_source_config.get("recording", {}).get("topic_templates", [])
        if isinstance(external_source_config, dict)
        else [],
    )
    excluded_recording_topics = set(
        external_source_config.get("recording", {}).get("exclude_topic_templates", [])
        if isinstance(external_source_config, dict)
        else []
    )
    if excluded_recording_topics:
        recording_topics = [topic for topic in recording_topics if topic not in excluded_recording_topics]
    recording["topic_templates"] = recording_topics
    recording.pop("exclude_topic_templates", None)

    loaded = {
        "platform": resolved_platform,
        "environment": resolved_environment,
        "suite": resolved_suite,
        "defaults": defaults,
        "analysis": analysis,
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

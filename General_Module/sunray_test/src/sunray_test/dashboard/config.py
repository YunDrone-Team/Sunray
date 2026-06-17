import os
from typing import Any, Dict, List

import yaml

from sunray_test.dashboard.types import PACKAGE_ROOT, VALID_ENVIRONMENTS, VALID_ITEM_GROUPS


def dashboard_config_path(config_name: str, package_root: str = PACKAGE_ROOT) -> str:
    if os.path.isabs(config_name):
        return config_name
    if config_name == "demo":
        config_name = "dashboard"
    if config_name.endswith(".yaml"):
        return os.path.join(package_root, "config", "dashboard", config_name)
    return os.path.join(package_root, "config", "dashboard", f"{config_name}.yaml")


def load_dashboard_config(config_name: str, package_root: str = PACKAGE_ROOT) -> Dict[str, Any]:
    path = dashboard_config_path(config_name, package_root)
    if not os.path.isfile(path):
        raise SystemExit(f"dashboard config not found: {path}")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"dashboard config must be a mapping: {path}")
    return data


def validate_dashboard_config(config: Dict[str, Any]) -> None:
    errors: List[str] = []
    raw_items = config.get("test_items", [])
    if not isinstance(raw_items, list) or not raw_items:
        errors.append("test_items must be a non-empty list")
        raw_items = []

    item_ids: List[str] = []
    item_groups: Dict[str, str] = {}
    hardware_ids = set()
    for index, raw_item in enumerate(raw_items):
        _validate_test_item(errors, item_ids, item_groups, hardware_ids, index, raw_item)

    known_item_ids = set(item_ids)
    _validate_default_items(errors, config, known_item_ids)
    _validate_hardware_dependencies(errors, raw_items, hardware_ids)

    rules = _validate_bringup_rules(errors, config, known_item_ids)
    known_conditions = known_item_ids | set(str(name) for name in rules.keys()) | {"always"}
    _validate_tab_sections(errors, config, known_item_ids, known_conditions)
    _validate_variables(errors, config, known_conditions)
    _validate_runtime_profiles(errors, config, known_item_ids)

    if errors:
        raise SystemExit("dashboard config validation failed:\n  - " + "\n  - ".join(errors))


def _validate_test_item(
    errors: List[str],
    item_ids: List[str],
    item_groups: Dict[str, str],
    hardware_ids: set,
    index: int,
    raw_item: Any,
) -> None:
    path = f"test_items[{index}]"
    if not isinstance(raw_item, dict):
        errors.append(f"{path} must be a mapping")
        return

    item_id = str(raw_item.get("id", "")).strip()
    if not item_id:
        errors.append(f"{path}.id is required")
        return
    if item_id in item_groups:
        errors.append(f"{path}.id duplicates item id: {item_id}")

    group = str(raw_item.get("group", "hardware")).strip()
    if group not in VALID_ITEM_GROUPS:
        allowed = ", ".join(sorted(VALID_ITEM_GROUPS))
        errors.append(f"{path}.group must be one of: {allowed}")

    step = raw_item.get("step")
    if not isinstance(step, dict):
        errors.append(f"{path}.step must be a mapping")
    elif "case" not in step or "type" not in step:
        errors.append(f"{path}.step must contain case and type")

    item_ids.append(item_id)
    item_groups[item_id] = group
    if group == "hardware":
        hardware_ids.add(item_id)


def _validate_default_items(errors: List[str], config: Dict[str, Any], known_item_ids: set) -> None:
    for default_item in config.get("default_items", []):
        if str(default_item) not in known_item_ids:
            errors.append(f"default_items references unknown item: {default_item}")


def _validate_hardware_dependencies(
    errors: List[str],
    raw_items: List[Any],
    hardware_ids: set,
) -> None:
    for index, raw_item in enumerate(raw_items):
        if not isinstance(raw_item, dict):
            continue
        item_id = str(raw_item.get("id", "")).strip()
        for hardware_id in raw_item.get("required_hardware", []):
            if str(hardware_id) not in hardware_ids:
                errors.append(
                    f"test_items[{index}] {item_id} requires unknown hardware item: {hardware_id}"
                )


def _validate_bringup_rules(
    errors: List[str],
    config: Dict[str, Any],
    known_item_ids: set,
) -> Dict[str, Any]:
    bringup = config.get("bringup", {})
    rules = bringup.get("rules", {}) if isinstance(bringup, dict) else {}
    if not isinstance(rules, dict):
        errors.append("bringup.rules must be a mapping")
        return {}

    for rule_name, rule in rules.items():
        if not isinstance(rule, dict):
            errors.append(f"bringup.rules.{rule_name} must be a mapping")
            continue
        _validate_rule_items(errors, rule, f"bringup.rules.{rule_name}", known_item_ids)
        _validate_rule_groups(errors, rule, f"bringup.rules.{rule_name}")
    return rules


def _validate_rule_items(
    errors: List[str],
    rule: Dict[str, Any],
    path: str,
    known_item_ids: set,
) -> None:
    for key in ("any_items", "all_items", "none_items"):
        for item_id in rule.get(key, []):
            if str(item_id) not in known_item_ids:
                errors.append(f"{path}.{key} references unknown item: {item_id}")


def _validate_rule_groups(errors: List[str], rule: Dict[str, Any], path: str) -> None:
    for key in ("any_groups", "all_groups", "none_groups"):
        for group in rule.get(key, []):
            if str(group) not in VALID_ITEM_GROUPS:
                errors.append(f"{path}.{key} references unknown group: {group}")


def _validate_tab_sections(
    errors: List[str],
    config: Dict[str, Any],
    known_item_ids: set,
    known_conditions: set,
) -> None:
    for section_name in ("bringup", "runner"):
        section = config.get(section_name, {})
        if not isinstance(section, dict):
            errors.append(f"{section_name} must be a mapping")
            continue
        _validate_tab_environment_section(
            errors,
            section_name,
            section,
            known_item_ids,
            known_conditions,
        )


def _validate_tab_environment_section(
    errors: List[str],
    section_name: str,
    section: Dict[str, Any],
    known_item_ids: set,
    known_conditions: set,
) -> None:
    for environment, env_config in section.items():
        if section_name == "bringup" and environment == "rules":
            continue
        if str(environment) not in VALID_ENVIRONMENTS:
            allowed = ", ".join(sorted(VALID_ENVIRONMENTS))
            errors.append(f"{section_name}.{environment} must be one of: {allowed}")
            continue
        tabs = env_config.get("tabs", []) if isinstance(env_config, dict) else None
        if not isinstance(tabs, list):
            errors.append(f"{section_name}.{environment}.tabs must be a list")
            continue
        _validate_tabs(errors, section_name, environment, tabs, known_conditions)


def _validate_tabs(
    errors: List[str],
    section_name: str,
    environment: str,
    tabs: List[Any],
    known_conditions: set,
) -> None:
    for index, tab in enumerate(tabs):
        tab_path = f"{section_name}.{environment}.tabs[{index}]"
        if not isinstance(tab, dict):
            errors.append(f"{tab_path} must be a mapping")
            continue
        when = str(tab.get("when", "always"))
        if when not in known_conditions:
            errors.append(f"{tab_path}.when references unknown condition: {when}")
        if not str(tab.get("command", "")).strip():
            errors.append(f"{tab_path}.command is required")


def _validate_variables(errors: List[str], config: Dict[str, Any], known_conditions: set) -> None:
    for variable_name, variable_config in (config.get("variables", {}) or {}).items():
        if not isinstance(variable_config, dict):
            errors.append(f"variables.{variable_name} must be a mapping")
            continue
        for index, rule in enumerate(variable_config.get("rules", [])):
            when = str(rule.get("when", ""))
            if when not in known_conditions:
                errors.append(
                    f"variables.{variable_name}.rules[{index}].when references unknown condition: {when}"
                )


def _validate_runtime_profiles(
    errors: List[str],
    config: Dict[str, Any],
    known_item_ids: set,
) -> None:
    profiles = config.get("runtime_profiles", {})
    if not isinstance(profiles, dict) or not str(profiles.get("default", "")).strip():
        errors.append("runtime_profiles.default is required")
        return

    for index, rule in enumerate(profiles.get("rules", [])):
        if not isinstance(rule, dict):
            errors.append(f"runtime_profiles.rules[{index}] must be a mapping")
            continue
        if not str(rule.get("profile", "")).strip():
            errors.append(f"runtime_profiles.rules[{index}].profile is required")
        _validate_runtime_profile_rule(errors, rule, index, known_item_ids)


def _validate_runtime_profile_rule(
    errors: List[str],
    rule: Dict[str, Any],
    index: int,
    known_item_ids: set,
) -> None:
    for key in ("any_items", "all_items"):
        for item_id in rule.get(key, []):
            if str(item_id) not in known_item_ids:
                errors.append(f"runtime_profiles.rules[{index}].{key} references unknown item: {item_id}")

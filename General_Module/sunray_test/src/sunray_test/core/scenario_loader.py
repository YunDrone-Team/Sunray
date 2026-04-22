import os
from typing import Any, Dict, List

from sunray_test.core.suite_loader import load_yaml


class _SafeFormatDict(dict):
    def __missing__(self, key):
        return "{" + key + "}"


def _render_template_safe(value: Any, variables: Dict[str, Any]) -> Any:
    if isinstance(value, str):
        return value.format_map(_SafeFormatDict(variables))
    if isinstance(value, list):
        return [_render_template_safe(item, variables) for item in value]
    if isinstance(value, dict):
        return {key: _render_template_safe(item, variables) for key, item in value.items()}
    return value


def scenario_config_dir(package_root: str) -> str:
    return os.path.join(package_root, "config", "scenarios")


def scenario_config_path(package_root: str, scenario_name: str) -> str:
    return os.path.join(scenario_config_dir(package_root), f"{scenario_name}.yaml")


def list_scenarios(package_root: str) -> List[Dict[str, str]]:
    directory = scenario_config_dir(package_root)
    scenarios: List[Dict[str, str]] = []
    if not os.path.isdir(directory):
        return scenarios

    for filename in sorted(os.listdir(directory)):
        if not filename.endswith(".yaml"):
            continue
        path = os.path.join(directory, filename)
        data = load_yaml(path)
        scenario_name = data.get("name") or os.path.splitext(filename)[0]
        scenarios.append(
            {
                "name": scenario_name,
                "display_name": data.get("display_name", scenario_name),
                "description": data.get("description", ""),
            }
        )
    return scenarios


def load_scenario(package_root: str, scenario_name: str, variables: Dict[str, Any]) -> Dict[str, Any]:
    path = scenario_config_path(package_root, scenario_name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"scenario config not found: {path}")

    scenario = _render_template_safe(load_yaml(path), variables)
    if not scenario.get("windows"):
        raise ValueError(f"scenario '{scenario_name}' has no windows defined")
    if not scenario.get("runner"):
        raise ValueError(f"scenario '{scenario_name}' has no runner config")
    return scenario

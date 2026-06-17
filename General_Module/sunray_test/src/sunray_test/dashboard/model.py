import copy
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

from sunray_test.dashboard.config import load_dashboard_config, validate_dashboard_config
from sunray_test.dashboard.types import (
    PACKAGE_ROOT,
    VALID_ENVIRONMENTS,
    DashboardPlan,
    DashboardSelection,
    TestItem,
)


def dedupe(values: Sequence[str]) -> List[str]:
    result: List[str] = []
    for value in values:
        if value not in result:
            result.append(value)
    return result


class SafeFormatDict(dict):
    def __missing__(self, key):
        return "{" + key + "}"


@dataclass
class DashboardModel:
    config: Dict[str, Any]
    package_root: str = PACKAGE_ROOT

    def __post_init__(self) -> None:
        validate_dashboard_config(self.config)
        self.test_items = [
            TestItem(
                item_id=str(item["id"]),
                name=str(item.get("name", item["id"])),
                group=str(item.get("group", "hardware")),
                step=copy.deepcopy(item["step"]),
                requires_airborne=bool(item.get("requires_airborne", False)),
                required_hardware=tuple(str(value) for value in item.get("required_hardware", [])),
                sim_only=bool(item.get("sim_only", False)),
                exp_only=bool(item.get("exp_only", False)),
                tags=tuple(str(value) for value in item.get("tags", [])),
            )
            for item in self.config.get("test_items", [])
        ]
        self.item_by_id = {item.item_id: item for item in self.test_items}
        self.item_id_aliases = {}
        for item in self.test_items:
            self.item_id_aliases[item.item_id] = item.item_id
            case_id = str(item.step.get("case", "")).strip()
            if case_id:
                self.item_id_aliases[case_id] = item.item_id
        self.default_items = [str(value) for value in self.config.get("default_items", [])]

    @classmethod
    def load(cls, config_name: str, package_root: str = PACKAGE_ROOT) -> "DashboardModel":
        return cls(load_dashboard_config(config_name, package_root), package_root)

    def items_by_group(self, group: str) -> List[TestItem]:
        return [item for item in self.test_items if item.group == group]

    def parse_item_ids(self, raw_items: str) -> List[str]:
        raw_item_ids = [part.strip() for part in raw_items.split(",") if part.strip()]
        item_ids = [self.item_id_aliases.get(item_id, item_id) for item_id in raw_item_ids]
        unknown = [raw_item_ids[index] for index, item_id in enumerate(item_ids) if item_id not in self.item_by_id]
        if unknown:
            raise SystemExit(f"未知测试项目: {', '.join(unknown)}")
        return dedupe(item_ids)

    def config_summary(self) -> Dict[str, Any]:
        bringup = self.config.get("bringup", {})
        runner = self.config.get("runner", {})
        return {
            "name": str(self.config.get("name", "dashboard")),
            "test_items": {
                "total": len(self.test_items),
                "hardware": len(self.items_by_group("hardware")),
                "function": len(self.items_by_group("function")),
                "ids": [item.item_id for item in self.test_items],
            },
            "default_items": list(self.default_items),
            "runtime_profiles": {
                "default": str((self.config.get("runtime_profiles", {}) or {}).get("default", "")),
                "rules": len((self.config.get("runtime_profiles", {}) or {}).get("rules", []) or []),
            },
            "bringup_tabs": {
                environment: len((bringup.get(environment, {}) or {}).get("tabs", []) or [])
                for environment in sorted(VALID_ENVIRONMENTS)
            },
            "runner_tabs": {
                environment: len((runner.get(environment, {}) or {}).get("tabs", []) or [])
                for environment in sorted(VALID_ENVIRONMENTS)
            },
            "variables": sorted(str(name) for name in (self.config.get("variables", {}) or {}).keys()),
        }

    def resolve_item_dependencies(self, requested_item_ids: Sequence[str]) -> DashboardSelection:
        selected = set(requested_item_ids)
        auto_added: List[str] = []
        for item_id in requested_item_ids:
            item = self.item_by_id[item_id]
            if item.group != "function":
                continue
            for hardware_id in item.required_hardware:
                if hardware_id not in selected:
                    selected.add(hardware_id)
                    auto_added.append(hardware_id)
        ordered = [item.item_id for item in self.test_items if item.item_id in selected]
        return DashboardSelection(
            requested_item_ids=dedupe(requested_item_ids),
            item_ids=ordered,
            auto_added_item_ids=dedupe(auto_added),
        )

    def validate_item_environment(self, item_ids: Sequence[str], environment: str) -> None:
        errors: List[str] = []
        for item_id in item_ids:
            item = self.item_by_id[item_id]
            if item.sim_only and environment != "sim":
                errors.append(f"{item.item_id} 只能在 sim 环境运行")
            if item.exp_only and environment != "exp":
                errors.append(f"{item.item_id} 只能在 exp 环境运行")
        if errors:
            raise SystemExit("测试项目与当前环境不匹配:\n  - " + "\n  - ".join(errors))

    def dependency_descriptions(self, requested_item_ids: Sequence[str]) -> List[str]:
        descriptions: List[str] = []
        for item_id in requested_item_ids:
            item = self.item_by_id[item_id]
            if item.group == "function" and item.required_hardware:
                hardware_names = [
                    f"{hardware_id}({self.item_by_id[hardware_id].name})"
                    for hardware_id in item.required_hardware
                ]
                descriptions.append(f"{item.item_id}({item.name}) -> {', '.join(hardware_names)}")
        return descriptions

    def build_suite(
        self,
        item_ids: Sequence[str],
        record_rosbag: bool,
        continue_on_failure: bool,
    ) -> Dict[str, Any]:
        selected_items = [self.item_by_id[item_id] for item_id in dedupe(item_ids)]
        needs_airborne = any(item.requires_airborne for item in selected_items)
        steps: List[Dict[str, Any]] = []

        for item in selected_items:
            if item.group == "hardware":
                steps.append(copy.deepcopy(item.step))

        if needs_airborne:
            steps.append({"phase": "arm_and_takeoff"})
            for item in selected_items:
                if item.group == "function":
                    steps.append(copy.deepcopy(item.step))

        return {
            "name": str(self.config.get("name", "dashboard")),
            "description": str(self.config.get("description", "Runtime suite generated by the dashboard")),
            "record_rosbag": bool(record_rosbag),
            "stop_on_failure": not bool(continue_on_failure),
            "report": {"title": str(self.config.get("report_title", "Sunray 测试报告"))},
            "steps": steps,
        }

    def default_external_source(self, environment: str, override: int = None) -> int:
        if override is not None:
            return override
        fallback = 2 if environment == "sim" else 3
        return int((self.config.get("external_sources", {}) or {}).get(environment, fallback))

    def build_condition_context(self, item_ids: Sequence[str]) -> Dict[str, bool]:
        selected = set(item_ids)
        selected_groups = {self.item_by_id[item_id].group for item_id in selected}
        context: Dict[str, bool] = {}
        for name, rule in (self.config.get("bringup", {}).get("rules", {}) or {}).items():
            any_items = {str(value) for value in rule.get("any_items", [])}
            all_items = {str(value) for value in rule.get("all_items", [])}
            none_items = {str(value) for value in rule.get("none_items", [])}
            any_groups = {str(value) for value in rule.get("any_groups", [])}
            all_groups = {str(value) for value in rule.get("all_groups", [])}
            none_groups = {str(value) for value in rule.get("none_groups", [])}
            enabled = False
            if any_items and selected.intersection(any_items):
                enabled = True
            if all_items and all_items.issubset(selected):
                enabled = True
            if any_groups and selected_groups.intersection(any_groups):
                enabled = True
            if all_groups and all_groups.issubset(selected_groups):
                enabled = True
            if none_items or none_groups:
                if not (any_items or all_items or any_groups or all_groups):
                    enabled = True
                if none_items and selected.intersection(none_items):
                    enabled = False
                if none_groups and selected_groups.intersection(none_groups):
                    enabled = False
            context[str(name)] = enabled
        for item_id in selected:
            context[item_id] = True
        context["always"] = True
        return context

    def describe_config_variable(self, name: str, condition_context: Dict[str, bool]) -> Tuple[str, str]:
        config = (self.config.get("variables", {}) or {}).get(name, {})
        for rule in config.get("rules", []):
            when = str(rule.get("when", ""))
            if condition_context.get(when, False):
                return str(rule.get("value", "")), when
        return str(config.get("default", "")), "default"

    def choose_runtime_profile_with_reason(self, item_ids: Sequence[str]) -> Tuple[str, str]:
        profiles = self.config.get("runtime_profiles", {})
        selected = set(item_ids)
        for rule in profiles.get("rules", []):
            any_items = {str(value) for value in rule.get("any_items", [])}
            all_items = {str(value) for value in rule.get("all_items", [])}
            if any_items and selected.intersection(any_items):
                return str(rule["profile"]), "any_items=" + ",".join(sorted(selected.intersection(any_items)))
            if all_items and all_items.issubset(selected):
                return str(rule["profile"]), "all_items=" + ",".join(sorted(all_items))
        default_profile = profiles.get("default")
        if not default_profile:
            raise SystemExit("dashboard runtime_profiles.default is required")
        return str(default_profile), "default"

    def build_runtime_state(
        self,
        item_ids: Sequence[str],
        environment: str,
        profile: str,
        external_source: int,
        uav_id: int,
    ) -> Dict[str, Any]:
        condition_context = self.build_condition_context(item_ids)
        variable_values: Dict[str, str] = {}
        variable_reasons: Dict[str, str] = {}
        for name in (self.config.get("variables", {}) or {}).keys():
            value, reason = self.describe_config_variable(str(name), condition_context)
            variable_values[str(name)] = value
            variable_reasons[str(name)] = reason

        return {
            "environment": environment,
            "profile": profile,
            "external_source": external_source,
            "condition_context": condition_context,
            "variables": variable_values,
            "variable_reasons": variable_reasons,
            "template_variables": {
                "uav_id": uav_id,
                "external_source": external_source,
                **variable_values,
            },
        }

    def render_config_template(self, value: str, variables: Dict[str, Any]) -> str:
        return str(value).format_map(SafeFormatDict(variables))

    def build_tabs(self, section_name: str, environment: str, runtime_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        condition_context = runtime_state["condition_context"]
        tabs: List[Dict[str, Any]] = []
        raw_tabs = self.config.get(section_name, {}).get(environment, {}).get("tabs", [])
        for raw_tab in raw_tabs:
            when = str(raw_tab.get("when", "always"))
            if not condition_context.get(when, False):
                continue
            tabs.append(
                {
                    "title": str(raw_tab.get("title", "tab")),
                    "delay_s": float(raw_tab.get("delay_s", 0.0)),
                    "hold_open": bool(raw_tab.get("hold_open", True)),
                    "command": self.render_config_template(
                        str(raw_tab["command"]),
                        runtime_state["template_variables"],
                    ),
                }
            )
        return tabs

    def build_plan(
        self,
        requested_item_ids: Sequence[str],
        environment: str,
        uav_id: int,
        external_source_override: int,
        record_rosbag: bool,
        continue_on_failure: bool,
    ) -> DashboardPlan:
        requested = dedupe(requested_item_ids)
        if not requested:
            raise SystemExit("未选择任何测试项目")
        selection = self.resolve_item_dependencies(requested)
        self.validate_item_environment(selection.item_ids, environment)
        profile, profile_reason = self.choose_runtime_profile_with_reason(selection.item_ids)
        external_source = self.default_external_source(environment, external_source_override)
        runtime_state = self.build_runtime_state(selection.item_ids, environment, profile, external_source, uav_id)
        suite = self.build_suite(selection.item_ids, record_rosbag, continue_on_failure)
        return DashboardPlan(
            selection=selection,
            environment=environment,
            profile=profile,
            profile_reason=profile_reason,
            external_source=external_source,
            runtime_state=runtime_state,
            suite=suite,
        )

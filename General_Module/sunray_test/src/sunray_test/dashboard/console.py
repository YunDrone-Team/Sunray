from typing import Dict, List, Sequence

import yaml

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest
from sunray_test.dashboard.text_format import display_width, pad_cells
from sunray_test.dashboard.types import DashboardPlan, TestItem


PHASE_DISPLAY_NAMES = {
    "arm_and_takeoff": "解锁/起飞",
    "land": "降落",
}


def print_config_check(model: DashboardModel) -> None:
    summary = model.config_summary()
    print("\n=== Dashboard Config Check ===", flush=True)
    print("status: ok", flush=True)
    print(f"name: {summary['name']}", flush=True)
    items = summary["test_items"]
    print(
        f"items: total={items['total']}, hardware={items['hardware']}, "
        f"function={items['function']}",
        flush=True,
    )
    print(f"default_items: {', '.join(summary['default_items']) or '-'}", flush=True)
    profiles = summary["runtime_profiles"]
    print(f"runtime_profiles: default={profiles['default']}, rules={profiles['rules']}", flush=True)
    print(
        "bringup_tabs: "
        + ", ".join(f"{env}={count}" for env, count in summary["bringup_tabs"].items()),
        flush=True,
    )
    print(
        "runner_tabs: "
        + ", ".join(f"{env}={count}" for env, count in summary["runner_tabs"].items()),
        flush=True,
    )
    print(f"variables: {', '.join(summary['variables']) or '-'}", flush=True)


def format_hardware_refs(item: TestItem, item_by_id: Dict[str, TestItem]) -> str:
    refs: List[str] = []
    for hardware_id in item.required_hardware:
        hardware_item = item_by_id.get(hardware_id)
        refs.append(f"{hardware_id}({hardware_item.name})" if hardware_item else hardware_id)
    return ",".join(refs)


def item_hints(item: TestItem, item_by_id: Dict[str, TestItem]) -> List[str]:
    hints: List[str] = []
    if item.required_hardware:
        hints.append("依赖硬件: " + format_hardware_refs(item, item_by_id))
    if item.sim_only:
        hints.append("仅仿真")
    if item.exp_only:
        hints.append("仅实机")
    hints.extend(item.tags)
    return hints


def format_item_line(index: int, item: TestItem, item_by_id: Dict[str, TestItem]) -> str:
    suffix = ""
    hints = item_hints(item, item_by_id)
    if hints:
        suffix = f" [{' / '.join(hints)}]"
    return f"{index:>2}.   {item.item_id:<16s} {item.name}{suffix}"


def print_items(model: DashboardModel) -> None:
    print("硬件测试项目：", flush=True)
    for index, item in enumerate(model.items_by_group("hardware"), 1):
        print(format_item_line(index, item, model.item_by_id), flush=True)
    print("\n功能测试项目：", flush=True)
    for index, item in enumerate(model.items_by_group("function"), 1):
        print(format_item_line(index, item, model.item_by_id), flush=True)


def print_suite_yaml(suite: Dict[str, object]) -> None:
    print("\n=== Generated Suite YAML ===", flush=True)
    print(yaml.safe_dump(suite, allow_unicode=True, sort_keys=False), end="", flush=True)


def plan_step_rows(plan: DashboardPlan) -> List[tuple]:
    rows = []
    for index, step in enumerate(plan.suite.get("steps", []), 1):
        if "phase" in step:
            name = PHASE_DISPLAY_NAMES.get(str(step["phase"]), str(step["phase"]))
        else:
            name = str(step.get("name") or step.get("case") or "-")
        rows.append((str(index), name))
    return rows


def format_plan_items_table(model: DashboardModel, plan: DashboardPlan) -> str:
    rows = plan_step_rows(plan)
    headers = ("序号", "测试项目")
    if not rows:
        rows = [("-", "-")]
    widths = [
        max(display_width(headers[0]), *(display_width(row[0]) for row in rows)),
        max(display_width(headers[1]), *(display_width(row[1]) for row in rows)),
    ]

    def line(values: Sequence[str]) -> str:
        return "| " + " | ".join(
            pad_cells(value, width)
            for value, width in zip(values, widths)
        ) + " |"

    separator = "|-" + "-|-".join("-" * width for width in widths) + "-|"
    return "\n".join([line(headers), separator, *(line(row) for row in rows)])


def format_request_items(model: DashboardModel, request: DashboardRequest) -> str:
    if not request.item_ids:
        return "-"
    return ", ".join(
        f"{item_id}({model.item_by_id[item_id].name})"
        for item_id in request.item_ids
    )


def print_tab_summary(label: str, tabs: Sequence[Dict[str, object]]) -> None:
    if not tabs:
        print(f"{label}: none", flush=True)
        return
    print(f"{label}:", flush=True)
    for index, tab in enumerate(tabs, 1):
        delay_s = float(tab.get("delay_s", 0.0))
        delay = f", delay={delay_s:.1f}s" if delay_s > 0 else ""
        print(f"  {index}. {tab.get('title', 'tab')}{delay}", flush=True)


def print_plan_preview(
    model: DashboardModel,
    plan: DashboardPlan,
    request: DashboardRequest,
    output_dir: str,
    suite_path: str,
    bringup_tabs: Sequence[Dict[str, object]],
    runner_tabs: Sequence[Dict[str, object]],
) -> None:
    print("\n=== Dashboard 测试计划 ===", flush=True)
    print(f"environment: {plan.environment}", flush=True)
    print(f"profile: {plan.profile} ({plan.profile_reason})", flush=True)
    print(f"external_source: {plan.external_source_label}({plan.external_source})", flush=True)
    print(f"requested: {format_request_items(model, request)}", flush=True)
    print(f"output: {output_dir}", flush=True)
    print(f"suite: {suite_path}", flush=True)
    print("\n[测试项目]", flush=True)
    print(format_plan_items_table(model, plan), flush=True)
    print("\n[启动链路]", flush=True)
    print_tab_summary("bringup", bringup_tabs)
    print_tab_summary("runner", runner_tabs)

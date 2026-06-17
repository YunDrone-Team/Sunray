from typing import Dict, Sequence

import yaml

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.selection_ui import format_item_names
from sunray_test.dashboard.session import DashboardRequest
from sunray_test.dashboard.types import DashboardPlan


def print_suite_yaml(suite: Dict[str, object]) -> None:
    print("\n=== Generated Suite YAML ===", flush=True)
    print(yaml.safe_dump(suite, allow_unicode=True, sort_keys=False), end="", flush=True)


def prompt_plan_action(interactive: bool, dry_run: bool, yes: bool, no_prompt: bool) -> str:
    if dry_run or yes or no_prompt or not interactive:
        return "start"
    while True:
        raw = input(
            "\n确认生成 suite 并启动测试？[Y/s/w/p/r/n] "
            "(s=查看suite, w=只写suite, p=重印计划, r=返回重选): "
        ).strip().lower()
        if raw in {"", "y", "yes"}:
            return "start"
        if raw in {"s", "suite", "show"}:
            return "show_suite"
        if raw in {"w", "write", "write-suite", "suite-only"}:
            return "write_suite"
        if raw in {"p", "plan", "preview"}:
            return "preview"
        if raw in {"r", "retry", "reselect", "back", "b"}:
            return "reselect"
        if raw in {"n", "no"}:
            return "cancel"
        if raw in {"?", "h", "help"}:
            print("y=启动, s=查看suite, w=只写suite, p=重印计划, r=返回重选, n=取消", flush=True)
            continue
        print("请输入 y、s、w、p、r 或 n。", flush=True)


def print_tab_preview(label: str, tabs: Sequence[Dict[str, object]]) -> None:
    if not tabs:
        print(f"{label}: no launch tabs required", flush=True)
        return
    print(f"{label} tabs:", flush=True)
    for index, tab in enumerate(tabs, 1):
        delay_s = float(tab.get("delay_s", 0.0))
        delay_text = f" delay={delay_s:.1f}s" if delay_s > 0 else ""
        print(f"  {index}. {tab.get('title', 'tab')}{delay_text}: {tab['command']}", flush=True)


def format_request_source(model: DashboardModel, source: str) -> str:
    if source == "items":
        return "--items"
    if source == "manual":
        return "manual"
    return source or "-"


def print_suite_preview(
    model: DashboardModel,
    plan: DashboardPlan,
    request: DashboardRequest,
    uav_id: int,
    no_bringup: bool,
    output_dir: str,
    suite_path: str = "",
    suite_will_be_written: bool = True,
    suite_write_mode: str = "before launch",
    record_rosbag: bool = True,
    continue_on_failure: bool = False,
    terminal_status: str = "",
    runner_command: str = "",
    bringup_tabs: Sequence[Dict[str, object]] = (),
    runner_tabs: Sequence[Dict[str, object]] = (),
) -> None:
    print("\n=== Dashboard 测试计划 ===", flush=True)
    print_context(
        model,
        plan,
        request,
        uav_id,
        no_bringup,
        output_dir,
        suite_path,
        suite_will_be_written,
        suite_write_mode,
    )
    print_run_policy(model, plan, no_bringup, record_rosbag, continue_on_failure)
    if terminal_status:
        print(f"terminal: {terminal_status}", flush=True)
    if runner_command:
        print(f"runner command: {runner_command}", flush=True)

    print_scope(model, plan)
    print_steps(plan)
    print("\n[启动窗口]", flush=True)
    print_tab_preview("bringup", bringup_tabs)
    if suite_path:
        print_tab_preview("runner", runner_tabs)


def print_context(
    model: DashboardModel,
    plan: DashboardPlan,
    request: DashboardRequest,
    uav_id: int,
    no_bringup: bool,
    output_dir: str,
    suite_path: str,
    suite_will_be_written: bool,
    suite_write_mode: str,
) -> None:
    print("\n[运行上下文]", flush=True)
    print(f"source: {format_request_source(model, request.source)}", flush=True)
    environment_note = (
        "Gazebo 仿真，会拉起 sim bringup"
        if plan.environment == "sim"
        else "实机 exp，会拉起 exp bringup"
    )
    if no_bringup:
        environment_note += "；当前 --no-bringup 关闭启动链路"
    print(f"environment: {plan.environment} ({environment_note})", flush=True)
    print(f"uav_id: {uav_id}", flush=True)
    print(f"runtime profile: {plan.profile} ({plan.profile_reason})", flush=True)
    for name, value in plan.runtime_state.get("variables", {}).items():
        reason = plan.runtime_state.get("variable_reasons", {}).get(name, "default")
        print(f"{name}: {value} ({reason})", flush=True)
    print(f"external_source: {plan.external_source}", flush=True)
    print(f"output root: {output_dir}", flush=True)
    if suite_path:
        suite_state = build_suite_state(suite_will_be_written, suite_write_mode)
        print(f"generated suite: {suite_path} ({suite_state})", flush=True)


def build_suite_state(suite_will_be_written: bool, suite_write_mode: str) -> str:
    if suite_will_be_written:
        return f"will be written {suite_write_mode}"
    return "preview path, not written"


def print_run_policy(
    model: DashboardModel,
    plan: DashboardPlan,
    no_bringup: bool,
    record_rosbag: bool,
    continue_on_failure: bool,
) -> None:
    print("\n[运行策略]", flush=True)
    print(f"record rosbag: {'yes' if record_rosbag else 'no'}", flush=True)
    failure_policy = "continue on failure" if continue_on_failure else "stop on first failure"
    print(f"failure policy: {failure_policy}", flush=True)
    needs_airborne = any(
        model.item_by_id[item_id].requires_airborne
        for item_id in plan.selection.item_ids
    )
    airborne_text = "arm_and_takeoff required" if needs_airborne else "not required"
    print(f"airborne phases: {airborne_text}", flush=True)
    bringup_text = "manual/disabled" if no_bringup else "dashboard will open terminal tabs"
    print(f"bringup launch: {bringup_text}", flush=True)


def print_scope(model: DashboardModel, plan: DashboardPlan) -> None:
    print("\n[测试范围]", flush=True)
    print(f"requested: {format_item_names(model, plan.selection.requested_item_ids)}", flush=True)
    dependency_descriptions = model.dependency_descriptions(plan.selection.requested_item_ids)
    if dependency_descriptions:
        print("function dependencies:", flush=True)
        for description in dependency_descriptions:
            print(f"  - {description}", flush=True)
    if plan.selection.auto_added_item_ids:
        print(f"auto-added hardware: {format_item_names(model, plan.selection.auto_added_item_ids)}", flush=True)
    print(f"final: {format_item_names(model, plan.selection.item_ids)}", flush=True)


def print_steps(plan: DashboardPlan) -> None:
    print("\n[执行步骤]", flush=True)
    for index, step in enumerate(plan.suite["steps"], 1):
        if "phase" in step:
            print(f"  {index}. phase: {step['phase']}", flush=True)
        else:
            print(f"  {index}. case: {step['case']} ({step.get('name', step['case'])})", flush=True)

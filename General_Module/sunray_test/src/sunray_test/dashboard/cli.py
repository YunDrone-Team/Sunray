import argparse
import time
from dataclasses import replace

from sunray_test.dashboard.console import (
    format_plan_items_table,
    print_config_check,
    print_items,
    print_plan_preview,
    print_suite_yaml,
)
from sunray_test.dashboard.history import open_latest_report, print_history
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest, DashboardSession
from sunray_test.dashboard.suite_runtime import (
    build_bringup_tabs,
    build_runner_tabs,
    resolve_output_dir,
    run_generated_suite,
    runtime_suite_path,
    validate_dashboard_resources,
    validate_dashboard_suite_schema,
    write_runtime_suite,
)
from sunray_test.dashboard.terminal import (
    ensure_terminal_available,
    launch_terminal_window,
)
from sunray_test.dashboard.tui_app import run_dashboard_tui
from sunray_test.dashboard.tui_state import ACTION_CANCEL, ACTION_START
from sunray_test.dashboard.types import DASHBOARD_UAV_ID, DashboardPlan


LOG_PREFIX = "[sunray_test_dashboard]"


def print_runner_started(model: DashboardModel, plan: DashboardPlan) -> None:
    print(f"\n{LOG_PREFIX} 测试执行已在新的 Sunray Test Runner 终端启动。", flush=True)
    print(f"{LOG_PREFIX} 测试项目:", flush=True)
    print(format_plan_items_table(model, plan), flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="Dashboard entry for selectable Sunray test items")
    parser.add_argument(
        "--config",
        default="dashboard",
        help="Dashboard config name under config/dashboard or an absolute yaml path",
    )
    parser.add_argument("--run-suite", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--sim", action="store_true", help="Use simulation environment and launch sim bringup")
    parser.add_argument(
        "--profile",
        choices=("sunray150_basic", "sunray150_lidar"),
        default="",
        help="Override vehicle profile; default is inferred from selected test items",
    )
    parser.add_argument("--platform", default="", help=argparse.SUPPRESS)
    parser.add_argument("--environment", default="", help=argparse.SUPPRESS)
    parser.add_argument("--suite", default="", help=argparse.SUPPRESS)
    parser.add_argument("--suite-file", default="", help=argparse.SUPPRESS)
    parser.add_argument("--external-source", type=int, default=None)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--sn", default="")
    parser.add_argument("--tester", default="")
    parser.add_argument("--no-prompt", action="store_true", help="Do not ask for SN/tester")
    parser.add_argument("--items", default="", help="Comma separated item ids, e.g. front_camera,battery,hover")
    parser.add_argument("--list", action="store_true", help="List dashboard test items and exit")
    parser.add_argument("--check-config", action="store_true", help="Validate dashboard config and print a summary")
    parser.add_argument(
        "--history",
        action="store_true",
        help="List recent test runs under the output directory and exit",
    )
    parser.add_argument("--history-limit", type=int, default=8, help="Number of recent runs to show with --history")
    parser.add_argument(
        "--open-latest-report",
        action="store_true",
        help="Open the latest dashboard report under the output directory",
    )
    action_group = parser.add_mutually_exclusive_group()
    action_group.add_argument(
        "--dry-run",
        action="store_true",
        help="Print generated suite without running ROS tests",
    )
    action_group.add_argument(
        "--show-suite",
        action="store_true",
        help="Print the generated runtime suite YAML and exit",
    )
    parser.add_argument("--no-bringup", action="store_true", help="Do not launch sim/exp bringup terminals")
    parser.add_argument("--record-rosbag", action="store_true", default=True)
    parser.add_argument("--no-record-rosbag", action="store_false", dest="record_rosbag")
    parser.add_argument("--continue-on-failure", action="store_true")
    return parser.parse_args()


def has_direct_action(args) -> bool:
    return bool(
        args.items
        or args.list
        or args.check_config
        or args.history
        or args.open_latest_report
        or args.dry_run
        or args.show_suite
    )


def build_items_request(model: DashboardModel, args) -> DashboardRequest:
    if not args.items:
        raise SystemExit("当前不是交互 TUI，请使用 --items 指定测试项目")
    return DashboardRequest(
        item_ids=model.parse_item_ids(args.items),
        source="items",
        profile_override=args.profile,
    )


def build_validated_plan(
    model: DashboardModel,
    args,
    request: DashboardRequest,
    environment: str,
) -> DashboardPlan:
    external_source_override = (
        request.external_source_override
        if request.external_source_override is not None
        else args.external_source
    )
    plan = model.build_plan(
        requested_item_ids=request.item_ids,
        environment=environment,
        uav_id=args.uav_id,
        external_source_override=external_source_override,
        record_rosbag=args.record_rosbag,
        continue_on_failure=args.continue_on_failure,
        param_overrides=request.param_overrides or {},
        profile_override=request.profile_override or args.profile,
    )
    validate_dashboard_resources(
        plan.profile,
        plan.environment,
        plan.suite,
        args.uav_id,
        plan.external_source,
    )
    validation_warning = validate_dashboard_suite_schema(
        plan.profile,
        plan.environment,
        plan.suite,
        args.uav_id,
        plan.external_source,
    )
    plan = replace(plan, validation_warning=validation_warning)
    if validation_warning and has_direct_action(args):
        print(f"[sunray_test_dashboard] {validation_warning}", flush=True)
    return plan


def maybe_launch_bringup(model: DashboardModel, args, plan: DashboardPlan) -> None:
    if args.no_bringup:
        return
    tabs = build_bringup_tabs(model, plan)
    if not tabs:
        return
    title = "Sunray Dashboard Sim Bringup" if plan.environment == "sim" else "Sunray Dashboard Exp Bringup"
    launch_terminal_window(title, tabs)
    time.sleep(0.3)


def launch_runner(
    model: DashboardModel,
    args,
    plan: DashboardPlan,
    suite_path: str,
    output_dir: str,
) -> None:
    tabs = build_runner_tabs(model, args, plan, suite_path, output_dir)
    if not tabs:
        raise SystemExit("dashboard runner config did not produce any runner tabs")
    launch_terminal_window("Sunray Test Runner", tabs)


def handle_plan_actions(
    model: DashboardModel,
    args,
    plan: DashboardPlan,
    request: DashboardRequest,
    suite_path: str,
    output_dir: str,
) -> str:
    if args.show_suite:
        print_suite_yaml(plan.suite)
        print(f"\n{LOG_PREFIX} show-suite only, not launching bringup or runner", flush=True)
        return "done"
    if args.dry_run:
        print_plan_preview(
            model=model,
            plan=plan,
            request=request,
            output_dir=output_dir,
            suite_path=suite_path,
            bringup_tabs=build_bringup_tabs(model, plan),
            runner_tabs=build_runner_tabs(model, args, plan, suite_path, output_dir),
        )
        print(f"\n{LOG_PREFIX} dry-run only, not launching bringup or runner", flush=True)
        return "done"
    return "start"


def initialize_dashboard(args):
    output_dir = resolve_output_dir(args)
    if args.history:
        print_history(output_dir, args.history_limit)
        return None, None, output_dir
    if args.open_latest_report:
        open_latest_report(output_dir)
        return None, None, output_dir

    model = DashboardModel.load(args.config)
    if args.check_config:
        print_config_check(model)
        return model, None, output_dir
    if args.list:
        print_items(model)
        return model, None, output_dir

    environment = "sim" if args.sim else "exp"
    args.uav_id = DASHBOARD_UAV_ID
    session = DashboardSession(
        model=model,
        environment=environment,
        output_dir=output_dir,
        uav_id=args.uav_id,
    )
    return model, session, output_dir


def main():
    args = parse_args()
    if args.run_suite:
        args.uav_id = DASHBOARD_UAV_ID
        return run_generated_suite(args)

    model, session, output_dir = initialize_dashboard(args)
    if model is None or session is None:
        return 0

    environment = session.environment
    if not has_direct_action(args):
        tui_result = run_dashboard_tui(
            session=session,
            args=args,
            build_plan_callback=lambda request: build_validated_plan(model, args, request, environment),
            build_suite_path_callback=lambda plan: runtime_suite_path(plan.suite, output_dir),
        )
        if tui_result is not None:
            if tui_result.action == ACTION_CANCEL:
                print(f"{LOG_PREFIX} 已退出。", flush=True)
                return 0
            plan = tui_result.plan
            request = tui_result.request
            suite_path = tui_result.suite_path
            if tui_result.action == ACTION_START:
                ensure_terminal_available()
                write_runtime_suite(plan.suite, suite_path)
                maybe_launch_bringup(model, args, plan)
                launch_runner(model, args, plan, suite_path, output_dir)
                print_runner_started(model, plan)
                return 0

        raise SystemExit("当前终端不支持全屏 TUI，请使用 --items 指定测试项目")

    request = build_items_request(model, args)
    plan = build_validated_plan(model, args, request, environment)
    suite_path = runtime_suite_path(plan.suite, output_dir)
    result = handle_plan_actions(
        model,
        args,
        plan,
        request,
        suite_path,
        output_dir,
    )
    if result == "done":
        return 0

    ensure_terminal_available()
    write_runtime_suite(plan.suite, suite_path)
    maybe_launch_bringup(model, args, plan)
    launch_runner(model, args, plan, suite_path, output_dir)
    print_runner_started(model, plan)
    return 0

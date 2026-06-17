import argparse
import time

from sunray_test.dashboard.history import open_latest_report, print_history
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest, DashboardSession
from sunray_test.dashboard.suite_runtime import (
    build_bringup_tabs,
    build_manual_runner_command,
    build_runner_tabs,
    resolve_output_dir,
    run_generated_suite,
    runtime_suite_path,
    validate_dashboard_resources,
    validate_dashboard_suite_schema,
    write_runtime_suite,
    write_suite_only_and_print,
)
from sunray_test.dashboard.terminal import (
    ensure_terminal_available,
    format_terminal_status,
    launch_terminal_window,
)
from sunray_test.dashboard.types import DASHBOARD_UAV_ID, DashboardPlan
from sunray_test.dashboard.ui import (
    print_config_check,
    print_items,
    print_suite_preview,
    print_suite_yaml,
    prompt_main_menu,
    prompt_plan_action,
    select_dashboard_interactively,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Dashboard entry for selectable Sunray test items")
    parser.add_argument(
        "--config",
        default="dashboard",
        help="Dashboard config name under config/dashboard or an absolute yaml path",
    )
    parser.add_argument("--run-suite", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--sim", action="store_true", help="Use simulation environment and launch sim bringup")
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
    action_group.add_argument("--dry-run", action="store_true", help="Print generated suite without running ROS tests")
    action_group.add_argument(
        "--show-suite",
        action="store_true",
        help="Print the generated runtime suite YAML and exit",
    )
    action_group.add_argument(
        "--write-suite-only",
        action="store_true",
        help="Write the generated runtime suite YAML and exit without launching ROS tests",
    )
    parser.add_argument("--no-bringup", action="store_true", help="Do not launch sim/exp bringup terminals")
    parser.add_argument("--record-rosbag", action="store_true", default=True)
    parser.add_argument("--no-record-rosbag", action="store_false", dest="record_rosbag")
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip dashboard launch confirmation")
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
        or args.write_suite_only
    )


def build_request(model: DashboardModel, args, session: DashboardSession) -> DashboardRequest:
    if args.items:
        return DashboardRequest(item_ids=model.parse_item_ids(args.items), source="items")
    return select_dashboard_interactively(session, no_bringup=args.no_bringup)


def print_plan_preview(
    model: DashboardModel,
    args,
    plan: DashboardPlan,
    request: DashboardRequest,
    suite_path: str,
    output_dir: str,
) -> None:
    runner_command = build_manual_runner_command(model, args, plan, suite_path, output_dir)
    print_suite_preview(
        model=model,
        plan=plan,
        request=request,
        uav_id=args.uav_id,
        no_bringup=args.no_bringup,
        output_dir=output_dir,
        suite_path=suite_path,
        suite_will_be_written=not (args.dry_run or args.show_suite),
        suite_write_mode="before exit" if args.write_suite_only else "before launch",
        record_rosbag=args.record_rosbag,
        continue_on_failure=args.continue_on_failure,
        terminal_status=format_terminal_status(args),
        runner_command=runner_command,
        bringup_tabs=build_bringup_tabs(model, plan),
        runner_tabs=build_runner_tabs(model, args, plan, suite_path, output_dir),
    )


def build_validated_plan(
    model: DashboardModel,
    args,
    request: DashboardRequest,
    environment: str,
) -> DashboardPlan:
    plan = model.build_plan(
        requested_item_ids=request.item_ids,
        environment=environment,
        uav_id=args.uav_id,
        external_source_override=args.external_source,
        record_rosbag=args.record_rosbag,
        continue_on_failure=args.continue_on_failure,
    )
    validate_dashboard_resources(
        plan.profile,
        plan.environment,
        plan.suite,
        args.uav_id,
        plan.external_source,
    )
    validate_dashboard_suite_schema(
        plan.profile,
        plan.environment,
        plan.suite,
        args.uav_id,
        plan.external_source,
    )
    return plan


def prompt_for_plan_action(args, interactive: bool, preview_callback) -> str:
    while True:
        action = prompt_plan_action(
            interactive=interactive,
            dry_run=args.dry_run,
            yes=args.yes,
            no_prompt=args.no_prompt,
        )
        if action == "show_suite":
            preview_callback(show_suite=True)
            continue
        if action == "preview":
            preview_callback(show_suite=False)
            continue
        return action


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
    interactive: bool,
) -> str:
    def preview_callback(show_suite: bool) -> None:
        if show_suite:
            print_suite_yaml(plan.suite)
        else:
            print_plan_preview(model, args, plan, request, suite_path, output_dir)

    print_plan_preview(model, args, plan, request, suite_path, output_dir)
    if args.show_suite:
        print_suite_yaml(plan.suite)
        print("\n[dashboard] show-suite only, not launching bringup or runner", flush=True)
        return "done"
    if args.write_suite_only:
        write_suite_only_and_print(model, args, plan, suite_path, output_dir)
        return "done"
    if args.dry_run:
        print("\n[dashboard] dry-run only, not launching bringup or runner", flush=True)
        return "done"

    action = prompt_for_plan_action(args, interactive, preview_callback)
    if action == "write_suite":
        write_suite_only_and_print(model, args, plan, suite_path, output_dir)
        return "done"
    if action == "reselect":
        print("\n[dashboard] 返回测试项目选择。", flush=True)
        return "reselect"
    if action == "cancel":
        print("\n[dashboard] 已取消，未生成 suite，未启动测试。", flush=True)
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

    if not has_direct_action(args):
        menu_action = prompt_main_menu(session, args.history_limit)
        if menu_action == "exit":
            print("[dashboard] 已退出。", flush=True)
            return 0

    environment = session.environment
    interactive = not bool(args.items)
    while True:
        request = build_request(model, args, session)
        plan = build_validated_plan(model, args, request, environment)
        suite_path = runtime_suite_path(plan.suite, output_dir)
        result = handle_plan_actions(
            model,
            args,
            plan,
            request,
            suite_path,
            output_dir,
            interactive,
        )
        if result == "reselect":
            continue
        if result == "done":
            return 0
        break

    ensure_terminal_available()
    write_runtime_suite(plan.suite, suite_path)
    maybe_launch_bringup(model, args, plan)
    launch_runner(model, args, plan, suite_path, output_dir)
    print("\n[dashboard] 测试执行已在新的 Sunray Test Runner 终端启动。", flush=True)
    return 0

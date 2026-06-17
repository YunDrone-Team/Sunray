import sys

from sunray_test.dashboard.history import open_latest_report
from sunray_test.dashboard.history_ui import browse_history_interactively
from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.selection_ui import print_items
from sunray_test.dashboard.session import DashboardSession


def print_config_check(model: DashboardModel) -> None:
    summary = model.config_summary()
    print("\n=== Dashboard Config Check ===", flush=True)
    print("status: ok", flush=True)
    print(f"name: {summary['name']}", flush=True)
    items = summary["test_items"]
    print(
        f"items: total={items['total']}, hardware={items['hardware']}, function={items['function']}",
        flush=True,
    )
    print(f"default_items: {', '.join(summary['default_items']) or '-'}", flush=True)
    profiles = summary["runtime_profiles"]
    print(f"runtime_profiles: default={profiles['default']}, rules={profiles['rules']}", flush=True)
    print(
        "bringup_tabs: "
        + ", ".join(f"{environment}={count}" for environment, count in summary["bringup_tabs"].items()),
        flush=True,
    )
    print(
        "runner_tabs: "
        + ", ".join(f"{environment}={count}" for environment, count in summary["runner_tabs"].items()),
        flush=True,
    )
    print(f"variables: {', '.join(summary['variables']) or '-'}", flush=True)


def prompt_main_menu(session: DashboardSession, history_limit: int) -> str:
    if not sys.stdin.isatty():
        raise SystemExit("当前不是交互终端，请使用 --items、--list 或 --history")
    while True:
        print("\n=== Sunray Test Dashboard ===", flush=True)
        print("1. 启动测试", flush=True)
        print("2. 最近结果", flush=True)
        print("3. 测试项目", flush=True)
        print("4. 配置检查", flush=True)
        print("l. 打开最近报告", flush=True)
        print("q. 退出", flush=True)
        raw = input("请选择 [1]: ").strip().lower()
        if raw in {"", "1", "start", "run"}:
            return "start"
        if raw in {"2", "history", "h"}:
            browse_history_interactively(session.output_dir, history_limit)
            continue
        if raw in {"3", "list", "items", "i"}:
            print_items(session.model)
            continue
        if raw in {"4", "check", "config", "c"}:
            print_config_check(session.model)
            continue
        if raw in {"l", "latest", "report", "open", "o"}:
            open_latest_report(session.output_dir)
            continue
        if raw in {"q", "quit", "exit"}:
            return "exit"
        print("请输入 1、2、3、4、l 或 q。", flush=True)

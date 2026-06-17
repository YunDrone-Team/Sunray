import sys
from typing import Dict

from sunray_test.dashboard.history import (
    collect_history_runs,
    open_path,
    print_history,
    print_history_detail,
    print_history_list,
)


def browse_history_detail_actions(run: Dict[str, object]) -> None:
    while True:
        print_history_detail(run)
        print("\n操作: o=打开报告, d=打开目录, r=打开result, 回车/q=返回列表", flush=True)
        raw = input("请选择: ").strip().lower()
        if raw in {"", "q", "quit", "back", "b"}:
            return
        if raw in {"o", "open", "report"}:
            open_path(run.get("report_path", ""), "报告")
            continue
        if raw in {"d", "dir", "directory", "folder"}:
            open_path(run.get("run_dir", ""), "运行目录")
            continue
        if raw in {"r", "result", "json"}:
            open_path(run.get("result_path", ""), "result JSON")
            continue
        print("请输入 o、d、r 或回车。", flush=True)


def browse_history_interactively(output_dir: str, limit: int) -> None:
    if not sys.stdin.isatty():
        print_history(output_dir, limit)
        return
    runs = collect_history_runs(output_dir)
    if not runs:
        print_history_list(runs, output_dir, limit)
        return
    while True:
        visible_runs = runs[: max(limit, 1)]
        print_history_list(visible_runs, output_dir, limit)
        raw = input("\n输入编号查看详情，回车/q 返回: ").strip().lower()
        if raw in {"", "q", "quit", "back", "b"}:
            return
        if raw.isdigit():
            index = int(raw)
            if 1 <= index <= len(visible_runs):
                browse_history_detail_actions(visible_runs[index - 1])
                continue
        print("无效选择。", flush=True)

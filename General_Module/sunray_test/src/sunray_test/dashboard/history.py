import json
import os
import shutil
import subprocess
from typing import Any, Dict, List, Sequence


def load_json_file(path: str) -> Dict[str, Any]:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        return {"_error": str(exc)}
    return data if isinstance(data, dict) else {"_error": "top-level JSON is not an object"}


def format_score(score) -> str:
    if isinstance(score, (int, float)):
        return f"{score:.1f}"
    return "-"


def format_cases(cases: Sequence[Dict[str, Any]]) -> str:
    if not cases:
        return "-"
    parts: List[str] = []
    for case in cases:
        if not isinstance(case, dict):
            continue
        name = str(case.get("case") or case.get("name") or "?")
        result = str(case.get("result") or "?")
        parts.append(f"{name}:{result}")
    return ", ".join(parts) if parts else "-"


def resolve_artifact_path(run_dir: str, relative_path: str) -> str:
    if not relative_path:
        return ""
    return relative_path if os.path.isabs(relative_path) else os.path.join(run_dir, relative_path)


def extract_overall_score(data: Dict[str, Any]) -> Dict[str, Any]:
    candidates = []
    analysis = data.get("analysis", {}) if isinstance(data.get("analysis"), dict) else {}
    candidates.append(analysis.get("scores", {}) if isinstance(analysis.get("scores"), dict) else {})
    flight_metrics = data.get("flight_metrics", {}) if isinstance(data.get("flight_metrics"), dict) else {}
    candidates.append(flight_metrics.get("scores", {}) if isinstance(flight_metrics.get("scores"), dict) else {})
    for scores in candidates:
        overall = scores.get("overall", {}) if isinstance(scores, dict) else {}
        if isinstance(overall, dict) and overall:
            return overall
    return {}


def format_summary_status(summary: Dict[str, Any], json_error: str) -> str:
    if json_error:
        return "invalid"
    status = str(summary.get("status") or "")
    counts = []
    for key in ("pass", "fail", "error", "unsupported"):
        value = summary.get(key)
        if isinstance(value, int) and value:
            counts.append(f"{key}={value}")
    if status and counts:
        return f"{status} ({', '.join(counts)})"
    if status:
        return status
    return ", ".join(counts) if counts else "-"


def collect_history_runs(output_dir: str) -> List[Dict[str, Any]]:
    runs: List[Dict[str, Any]] = []
    if not os.path.isdir(output_dir):
        return runs
    for entry in os.listdir(output_dir):
        run_dir = os.path.join(output_dir, entry)
        if not os.path.isdir(run_dir):
            continue
        result_path = os.path.join(run_dir, "test_result.json")
        if not os.path.isfile(result_path):
            continue
        data = load_json_file(result_path)
        run_info = data.get("run_info", {}) if isinstance(data.get("run_info"), dict) else {}
        summary = data.get("summary", {}) if isinstance(data.get("summary"), dict) else {}
        overall = extract_overall_score(data)
        report_path = os.path.join(run_dir, "report.html")
        json_error = str(data.get("_error", ""))
        artifacts = data.get("artifacts", {}) if isinstance(data.get("artifacts"), dict) else {}
        bag_file = resolve_artifact_path(run_dir, str(artifacts.get("bag_file") or ""))
        event_log = resolve_artifact_path(run_dir, str(artifacts.get("event_log_jsonl") or "event_log.jsonl"))
        flight_metrics = data.get("flight_metrics", {}) if isinstance(data.get("flight_metrics"), dict) else {}
        runs.append(
            {
                "name": entry,
                "run_dir": run_dir,
                "result_path": result_path,
                "report_path": report_path if os.path.isfile(report_path) else "",
                "event_log_path": event_log if os.path.isfile(event_log) else "",
                "bag_path": bag_file if bag_file and os.path.isfile(bag_file) else "",
                "started_at": str(run_info.get("started_at") or ""),
                "finished_at": str(run_info.get("finished_at") or ""),
                "platform": str(run_info.get("platform") or "-"),
                "environment": str(run_info.get("environment") or "-"),
                "suite": str(run_info.get("suite") or "-"),
                "uav_id": run_info.get("uav_id", "-"),
                "sn": str(run_info.get("sn") or "-"),
                "tester": str(run_info.get("tester") or "-"),
                "status": format_summary_status(summary, json_error),
                "interrupted": bool(run_info.get("interrupted", False)),
                "interruption_reason": str(run_info.get("interruption_reason") or ""),
                "score": overall.get("score"),
                "grade": str(overall.get("grade") or "-"),
                "cases": data.get("cases", []) if isinstance(data.get("cases"), list) else [],
                "scores": flight_metrics.get("scores", {}) if isinstance(flight_metrics.get("scores"), dict) else {},
                "flight_errors": (
                    flight_metrics.get("errors", [])
                    if isinstance(flight_metrics.get("errors"), list)
                    else []
                ),
                "error": json_error,
            }
        )
    return sorted(runs, key=lambda run: (run["started_at"], run["name"]), reverse=True)


def latest_history_run(output_dir: str) -> Dict[str, Any]:
    runs = collect_history_runs(output_dir)
    return runs[0] if runs else {}


def print_history_list(runs: Sequence[Dict[str, Any]], output_dir: str, limit: int) -> None:
    print("\n=== Sunray Test History ===", flush=True)
    print(f"output root: {output_dir}", flush=True)
    if not runs:
        print("未找到测试结果。", flush=True)
        return
    for index, run in enumerate(runs[: max(limit, 1)], 1):
        status = run["status"]
        if run["interrupted"]:
            status = f"{status}, interrupted"
        score_text = format_score(run["score"])
        title = (
            f"{index}. {run['name']} | {run['environment']} | {run['platform']} | "
            f"{run['suite']} | {status} | score={score_text} | grade={run['grade']}"
        )
        print(title, flush=True)
        if run["started_at"] or run["finished_at"]:
            print(f"   time: {run['started_at']} -> {run['finished_at']}", flush=True)
        print(f"   cases: {format_cases(run['cases'])}", flush=True)
        if run["report_path"]:
            print(f"   report: {run['report_path']}", flush=True)
        print(f"   result: {run['result_path']}", flush=True)
        if run["error"]:
            print(f"   error: {run['error']}", flush=True)


def print_history(output_dir: str, limit: int) -> None:
    print_history_list(collect_history_runs(output_dir), output_dir, limit)


def open_latest_report(output_dir: str) -> bool:
    run = latest_history_run(output_dir)
    if not run:
        print("未找到测试结果。", flush=True)
        return False
    report_path = str(run.get("report_path") or "")
    if not report_path:
        print(f"最近一次测试没有 report.html: {run.get('run_dir', output_dir)}", flush=True)
        return False
    open_path(report_path, "最近报告")
    return True


def print_history_detail(run: Dict[str, Any]) -> None:
    status = run["status"]
    if run["interrupted"]:
        status = f"{status}, interrupted"
    print(f"\n=== Test Run Detail: {run['name']} ===", flush=True)
    print(f"environment: {run['environment']}", flush=True)
    print(f"platform: {run['platform']}", flush=True)
    print(f"suite: {run['suite']}", flush=True)
    print(f"uav_id: {run['uav_id']}", flush=True)
    print(f"sn/tester: {run['sn']} / {run['tester']}", flush=True)
    print(f"time: {run['started_at']} -> {run['finished_at']}", flush=True)
    print(f"status: {status}", flush=True)
    print(f"overall: score={format_score(run['score'])}, grade={run['grade']}", flush=True)
    if run["interruption_reason"]:
        print(f"interruption: {run['interruption_reason']}", flush=True)

    print("cases:", flush=True)
    if run["cases"]:
        for index, case in enumerate(run["cases"], 1):
            name = str(case.get("name") or case.get("case") or case.get("id") or "?")
            case_id = str(case.get("id") or case.get("case") or "-")
            result = str(case.get("result") or "?")
            detail = str(case.get("detail") or "").strip()
            suffix = f" - {detail}" if detail else ""
            print(f"  {index}. {name} [{case_id}] {result}{suffix}", flush=True)
    else:
        print("  none", flush=True)

    scores = run.get("scores", {}) if isinstance(run.get("scores"), dict) else {}
    score_lines: List[str] = []
    for name, score_data in scores.items():
        if name in {"grade_thresholds", "overall"} or not isinstance(score_data, dict):
            continue
        score_lines.append(
            f"{name}: score={format_score(score_data.get('score'))}, grade={score_data.get('grade', '-')}"
        )
    if score_lines:
        print("scores:", flush=True)
        for line in score_lines:
            print(f"  {line}", flush=True)

    flight_errors = run.get("flight_errors", [])
    if flight_errors:
        print("flight metric notes:", flush=True)
        for error in flight_errors:
            print(f"  - {error}", flush=True)

    print("artifacts:", flush=True)
    print(f"  run_dir: {run['run_dir']}", flush=True)
    print(f"  result: {run['result_path']}", flush=True)
    if run["report_path"]:
        print(f"  report: {run['report_path']}", flush=True)
    if run["event_log_path"]:
        print(f"  event_log: {run['event_log_path']}", flush=True)
    if run["bag_path"]:
        print(f"  bag: {run['bag_path']}", flush=True)
    if run["error"]:
        print(f"error: {run['error']}", flush=True)


def open_path(path: str, label: str) -> None:
    if not path:
        print(f"{label} 不存在。", flush=True)
        return
    if not os.path.exists(path):
        print(f"{label} 不存在: {path}", flush=True)
        return
    opener = shutil.which("xdg-open") or shutil.which("gio")
    if not opener:
        print(f"未找到 xdg-open/gio，无法自动打开 {label}: {path}", flush=True)
        return
    command = [opener, path] if os.path.basename(opener) == "xdg-open" else [opener, "open", path]
    try:
        subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as exc:
        print(f"打开 {label} 失败: {exc}", flush=True)
        return
    print(f"已尝试打开 {label}: {path}", flush=True)

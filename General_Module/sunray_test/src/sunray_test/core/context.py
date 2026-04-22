import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List


@dataclass
class RunPaths:
    package_root: str
    workspace_root: str
    output_root: str
    run_dir: str
    result_json: str
    report_html: str
    event_log_jsonl: str


@dataclass
class RunContext:
    package_root: str
    workspace_root: str
    run_paths: RunPaths
    platform_name: str
    environment_name: str
    suite_name: str
    uav_id: int
    uav_name: str
    platform: Dict[str, Any]
    environment: Dict[str, Any]
    suite: Dict[str, Any]
    resolved_topics: Dict[str, str]
    recording_topics: List[str]
    missions: Dict[str, Any]
    defaults: Dict[str, Any]
    report: Dict[str, Any]
    metadata: Dict[str, Any] = field(default_factory=dict)
    artifacts: Dict[str, Any] = field(default_factory=dict)


def package_root_from_file() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def workspace_root_from_package(package_root: str) -> str:
    return os.path.abspath(os.path.join(package_root, "..", ".."))


def create_run_paths(package_root: str, workspace_root: str, output_root: str) -> RunPaths:
    expanded_output_root = os.path.expanduser(output_root)
    os.makedirs(expanded_output_root, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(expanded_output_root, timestamp)
    os.makedirs(run_dir, exist_ok=True)
    return RunPaths(
        package_root=package_root,
        workspace_root=workspace_root,
        output_root=expanded_output_root,
        run_dir=run_dir,
        result_json=os.path.join(run_dir, "test_result.json"),
        report_html=os.path.join(run_dir, "report.html"),
        event_log_jsonl=os.path.join(run_dir, "event_log.jsonl"),
    )

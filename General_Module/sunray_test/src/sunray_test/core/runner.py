import json
import os
import signal
import traceback
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict

import rospy

from sunray_test.adapters.uav_adapter import UAVAdapter
from sunray_test.capabilities.event_logger import EventLogger
from sunray_test.capabilities.rosbag_recorder import RosbagRecorder
from sunray_test.cases.base import CaseExecutionContext
from sunray_test.cases.registry import get_case_class
from sunray_test.core.context import RunContext, create_run_paths, package_root_from_file, workspace_root_from_package
from sunray_test.core.result_model import CaseResult, ResultStore
from sunray_test.core.suite_loader import load_config_triplet
from sunray_test.reports.flight_metrics import enrich_report_payload
from sunray_test.phases.registry import run_phase
from sunray_test.reports.html_renderer import render_html


@dataclass
class RunnerArgs:
    platform: str
    environment: str
    suite: str
    uav_id: int
    output_dir: str = ""
    sn: str = ""
    tester: str = ""
    prompt_metadata: bool = True


class TestRunner:
    def __init__(self, args: RunnerArgs) -> None:
        self.args = args
        self.package_root = package_root_from_file()
        self.workspace_root = workspace_root_from_package(self.package_root)
        self.loaded = load_config_triplet(
            package_root=self.package_root,
            platform_name=args.platform,
            environment_name=args.environment,
            suite_name=args.suite,
            uav_id=args.uav_id,
        )

        output_root = args.output_dir or os.path.join(
            self.workspace_root,
            "tests",
            "output",
        )
        self.run_paths = create_run_paths(self.package_root, self.workspace_root, output_root)

        uav_name = f"/uav{args.uav_id}"
        self.context = RunContext(
            package_root=self.package_root,
            workspace_root=self.workspace_root,
            run_paths=self.run_paths,
            platform_name=args.platform,
            environment_name=args.environment,
            suite_name=args.suite,
            uav_id=args.uav_id,
            uav_name=uav_name,
            platform=self.loaded["platform"],
            environment=self.loaded["environment"],
            suite=self.loaded["suite"],
            resolved_topics=self.loaded["topics"],
            recording_topics=self.loaded["recording"].get("topic_templates", []),
            missions=self.loaded["missions"],
            defaults=self.loaded["defaults"],
            report=self.loaded["report"],
        )
        self.context.metadata = self._resolve_metadata()
        self.result_store = ResultStore()
        self.event_logger = EventLogger()
        self.rosbag = RosbagRecorder()
        self.state = "precheck"
        self.vehicle = None
        self.interrupted = False
        self.interruption_reason = ""
        self.exit_code_override = None

    def _resolve_metadata(self) -> Dict[str, Any]:
        metadata = {"sn": self.args.sn, "tester": self.args.tester}
        if self.args.prompt_metadata:
            if not metadata["sn"]:
                metadata["sn"] = input("请输入设备SN: ").strip()
            if not metadata["tester"]:
                metadata["tester"] = input("请输入测试人员: ").strip()
        return metadata

    def _make_case(self, step: Dict[str, Any]):
        case_type = step["type"]
        case_cls = get_case_class(case_type)
        execution_context = CaseExecutionContext(
            case_id=step["case"],
            name=step.get("name", step["case"]),
            category=step.get("category", case_cls.category),
            params=step.get("params", {}),
            required_state=step.get("required_state", case_cls.default_required_state),
            resulting_state=step.get("resulting_state", case_cls.default_resulting_state),
        )
        return case_cls(execution_context)

    def _ensure_state(self, required_state: str) -> None:
        if required_state and self.state != required_state:
            raise RuntimeError(f"invalid test order: current state={self.state}, required={required_state}")

    @staticmethod
    def _print_banner(message: str) -> None:
        line = "=" * 72
        print(f"\n{line}\n{message}\n{line}", flush=True)

    def _write_outputs(self) -> None:
        payload = {
            "run_info": {
                "platform": self.context.platform_name,
                "environment": self.context.environment_name,
                "suite": self.context.suite_name,
                "uav_id": self.context.uav_id,
                "uav_name": self.context.uav_name,
                "sn": self.context.metadata.get("sn", ""),
                "tester": self.context.metadata.get("tester", ""),
                "run_dir": self.context.run_paths.run_dir,
                "report_title": self.context.report.get("title", f"{self.context.platform_name} 测试报告"),
                "interrupted": self.interrupted,
                "interruption_reason": self.interruption_reason,
            },
            "artifacts": {
                "run_dir": self.context.run_paths.run_dir,
                "event_log_jsonl": "event_log.jsonl",
                "report_html": "report.html",
                "bag_file": os.path.relpath(
                    self.context.artifacts["bag_file"], self.context.run_paths.run_dir
                ) if self.context.artifacts.get("bag_file") else "",
                "recording_topics": self.context.recording_topics,
            },
            "config": {
                "platform": self.context.platform_name,
                "environment": self.context.environment_name,
                "suite": self.context.suite_name,
                "defaults": self.context.defaults,
                "topics": self.context.resolved_topics,
                "missions": self.context.missions,
            },
        }
        self.result_store.write_json(self.context.run_paths.result_json, payload)
        self.event_logger.write_jsonl(self.context.run_paths.event_log_jsonl)
        with open(self.context.run_paths.result_json, "r", encoding="utf-8") as handle:
            report_payload = json.load(handle)
        report_payload = enrich_report_payload(report_payload, self.workspace_root)
        with open(self.context.run_paths.result_json, "w", encoding="utf-8") as handle:
            json.dump(report_payload, handle, ensure_ascii=False, indent=2)
        with open(self.context.run_paths.report_html, "w", encoding="utf-8") as handle:
            handle.write(render_html(report_payload))

    @staticmethod
    def _handle_sigint(signum, frame) -> None:
        raise KeyboardInterrupt("SIGINT received")

    def _should_emergency_land(self) -> bool:
        if self.vehicle is None:
            return False
        if self.state == "airborne":
            return True
        try:
            return bool(self.vehicle.state.armed)
        except Exception:
            return False

    def _emergency_land_after_interrupt(self) -> None:
        if not self._should_emergency_land():
            return

        self._print_banner("[INTERRUPT] emergency_land | current_state=%s" % self.state)
        self.event_logger.log("interrupt_emergency_land_start", f"state={self.state}")
        state_before = self.state
        try:
            self.state = run_phase("land", self.context, self.vehicle, self.event_logger)
            self.result_store.record_phase("interrupt_emergency_land", state_before, self.state)
            self.event_logger.log("interrupt_emergency_land_end", f"state={self.state}")
            rospy.logwarn("已执行紧急降落")
        except Exception as exc:
            self.event_logger.log("interrupt_emergency_land_fail", str(exc))
            rospy.logerr("紧急降落失败: %s", exc)

    def _mark_remaining_cases_failed(self, start_index: int, reason: str) -> None:
        recorded_case_ids = {item.case_id for item in self.result_store.case_results}
        now = datetime.now().isoformat(timespec="seconds")

        for step in self.context.suite["steps"][max(start_index, 0):]:
            if "case" not in step:
                continue
            case_id = step["case"]
            if case_id in recorded_case_ids:
                continue

            case_cls = get_case_class(step["type"])
            self.result_store.add_case_result(
                CaseResult(
                    case_id=case_id,
                    name=step.get("name", case_id),
                    category=step.get("category", case_cls.category),
                    result="fail",
                    detail=f"not executed because test aborted early: {reason}",
                    required_state=step.get("required_state", case_cls.default_required_state),
                    resulting_state=self.state,
                    started_at=now,
                    finished_at=now,
                )
            )
            recorded_case_ids.add(case_id)

    def _mark_current_case_failed(self, step: Dict[str, Any], reason: str) -> None:
        if not step or "case" not in step:
            return

        case_id = step["case"]
        if any(item.case_id == case_id for item in self.result_store.case_results):
            return

        case_cls = get_case_class(step["type"])
        now = datetime.now().isoformat(timespec="seconds")
        self.result_store.add_case_result(
            CaseResult(
                case_id=case_id,
                name=step.get("name", case_id),
                category=step.get("category", case_cls.category),
                result="fail",
                detail=f"interrupted during execution: {reason}",
                required_state=step.get("required_state", case_cls.default_required_state),
                resulting_state=self.state,
                started_at=now,
                finished_at=now,
            )
        )

    def run(self) -> int:
        previous_sigint_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._handle_sigint)
        rospy.init_node("sunray_test_runner", disable_signals=True)
        self._print_banner(
            f"启动测试: platform={self.context.platform_name} env={self.context.environment_name} "
            f"suite={self.context.suite_name} uav={self.context.uav_name}"
        )
        self.vehicle = UAVAdapter(
            state_topic=self.context.resolved_topics["uav_state"],
            command_topic=self.context.resolved_topics["uav_control_cmd"],
            setup_topic=self.context.resolved_topics["uav_setup"],
        )
        stop_on_failure = bool(self.context.suite.get("stop_on_failure", True))
        record_rosbag = bool(self.context.suite.get("record_rosbag", True))
        bag_started = False
        current_step_index = -1
        current_step = None

        try:
            for current_step_index, step in enumerate(self.context.suite["steps"]):
                current_step = step
                if "phase" in step:
                    self._print_banner(f"[PHASE] {step['phase']} | current_state={self.state}")
                    if record_rosbag and not bag_started and step["phase"] == "arm_and_takeoff":
                        bag_path = self.rosbag.start(
                            self.context.run_paths.run_dir,
                            self.loaded["recording"].get("bag_prefix", self.context.platform_name),
                            self.context.recording_topics,
                        )
                        self.context.artifacts["bag_file"] = bag_path
                        bag_started = True
                        rospy.loginfo("rosbag 输出文件: %s", bag_path)
                    state_before = self.state
                    try:
                        self.state = run_phase(step["phase"], self.context, self.vehicle, self.event_logger)
                    except Exception as exc:
                        self.result_store.record_phase(
                            step["phase"],
                            state_before,
                            self.state,
                            status="failed",
                            detail=str(exc),
                        )
                        raise
                    self.result_store.record_phase(step["phase"], state_before, self.state, status="completed")
                    rospy.loginfo("[PHASE] %s 完成: %s -> %s", step["phase"], state_before, self.state)
                    current_step = None
                    continue

                case = self._make_case(step)
                self._print_banner(
                    f"[CASE] {case.execution_context.case_id} | name={case.execution_context.name} "
                    f"| required_state={case.execution_context.required_state}"
                )
                self._ensure_state(case.execution_context.required_state)
                try:
                    result = case.run(self.context, self.vehicle, self.event_logger)
                except (KeyboardInterrupt, rospy.ROSInterruptException):
                    raise
                except Exception as exc:  # pragma: no cover - runtime protection
                    result = CaseResult(
                        case_id=case.execution_context.case_id,
                        name=case.execution_context.name,
                        category=case.execution_context.category,
                        result="error",
                        detail=f"{exc}\n{traceback.format_exc()}",
                        required_state=case.execution_context.required_state,
                        resulting_state=self.state,
                    )
                self.result_store.add_case_result(result)
                self.state = result.resulting_state or self.state
                rospy.loginfo(
                    "[CASE] %s 结束: result=%s next_state=%s",
                    result.case_id,
                    result.result,
                    self.state,
                )
                if result.result in {"fail", "error"} and stop_on_failure:
                    rospy.logwarn("硬件检测失败，中断测试")
                    self._mark_remaining_cases_failed(
                        current_step_index + 1,
                        f"stopped after case {result.case_id} returned {result.result}",
                    )
                    break
                current_step = None

            if self.state == "airborne":
                self._print_banner("[PHASE] auto_land | current_state=airborne")
                state_before = self.state
                self.state = run_phase("land", self.context, self.vehicle, self.event_logger)
                self.result_store.record_phase("land", state_before, self.state, status="completed")
                rospy.loginfo("[PHASE] auto_land 完成: %s -> %s", state_before, self.state)
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            self.interrupted = True
            self.interruption_reason = "Ctrl+C"
            self.exit_code_override = 130
            self.event_logger.log("run_interrupted", f"state={self.state}")
            rospy.logwarn("开始紧急降落并中断测试")
            self._mark_current_case_failed(current_step, self.interruption_reason)
            self._mark_remaining_cases_failed(current_step_index + 1, self.interruption_reason)
            self._emergency_land_after_interrupt()
        except Exception as exc:
            self.interrupted = True
            self.interruption_reason = str(exc)
            self.exit_code_override = 1
            self.event_logger.log("run_interrupted", f"state={self.state} reason={exc}")
            rospy.logerr("测试异常中断: %s", exc)
            self._mark_current_case_failed(current_step, self.interruption_reason)
            self._mark_remaining_cases_failed(current_step_index + 1, self.interruption_reason)
            self._emergency_land_after_interrupt()
        finally:
            self.rosbag.stop()
            self._write_outputs()
            rospy.loginfo("结果文件: %s", self.context.run_paths.result_json)
            rospy.loginfo("报告文件: %s", self.context.run_paths.report_html)
            rospy.loginfo("事件日志: %s", self.context.run_paths.event_log_jsonl)
            signal.signal(signal.SIGINT, previous_sigint_handler)
            try:
                rospy.signal_shutdown("sunray_test finished")
            except Exception:
                pass

        if self.exit_code_override is not None:
            return self.exit_code_override

        summary = self.result_store.summary()
        self._print_banner(
            "测试结束 "
            f"total={summary['total']} pass={summary['pass']} fail={summary['fail']} "
            f"error={summary['error']} unsupported={summary['unsupported']}"
        )
        return 0 if summary["fail"] == 0 and summary["error"] == 0 else 1

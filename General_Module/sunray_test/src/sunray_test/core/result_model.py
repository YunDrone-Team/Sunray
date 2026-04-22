import json
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional


VALID_RESULTS = {"pass", "fail", "error", "unsupported"}


@dataclass
class CaseResult:
    case_id: str
    name: str
    category: str
    result: str
    detail: str = ""
    metrics: Dict[str, Any] = field(default_factory=dict)
    artifacts: Dict[str, Any] = field(default_factory=dict)
    required_state: Optional[str] = None
    resulting_state: Optional[str] = None
    started_at: str = ""
    finished_at: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.case_id,
            "name": self.name,
            "category": self.category,
            "result": self.result,
            "detail": self.detail,
            "metrics": self.metrics,
            "artifacts": self.artifacts,
            "required_state": self.required_state,
            "resulting_state": self.resulting_state,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
        }


class ResultStore:
    def __init__(self) -> None:
        self.phase_log: List[Dict[str, Any]] = []
        self.case_results: List[CaseResult] = []
        self.started_at = datetime.now().isoformat(timespec="seconds")
        self.finished_at = ""

    def record_phase(
        self,
        phase: str,
        state_before: str,
        state_after: str,
        status: str = "completed",
        detail: str = "",
    ) -> None:
        self.phase_log.append(
            {
                "phase": phase,
                "state_before": state_before,
                "state_after": state_after,
                "status": status,
                "detail": detail,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            }
        )

    def add_case_result(self, case_result: CaseResult) -> None:
        if case_result.result not in VALID_RESULTS:
            raise ValueError(f"unsupported result: {case_result.result}")
        self.case_results.append(case_result)

    def summary(self) -> Dict[str, int]:
        counts = {key: 0 for key in sorted(VALID_RESULTS)}
        for case_result in self.case_results:
            counts[case_result.result] += 1
        counts["total"] = len(self.case_results)
        return counts

    def write_json(self, path: str, payload: Dict[str, Any]) -> None:
        self.finished_at = datetime.now().isoformat(timespec="seconds")
        payload["run_info"]["started_at"] = self.started_at
        payload["run_info"]["finished_at"] = self.finished_at
        payload["summary"] = self.summary()
        payload["phase_log"] = self.phase_log
        payload["cases"] = [case_result.as_dict() for case_result in self.case_results]
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict

from sunray_test.core.result_model import CaseResult


@dataclass
class CaseExecutionContext:
    case_id: str
    name: str
    category: str
    params: Dict[str, Any] = field(default_factory=dict)
    required_state: str = ""
    resulting_state: str = ""


class BaseCase:
    case_type = "base"
    default_required_state = "precheck"
    default_resulting_state = ""
    category = "generic"

    def __init__(self, execution_context: CaseExecutionContext) -> None:
        self.execution_context = execution_context

    def run(self, context, vehicle, event_logger) -> CaseResult:
        event_logger.log("case_start", self.execution_context.case_id)
        try:
            result = self.execute(context, vehicle, event_logger)
        except Exception:
            event_logger.log("case_end", f"{self.execution_context.case_id}:error")
            raise
        event_logger.log("case_end", f"{self.execution_context.case_id}:{result.result}")
        return result

    def execute(self, context, vehicle, event_logger) -> CaseResult:  # pragma: no cover - interface
        raise NotImplementedError

    def _result(self, result: str, detail: str = "", metrics=None, artifacts=None) -> CaseResult:
        now = datetime.now().isoformat(timespec="seconds")
        return CaseResult(
            case_id=self.execution_context.case_id,
            name=self.execution_context.name,
            category=self.execution_context.category,
            result=result,
            detail=detail,
            metrics=metrics or {},
            artifacts=artifacts or {},
            required_state=self.execution_context.required_state,
            resulting_state=self.execution_context.resulting_state or self.execution_context.required_state,
            started_at=now,
            finished_at=now,
        )

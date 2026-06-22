from dataclasses import dataclass
from typing import Any, Dict, List

from sunray_test.dashboard.model import DashboardModel


@dataclass(frozen=True)
class DashboardRequest:
    item_ids: List[str]
    source: str
    param_overrides: Dict[str, Dict[str, Any]] = None
    external_source_override: int = None
    profile_override: str = ""


@dataclass
class DashboardSession:
    model: DashboardModel
    environment: str
    output_dir: str
    uav_id: int = 1

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DASHBOARD_UAV_ID = 1
VALID_ITEM_GROUPS = {"hardware", "function"}
VALID_ENVIRONMENTS = {"sim", "exp"}


@dataclass(frozen=True)
class TestItem:
    item_id: str
    name: str
    group: str
    step: Dict[str, Any]
    param_schema: Sequence[Dict[str, Any]] = ()
    requires_airborne: bool = False
    required_hardware: Sequence[str] = ()
    sim_only: bool = False
    exp_only: bool = False
    tags: Sequence[str] = ()


@dataclass(frozen=True)
class DashboardSelection:
    requested_item_ids: List[str]
    item_ids: List[str]
    auto_added_item_ids: List[str]


@dataclass(frozen=True)
class DashboardPlan:
    selection: DashboardSelection
    environment: str
    profile: str
    profile_reason: str
    external_source: int
    external_source_label: str
    runtime_state: Dict[str, Any]
    suite: Dict[str, Any]
    validation_warning: str = ""

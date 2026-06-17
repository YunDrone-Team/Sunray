from dataclasses import dataclass
from typing import List

from sunray_test.dashboard.model import DashboardModel


@dataclass(frozen=True)
class DashboardRequest:
    item_ids: List[str]
    source: str


@dataclass
class DashboardSession:
    model: DashboardModel
    environment: str
    output_dir: str
    uav_id: int = 1

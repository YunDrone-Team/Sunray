from typing import Dict, Type


CASE_REGISTRY: Dict[str, Type] = {}


def register_case(case_type: str):
    def decorator(case_cls):
        existing = CASE_REGISTRY.get(case_type)
        if existing is not None and existing is not case_cls:
            raise ValueError(f"duplicate case registration: {case_type}")
        case_cls.case_type = case_type
        CASE_REGISTRY[case_type] = case_cls
        return case_cls

    return decorator


def get_case_class(case_type: str):
    try:
        return CASE_REGISTRY[case_type]
    except KeyError as exc:
        raise KeyError(f"unsupported case type: {case_type}") from exc


from sunray_test.cases.flight.hover import HoverCase  # noqa: E402,F401
from sunray_test.cases.flight.visual_landing import VisualLandingCase  # noqa: E402,F401
from sunray_test.cases.flight.waypoint import WaypointMissionCase  # noqa: E402,F401
from sunray_test.cases.hardware.battery_voltage import BatteryVoltageCase  # noqa: E402,F401
from sunray_test.cases.hardware.camera_alive import CameraAliveCase  # noqa: E402,F401

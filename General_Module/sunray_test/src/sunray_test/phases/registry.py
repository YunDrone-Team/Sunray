from typing import Callable, Dict


PHASE_REGISTRY: Dict[str, Callable] = {}


def register_phase(phase_name: str):
    def decorator(phase_fn):
        existing = PHASE_REGISTRY.get(phase_name)
        if existing is not None and existing is not phase_fn:
            raise ValueError(f"duplicate phase registration: {phase_name}")
        PHASE_REGISTRY[phase_name] = phase_fn
        return phase_fn

    return decorator


def run_phase(name, context, vehicle, event_logger=None):
    try:
        phase_fn = PHASE_REGISTRY[name]
    except KeyError as exc:
        raise ValueError(f"unsupported phase: {name}") from exc
    return phase_fn(context, vehicle, event_logger)


from sunray_test.phases.common import phase_arm_and_takeoff  # noqa: E402,F401
from sunray_test.phases.common import phase_land  # noqa: E402,F401

import rospy

from sunray_test.adapters.uav_adapter import UAVAdapter
from sunray_test.capabilities.event_logger import EventLogger
from sunray_test.core.context import RunContext
from sunray_test.phases.registry import register_phase


@register_phase("arm_and_takeoff")
def phase_arm_and_takeoff(
    context: RunContext, vehicle: UAVAdapter, event_logger: EventLogger = None
) -> str:
    vehicle.wait_for_connection()
    vehicle.ensure_cmd_mode()
    vehicle.wait_before_arm(3.0)
    if event_logger is not None:
        event_logger.log("arm_start", "unlock")
    vehicle.arm(
        timeout_s=float(context.defaults.get("arm_timeout_s", 20.0)),
        retry_interval_s=float(context.defaults.get("arm_retry_interval_s", 1.0)),
    )
    if event_logger is not None:
        event_logger.log("arm_end", "unlock")
        event_logger.log("takeoff_start", str(float(context.defaults["takeoff_target_z_m"])))
    vehicle.takeoff(
        [0.0, 0.0, float(context.defaults["takeoff_target_z_m"])],
        reach_radius_m=float(context.defaults.get("takeoff_reach_radius_m", 0.12)),
        stable_time_s=float(context.defaults.get("takeoff_stable_time_s", 2.0)),
        timeout_s=float(context.defaults.get("takeoff_timeout_s", 40.0)),
        rate_hz=float(context.defaults.get("takeoff_command_rate_hz", 20.0)),
    )
    settle_time_s = float(context.defaults.get("post_takeoff_settle_time_s", 0.0))
    if settle_time_s > 0:
        vehicle.hold_position(
            settle_time_s,
            rate_hz=float(context.defaults.get("takeoff_command_rate_hz", 20.0)),
            target_z_m=float(context.defaults["takeoff_target_z_m"]),
            label="Takeoff Settle",
        )
    if event_logger is not None:
        event_logger.log("takeoff_end", str(float(context.defaults["takeoff_target_z_m"])))
    return "airborne"


@register_phase("land")
def phase_land(context: RunContext, vehicle: UAVAdapter, event_logger: EventLogger = None) -> str:
    if event_logger is not None:
        event_logger.log("land_start", "land")
    vehicle.land()
    if event_logger is not None:
        event_logger.log("land_end", "land")
    return "landed"

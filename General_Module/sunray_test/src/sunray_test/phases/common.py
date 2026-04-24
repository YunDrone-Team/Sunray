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
    vehicle.arm()
    if event_logger is not None:
        event_logger.log("arm_end", "unlock")
        event_logger.log("takeoff_start", str(list(context.defaults["takeoff_target_pos"])))
    vehicle.takeoff(context.defaults["takeoff_target_pos"])
    settle_time_s = float(context.defaults.get("post_takeoff_settle_time_s", 0.0))
    if settle_time_s > 0:
        try:
            rospy.sleep(settle_time_s)
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("Takeoff settle sleep interrupted") from exc
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS shutdown requested")
    if event_logger is not None:
        event_logger.log("takeoff_end", str(list(context.defaults["takeoff_target_pos"])))
    return "airborne"


@register_phase("land")
def phase_land(context: RunContext, vehicle: UAVAdapter, event_logger: EventLogger = None) -> str:
    if event_logger is not None:
        event_logger.log("land_start", "land")
    vehicle.land()
    if event_logger is not None:
        event_logger.log("land_end", "land")
    return "landed"

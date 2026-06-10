import select
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from sunray_msgs.msg import UAVControlCMD

from sunray_test.cases.base import BaseCase
from sunray_test.cases.registry import register_case


class EgoControlKeepalive:
    def __init__(
        self,
        pos_cmd_topic,
        control_cmd_topic,
        rate_hz=20.0,
        stale_timeout_s=0.5,
        zero_velocity_epsilon=1.0e-3,
    ):
        self.pos_cmd_topic = pos_cmd_topic
        self.control_cmd_topic = control_cmd_topic
        self.rate_hz = float(rate_hz)
        self.stale_timeout_s = float(stale_timeout_s)
        self.zero_velocity_epsilon = float(zero_velocity_epsilon)
        self._lock = threading.Lock()
        self._latest_pos_cmd = None
        self._latest_pos_cmd_time = None
        self._stop_event = threading.Event()
        self._thread = None
        self._sub = None
        self._pub = None

    def start(self):
        if self._thread is not None:
            return
        self._pub = rospy.Publisher(self.control_cmd_topic, UAVControlCMD, queue_size=10)
        self._sub = rospy.Subscriber(self.pos_cmd_topic, PositionCommand, self._pos_cmd_cb, queue_size=20)
        self._thread = threading.Thread(target=self._run, name="ego_control_keepalive")
        self._thread.daemon = True
        self._thread.start()
        rospy.loginfo(
            "EGO keepalive started: pos_cmd=%s control_cmd=%s rate=%.1fHz",
            self.pos_cmd_topic,
            self.control_cmd_topic,
            self.rate_hz,
        )

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None
        rospy.loginfo("EGO keepalive stopped")

    def _pos_cmd_cb(self, msg):
        with self._lock:
            self._latest_pos_cmd = msg
            self._latest_pos_cmd_time = rospy.Time.now()

    def _run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            msg = None
            msg_time = None
            with self._lock:
                msg = self._latest_pos_cmd
                msg_time = self._latest_pos_cmd_time

            if msg is not None and msg_time is not None:
                age_s = (rospy.Time.now() - msg_time).to_sec()
                if age_s <= self.stale_timeout_s:
                    self._pub.publish(self._build_control_cmd(msg))

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _build_control_cmd(self, msg):
        cmd = UAVControlCMD()
        cmd.header.stamp = rospy.Time.now()
        cmd.desired_pos = [msg.position.x, msg.position.y, msg.position.z]
        cmd.desired_vel = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        cmd.desired_acc = [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z]
        cmd.desired_yaw = msg.yaw
        cmd.desired_yaw_rate = msg.yaw_dot

        if self._is_zero_velocity(msg):
            cmd.cmd = UAVControlCMD.XyzPosYaw
        else:
            cmd.cmd = UAVControlCMD.XyzPosVelYaw
        return cmd

    def _is_zero_velocity(self, msg):
        return (
            abs(msg.velocity.x) <= self.zero_velocity_epsilon
            and abs(msg.velocity.y) <= self.zero_velocity_epsilon
            and abs(msg.velocity.z) <= self.zero_velocity_epsilon
        )


@register_case("flight.ego_goal")
class EgoGoalCase(BaseCase):
    case_type = "flight.ego_goal"
    category = "flight"
    default_required_state = "airborne"
    default_resulting_state = "airborne"
    _INPUT_CANCELLED = object()

    @staticmethod
    def _normalise_goal(raw_goal, default_z_m):
        if len(raw_goal) < 2:
            raise ValueError(f"ego goal must contain at least x/y: {raw_goal}")
        z_m = raw_goal[2] if len(raw_goal) >= 3 else default_z_m
        return [float(raw_goal[0]), float(raw_goal[1]), float(z_m)]

    @staticmethod
    def _distance(current, target, use_xy_only):
        dx = current[0] - target[0]
        dy = current[1] - target[1]
        if use_xy_only:
            return (dx ** 2 + dy ** 2) ** 0.5
        dz = current[2] - target[2]
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    @staticmethod
    def _build_goal_msg(goal, frame_id):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def _publish_goal_burst(pub, msg, burst_count, burst_interval_s):
        for _ in range(burst_count):
            if rospy.is_shutdown():
                raise KeyboardInterrupt("ROS shutdown requested")
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            if burst_interval_s > 0:
                rospy.sleep(burst_interval_s)

    @staticmethod
    def _sleep_rate(rate):
        try:
            rate.sleep()
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("ROS rate sleep interrupted") from exc
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS shutdown requested")

    def _resolve_goals(self, context, default_z_m):
        goal_source = str(
            self.execution_context.params.get(
                "goal_source",
                context.defaults.get("ego_goal_source", "list"),
            )
        ).strip().lower()

        if goal_source == "input":
            return goal_source, "rviz_input", None

        if goal_source != "list":
            raise ValueError(f"unsupported goal_source: {goal_source}")

        mission_key = self.execution_context.params["mission_key"]
        mission = context.missions[mission_key]
        if isinstance(mission, dict):
            raw_goals = mission.get("goals", mission.get("waypoints"))
        else:
            raw_goals = mission
        if not raw_goals:
            raise ValueError(f"ego goal mission is empty: {mission_key}")
        return goal_source, mission_key, [self._normalise_goal(goal, default_z_m) for goal in raw_goals]

    def _wait_for_rviz_goal(self, default_z_m):
        received = threading.Event()
        result = [None]

        def _cb(msg):
            if received.is_set():
                return
            goal = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z if msg.pose.position.z >= 0.3 else default_z_m,
            ]
            result[0] = goal
            received.set()

        sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, _cb, queue_size=1)
        rospy.sleep(0.1)
        print("\n[EGO Input] 请在 rviz 中发布 2D Nav Goal，或在终端输入 q 退出", flush=True)

        try:
            while not rospy.is_shutdown():
                if received.is_set():
                    break
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    line = sys.stdin.readline()
                    if not line or line.strip().lower() == "q":
                        return self._INPUT_CANCELLED
                    print("[EGO Input] 无效输入，请在 rviz 中点击目标点或输入 q 退出", flush=True)
        finally:
            sub.unregister()

        if result[0] is not None:
            print(f"[EGO Input] 收到目标点: [{result[0][0]:.2f}, {result[0][1]:.2f}, {result[0][2]:.2f}]", flush=True)
        return result[0]

    def _publish_until_reached(
        self,
        pub,
        frame_id,
        goal,
        vehicle,
        reach_radius_m,
        stable_time_s,
        hold_time_s,
        timeout_s,
        publish_burst_count,
        publish_burst_interval_s,
        use_xy_only,
    ):
        msg = self._build_goal_msg(goal, frame_id)
        self._publish_goal_burst(pub, msg, publish_burst_count, publish_burst_interval_s)
        start_time = time.time()
        stable_start = None
        first_entry_time = None
        rate = rospy.Rate(10)
        last_logged_distance = None

        while not rospy.is_shutdown():
            current = vehicle.state.position
            distance = self._distance(current, goal, use_xy_only)
            rounded_distance = round(distance, 2)
            if rounded_distance != last_logged_distance:
                print(f"\r[EGO Goal] target={goal} dist={rounded_distance:.2f}m", end="", flush=True)
                last_logged_distance = rounded_distance

            if distance < reach_radius_m:
                if first_entry_time is None:
                    first_entry_time = time.time()
                if stable_start is None:
                    rospy.loginfo("进入 EGO 目标点到达半径，开始判稳")
                    stable_start = time.time()
            else:
                stable_start = None

            if stable_start is not None and (time.time() - stable_start >= stable_time_s):
                print()
                if first_entry_time is not None:
                    rospy.loginfo("EGO 目标点稳定耗时 %.2fs", time.time() - first_entry_time)
                rospy.loginfo("EGO 目标点已到达并停稳")
                break

            if time.time() - start_time > timeout_s:
                print()
                raise RuntimeError(f"ego goal timeout: {goal}")

            self._sleep_rate(rate)

        if hold_time_s <= 0:
            return

        rospy.loginfo("开始 EGO 目标点停留 %.1fs", hold_time_s)
        hold_deadline = time.time() + hold_time_s
        last_hold_display = None
        while not rospy.is_shutdown() and time.time() < hold_deadline:
            remaining = max(0, int(hold_deadline - time.time()))
            if remaining != last_hold_display:
                print(f"\r[EGO Goal Hold] 倒计时: {remaining:02d}s", end="", flush=True)
                last_hold_display = remaining
            rospy.sleep(0.1)
        print()

    def execute(self, context, vehicle, event_logger):
        params = self.execution_context.params
        goal_topic = str(params.get("goal_topic", "/move_base_simple/goal"))
        frame_id = str(params.get("frame_id", "world"))
        z_m = float(params.get("z_m", context.defaults.get("ego_goal_z_m", 1.0)))
        reach_radius_m = float(
            params.get(
                "reach_radius_m",
                context.defaults.get("ego_goal_reach_radius_m", context.defaults["waypoint_reach_radius_m"]),
            )
        )
        stable_time_s = float(
            params.get(
                "stable_time_s",
                context.defaults.get("ego_goal_stable_time_s", context.defaults["waypoint_stable_time_s"]),
            )
        )
        hold_time_s = float(
            params.get("hold_time_s", context.defaults.get("ego_goal_hold_time_s", context.defaults["waypoint_hold_time_s"]))
        )
        timeout_s = float(
            params.get("timeout_s", context.defaults.get("ego_goal_timeout_s", context.defaults["waypoint_timeout_s"]))
        )
        publish_burst_count = max(
            1,
            int(params.get("publish_burst_count", context.defaults.get("ego_goal_publish_burst_count", 3))),
        )
        publish_burst_interval_s = max(
            0.0,
            float(
                params.get(
                    "publish_burst_interval_s",
                    context.defaults.get("ego_goal_publish_burst_interval_s", 0.1),
                )
            ),
        )
        use_xy_only = bool(params.get("use_xy_only", context.defaults.get("ego_goal_use_xy_only", True)))
        keepalive_enabled = bool(params.get("keepalive_enabled", context.defaults.get("ego_keepalive_enabled", True)))
        keepalive_rate_hz = float(params.get("keepalive_rate_hz", context.defaults.get("ego_keepalive_rate_hz", 20.0)))
        keepalive_stale_timeout_s = float(
            params.get("keepalive_stale_timeout_s", context.defaults.get("ego_keepalive_stale_timeout_s", 0.5))
        )
        keepalive_zero_velocity_epsilon = float(
            params.get(
                "keepalive_zero_velocity_epsilon",
                context.defaults.get("ego_keepalive_zero_velocity_epsilon", 1.0e-3),
            )
        )
        post_transition_enabled = bool(
            params.get(
                "post_transition_enabled",
                context.defaults.get("ego_post_transition_enabled", False),
            )
        )
        post_transition_target_yaw_rad = float(
            params.get(
                "post_transition_target_yaw_rad",
                context.defaults.get("ego_post_transition_target_yaw_rad", 0.0),
            )
        )
        post_transition_yaw_rate_rad_s = float(
            params.get(
                "post_transition_yaw_rate_rad_s",
                context.defaults.get("ego_post_transition_yaw_rate_rad_s", 0.25),
            )
        )
        post_transition_hold_after_s = float(
            params.get(
                "post_transition_hold_after_s",
                context.defaults.get("ego_post_transition_hold_after_s", 1.0),
            )
        )
        post_transition_target_z_m = params.get(
            "post_transition_target_z_m",
            context.defaults.get("ego_post_transition_target_z_m", None),
        )
        if post_transition_target_z_m is not None:
            post_transition_target_z_m = float(post_transition_target_z_m)
        pos_cmd_topic = str(params.get("pos_cmd_topic", context.resolved_topics.get("ego_pos_cmd", "/uav1/pos_cmd")))
        control_cmd_topic = str(
            params.get("control_cmd_topic", context.resolved_topics.get("uav_control_cmd", "/uav1/sunray/uav_control_cmd"))
        )
        goal_source, mission_key, goals = self._resolve_goals(context, z_m)

        rospy.loginfo(
            "[CASE] %s: 开始 EGO 目标点测试 source=%s topic=%s frame=%s mission=%s",
            self.execution_context.case_id,
            goal_source,
            goal_topic,
            frame_id,
            mission_key,
        )
        pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        rospy.sleep(0.5)

        executed_goals = []
        keepalive = None

        if keepalive_enabled:
            keepalive = EgoControlKeepalive(
                pos_cmd_topic=pos_cmd_topic,
                control_cmd_topic=control_cmd_topic,
                rate_hz=keepalive_rate_hz,
                stale_timeout_s=keepalive_stale_timeout_s,
                zero_velocity_epsilon=keepalive_zero_velocity_epsilon,
            )
            keepalive.start()

        try:
            if goal_source == "input":
                while True:
                    goal = self._wait_for_rviz_goal(z_m)
                    if goal is self._INPUT_CANCELLED:
                        break
                    if goal is None:
                        break

                    idx = len(executed_goals)
                    rospy.loginfo(
                        "[CASE] %s: 执行 rviz 目标点 %d -> %s",
                        self.execution_context.case_id,
                        idx + 1,
                        goal,
                    )
                    event_logger.log("ego_goal_start", f"{self.execution_context.case_id}:{idx}:{goal}")
                    try:
                        self._publish_until_reached(
                            pub, frame_id, goal, vehicle,
                            reach_radius_m, stable_time_s, hold_time_s,
                            timeout_s, publish_burst_count, publish_burst_interval_s, use_xy_only,
                        )
                    except RuntimeError as exc:
                        event_logger.log("ego_goal_fail", f"{self.execution_context.case_id}:{idx}:{exc}")
                        rospy.logwarn("[CASE] %s: EGO 目标点失败 %s", self.execution_context.case_id, exc)
                        print("[EGO Input] 目标点执行失败，请重新发布下一个目标点。", flush=True)
                        continue
                    event_logger.log("ego_goal_end", f"{self.execution_context.case_id}:{idx}")
                    executed_goals.append(goal)

                if not executed_goals:
                    rospy.logwarn("[CASE] %s: 用户主动退出 EGO 目标点输入", self.execution_context.case_id)
                    return self._result(
                        "unsupported",
                        detail="ego goal input cancelled by user",
                        metrics={
                            "goal_source": goal_source,
                            "goal_topic": goal_topic,
                            "frame_id": frame_id,
                            "mission_key": mission_key,
                            "goal_count": 0,
                            "goals": [],
                            "use_xy_only": use_xy_only,
                            "keepalive_enabled": keepalive_enabled,
                        },
                    )

                rospy.loginfo(
                    "[CASE] %s: 用户结束 EGO 目标点输入，共执行 %d 个目标点",
                    self.execution_context.case_id,
                    len(executed_goals),
                )
            else:
                for idx, goal in enumerate(goals):
                    rospy.loginfo(
                        "[CASE] %s: 发布 EGO 目标点 %d/%d -> %s",
                        self.execution_context.case_id,
                        idx + 1,
                        len(goals),
                        goal,
                    )
                    event_logger.log("ego_goal_start", f"{self.execution_context.case_id}:{idx}:{goal}")
                    try:
                        self._publish_until_reached(
                            pub, frame_id, goal, vehicle,
                            reach_radius_m, stable_time_s, hold_time_s,
                            timeout_s, publish_burst_count, publish_burst_interval_s, use_xy_only,
                        )
                    except RuntimeError as exc:
                        event_logger.log("ego_goal_fail", f"{self.execution_context.case_id}:{idx}:{exc}")
                        rospy.logwarn("[CASE] %s: EGO 目标点失败 %s", self.execution_context.case_id, exc)
                        return self._result(
                            "fail",
                            detail=str(exc),
                            metrics={
                                "goal_source": goal_source,
                                "goal_topic": goal_topic,
                                "frame_id": frame_id,
                                "mission_key": mission_key,
                                "failed_goal_index": idx,
                                "failed_goal": goal,
                                "completed_goal_count": len(executed_goals),
                                "completed_goals": executed_goals,
                                "reach_radius_m": reach_radius_m,
                                "stable_time_s": stable_time_s,
                                "timeout_s": timeout_s,
                                "publish_burst_count": publish_burst_count,
                                "publish_burst_interval_s": publish_burst_interval_s,
                                "use_xy_only": use_xy_only,
                                "keepalive_enabled": keepalive_enabled,
                                "keepalive_rate_hz": keepalive_rate_hz,
                            },
                        )
                    event_logger.log("ego_goal_end", f"{self.execution_context.case_id}:{idx}")
                    executed_goals.append(goal)
        finally:
            if keepalive is not None:
                keepalive.stop()

        if post_transition_enabled:
            rospy.loginfo("[CASE] %s: 执行 EGO 后偏航过渡", self.execution_context.case_id)
            vehicle.transition_yaw(
                target_yaw_rad=post_transition_target_yaw_rad,
                yaw_rate_rad_s=post_transition_yaw_rate_rad_s,
                hold_after_s=post_transition_hold_after_s,
                target_z_m=post_transition_target_z_m,
            )

        rospy.loginfo("[CASE] %s 完成", self.execution_context.case_id)
        return self._result(
            "pass",
            detail=f"completed {len(executed_goals)} ego goals",
            metrics={
                "goal_source": goal_source,
                "goal_topic": goal_topic,
                "frame_id": frame_id,
                "mission_key": mission_key,
                "goal_count": len(executed_goals),
                "goals": executed_goals,
                "publish_burst_count": publish_burst_count,
                "publish_burst_interval_s": publish_burst_interval_s,
                "use_xy_only": use_xy_only,
                "keepalive_enabled": keepalive_enabled,
                "keepalive_rate_hz": keepalive_rate_hz,
                "keepalive_pos_cmd_topic": pos_cmd_topic,
                "keepalive_control_cmd_topic": control_cmd_topic,
                "post_transition_enabled": post_transition_enabled,
                "post_transition_target_yaw_rad": post_transition_target_yaw_rad,
                "post_transition_yaw_rate_rad_s": post_transition_yaw_rate_rad_s,
                "post_transition_hold_after_s": post_transition_hold_after_s,
                "post_transition_target_z_m": post_transition_target_z_m,
            },
        )

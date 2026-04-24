import subprocess
import time
from typing import List, Sequence

import rospy
from sunray_msgs.msg import UAVControlCMD, UAVSetup, UAVState


class UAVAdapter:
    def __init__(self, state_topic: str, command_topic: str, setup_topic: str) -> None:
        self._state_topic = state_topic
        self._command_topic = command_topic
        self._setup_topic = setup_topic
        self._state = UAVState()
        self._state_sub = rospy.Subscriber(self._state_topic, UAVState, self._state_cb)
        self._cmd_pub = rospy.Publisher(self._command_topic, UAVControlCMD, queue_size=10)
        self._setup_pub = rospy.Publisher(self._setup_topic, UAVSetup, queue_size=10)

    @property
    def state(self) -> UAVState:
        return self._state

    def _state_cb(self, msg: UAVState) -> None:
        self._state = msg

    @staticmethod
    def _raise_if_shutdown() -> None:
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS shutdown requested")

    @staticmethod
    def _sleep_or_interrupt(duration_s: float) -> None:
        try:
            rospy.sleep(duration_s)
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("ROS sleep interrupted") from exc
        UAVAdapter._raise_if_shutdown()

    @staticmethod
    def _rate_sleep_or_interrupt(rate: rospy.Rate) -> None:
        try:
            rate.sleep()
        except rospy.ROSInterruptException as exc:
            raise KeyboardInterrupt("ROS rate sleep interrupted") from exc
        UAVAdapter._raise_if_shutdown()

    @staticmethod
    def _countdown(label: str, duration_s: float) -> None:
        end_time = time.time() + duration_s
        last_display = None
        while not rospy.is_shutdown():
            remaining = max(0, int(end_time - time.time()))
            if remaining != last_display:
                print(f"\r[{label}] 倒计时: {remaining:02d}s", end="", flush=True)
                last_display = remaining
            if remaining <= 0:
                break
            UAVAdapter._sleep_or_interrupt(0.05)
        print()
        UAVAdapter._raise_if_shutdown()

    def wait_for_connection(self, timeout_s: float = 15.0) -> None:
        rospy.loginfo("等待飞控连接")
        deadline = time.time() + timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._state.connected:
                rospy.loginfo("飞控连接成功")
                return
            self._rate_sleep_or_interrupt(rate)
        self._raise_if_shutdown()
        raise RuntimeError("UAV connection timeout")

    def ensure_cmd_mode(self) -> None:
        setup_cmd = UAVSetup()

        while not rospy.is_shutdown() and self._state.control_mode != UAVSetup.CMD_CONTROL:
            rospy.loginfo("切换到 CMD_CONTROL 模式")
            setup_cmd.cmd = UAVSetup.SET_CONTROL_MODE
            setup_cmd.control_mode = "CMD_CONTROL"
            self._setup_pub.publish(setup_cmd)
            self._sleep_or_interrupt(1.0)

        self._raise_if_shutdown()
        rospy.loginfo("控制模式已切换完成")

    def wait_before_arm(self, wait_time_s: float) -> None:
        if wait_time_s <= 0:
            return
        rospy.loginfo("CMD_CONTROL 已切换完成，%.1f 秒后开始解锁", wait_time_s)
        self._countdown("Arm Delay", wait_time_s)

    def arm(self) -> None:
        setup_cmd = UAVSetup()
        while not rospy.is_shutdown() and not self._state.armed:
            rospy.loginfo("发送解锁指令")
            setup_cmd.cmd = UAVSetup.ARM
            self._setup_pub.publish(setup_cmd)
            self._sleep_or_interrupt(1.0)
        self._raise_if_shutdown()
        rospy.loginfo("飞机已解锁")

    def takeoff(self, target_pos: Sequence[float]) -> None:
        rospy.loginfo("开始起飞，目标悬停点: %s", list(target_pos))
        cmd = UAVControlCMD()
        while not rospy.is_shutdown() and self._state.landed_state != 2:
            cmd.cmd = UAVControlCMD.Takeoff
            self._cmd_pub.publish(cmd)
            rospy.loginfo("起飞中，当前 landed_state=%s", self._state.landed_state)
            self._sleep_or_interrupt(2.0)

        self._raise_if_shutdown()
        cmd.cmd = UAVControlCMD.XyzPosYaw
        cmd.desired_pos = list(target_pos)
        cmd.desired_yaw = 0.0
        self._cmd_pub.publish(cmd)
        self._sleep_or_interrupt(2.0)
        rospy.loginfo("起飞完成，已发送定点控制")

    def hover(self, duration_s: float, rate_hz: float = 20.0) -> None:
        rospy.loginfo("开始悬停，持续 %.1fs", duration_s)
        cmd = UAVControlCMD()
        deadline = time.time() + duration_s
        rate = rospy.Rate(rate_hz)
        last_display = None
        while not rospy.is_shutdown() and time.time() < deadline:
            cmd.header.stamp = rospy.Time.now()
            cmd.cmd = UAVControlCMD.Hover
            self._cmd_pub.publish(cmd)
            remaining = max(0, int(deadline - time.time()))
            if remaining != last_display:
                print(f"\r[Hover] 倒计时: {remaining:02d}s", end="", flush=True)
                last_display = remaining
            self._rate_sleep_or_interrupt(rate)
        print()
        self._raise_if_shutdown()
        rospy.loginfo("悬停阶段结束")

    def goto_waypoint(
        self,
        target: Sequence[float],
        reach_radius_m: float,
        stable_time_s: float,
        hold_time_s: float,
        timeout_s: float,
        rate_hz: float = 20.0,
    ) -> None:
        cmd = UAVControlCMD()
        cmd.cmd = UAVControlCMD.XyzPosYaw
        cmd.desired_pos = list(target)
        cmd.desired_yaw = 0.0

        rospy.loginfo(
            "开始飞向航点 target=%s, reach_radius=%.2f, stable_time=%.1f, hold_time=%.1f, timeout=%.1f",
            list(target),
            reach_radius_m,
            stable_time_s,
            hold_time_s,
            timeout_s,
        )
        waypoint_start = time.time()
        stable_start = None
        first_entry_time = None
        rate = rospy.Rate(rate_hz)
        last_logged_distance = None

        while not rospy.is_shutdown():
            cur = self._state.position
            dx = cur[0] - target[0]
            dy = cur[1] - target[1]
            dz = cur[2] - target[2]
            dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
            rounded_distance = round(dist, 2)
            if rounded_distance != last_logged_distance:
                print(f"\r[Waypoint] target={list(target)} dist={rounded_distance:.2f}m", end="", flush=True)
                last_logged_distance = rounded_distance

            if dist < reach_radius_m:
                if first_entry_time is None:
                    first_entry_time = time.time()
                if stable_start is None:
                    rospy.loginfo("进入航点到达半径，开始判稳")
                    stable_start = time.time()
            else:
                stable_start = None

            if stable_start is not None and (time.time() - stable_start >= stable_time_s):
                print()
                if first_entry_time is not None:
                    stabilization_time_s = time.time() - first_entry_time
                    rospy.loginfo(
                        "航点稳定耗时 %.2fs ",
                        stabilization_time_s,
                    )
                rospy.loginfo("航点已到达并停稳")
                break

            if time.time() - waypoint_start > timeout_s:
                print()
                raise RuntimeError(f"waypoint timeout: {target}")

            cmd.header.stamp = rospy.Time.now()
            self._cmd_pub.publish(cmd)
            self._rate_sleep_or_interrupt(rate)

        self._raise_if_shutdown()
        rospy.loginfo("开始航点停留 %.1fs", hold_time_s)
        hold_deadline = time.time() + hold_time_s
        last_hold_display = None
        while not rospy.is_shutdown() and time.time() < hold_deadline:
            cmd.header.stamp = rospy.Time.now()
            self._cmd_pub.publish(cmd)
            remaining = max(0, int(hold_deadline - time.time()))
            if remaining != last_hold_display:
                print(f"\r[Waypoint Hold] 倒计时: {remaining:02d}s", end="", flush=True)
                last_hold_display = remaining
            self._rate_sleep_or_interrupt(rate)
        print()
        self._raise_if_shutdown()
        rospy.loginfo("航点停留结束")

    def land(self) -> None:
        rospy.loginfo("开始降落")
        cmd = UAVControlCMD()
        while not rospy.is_shutdown() and self._state.armed:
            cmd.cmd = UAVControlCMD.Land
            self._cmd_pub.publish(cmd)
            rospy.loginfo("降落中，armed=%s", self._state.armed)
            self._sleep_or_interrupt(1.0)
        self._raise_if_shutdown()
        rospy.loginfo("降落完成")

    def visual_land(self, launch_file: str, auto_takeoff: bool, height_m: float) -> int:
        rospy.loginfo(
            "启动视觉降落 launch=%s auto_takeoff=%s height=%.2f",
            launch_file,
            auto_takeoff,
            height_m,
        )
        result = subprocess.run(
            [
                "roslaunch",
                "sunray_tutorial",
                launch_file,
                f"auto_takeoff:={'true' if auto_takeoff else 'false'}",
                f"height:={height_m}",
            ]
        )
        rospy.loginfo("视觉降落结束，返回码=%s", result.returncode)
        return result.returncode

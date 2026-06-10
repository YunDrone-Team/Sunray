import os
import math
import subprocess
import tempfile
import time
from typing import Dict, List, Optional, Sequence
from xml.sax.saxutils import quoteattr

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

    def _state_brief(self) -> str:
        return (
            f"armed={self._state.armed}, "
            f"mode={self._state.mode}, "
            f"landed_state={self._state.landed_state}, "
            f"position={[round(v, 3) for v in self._state.position]}, "
            f"velocity={[round(v, 3) for v in self._state.velocity]}"
        )

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

    def arm(self, timeout_s: float = 20.0, retry_interval_s: float = 1.0) -> None:
        setup_cmd = UAVSetup()
        rospy.loginfo("开始解锁，timeout=%.1fs", timeout_s)
        start_time = time.time()
        last_state_log = 0.0
        while not rospy.is_shutdown() and not self._state.armed:
            if time.time() - start_time > timeout_s:
                raise RuntimeError(f"arming timeout: {self._state_brief()}")
            rospy.loginfo("发送解锁指令")
            setup_cmd.cmd = UAVSetup.ARM
            self._setup_pub.publish(setup_cmd)
            if time.time() - last_state_log >= 3.0:
                rospy.loginfo("当前状态: %s", self._state_brief())
                last_state_log = time.time()
            self._sleep_or_interrupt(retry_interval_s)
        self._raise_if_shutdown()
        rospy.loginfo("飞机已解锁: %s", self._state_brief())

    def takeoff(
        self,
        target_pos: Sequence[float],
        reach_radius_m: float = 0.12,
        stable_time_s: float = 2.0,
        timeout_s: float = 40.0,
        rate_hz: float = 20.0,
    ) -> None:
        target_z = float(target_pos[2]) if len(target_pos) >= 3 else 0.0
        rospy.loginfo("开始起飞，目标高度: %.2f m", target_z)
        if not self._state.armed:
            raise RuntimeError(f"takeoff rejected because UAV is not armed: {self._state_brief()}")
        cmd = UAVControlCMD()
        takeoff_start = time.time()
        while not rospy.is_shutdown() and self._state.landed_state != 2:
            if time.time() - takeoff_start > timeout_s:
                raise RuntimeError(f"takeoff timeout before airborne: {self._state_brief()}")
            if not self._state.armed:
                raise RuntimeError(f"takeoff aborted because UAV disarmed: {self._state_brief()}")
            cmd.cmd = UAVControlCMD.Takeoff
            self._cmd_pub.publish(cmd)
            rospy.loginfo("起飞中，当前 landed_state=%s", self._state.landed_state)
            self._sleep_or_interrupt(1.0)

        self._raise_if_shutdown()
        rospy.loginfo(
            "等待起飞稳定 reach_radius=%.2fm stable_time=%.1fs timeout=%.1fs",
            reach_radius_m,
            stable_time_s,
            timeout_s,
        )

        stable_start = None
        target_start = time.time()
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            cur = self._state.position
            dz = abs(cur[2] - target_z)
            rounded_distance = round(dz, 2)
            print(f"\r[Takeoff] z_error={rounded_distance:.2f}m target_z={target_z:.2f}m", end="", flush=True)

            if dz <= reach_radius_m:
                if stable_start is None:
                    rospy.loginfo("进入起飞目标半径，开始判稳")
                    stable_start = time.time()
            else:
                stable_start = None

            if stable_start is not None and (time.time() - stable_start >= stable_time_s):
                print()
                rospy.loginfo("起飞完成，已到达目标悬停点")
                break

            if time.time() - target_start > timeout_s:
                print()
                raise RuntimeError(f"takeoff target timeout: z={target_z}")

            self._rate_sleep_or_interrupt(rate)

        print()
        self._raise_if_shutdown()

    def hold_position(
        self,
        duration_s: float,
        rate_hz: float = 20.0,
        target_z_m: Optional[float] = None,
        label: str = "Hold",
    ) -> List[float]:
        rospy.loginfo("开始定点保持，持续 %.1fs", duration_s)
        target_pos = list(self._state.position)
        if len(target_pos) < 3:
            raise RuntimeError(f"hold rejected because UAV position is invalid: {self._state_brief()}")
        if target_z_m is not None:
            target_pos[2] = float(target_z_m)
        target_pos = target_pos[:3]
        rospy.loginfo("定点保持目标: %s", [round(v, 3) for v in target_pos])
        cmd = UAVControlCMD()
        cmd.cmd = UAVControlCMD.XyzPosYaw
        cmd.desired_pos = target_pos
        cmd.desired_yaw = 0.0
        deadline = time.time() + duration_s
        rate = rospy.Rate(rate_hz)
        last_display = None
        while not rospy.is_shutdown() and time.time() < deadline:
            cmd.header.stamp = rospy.Time.now()
            self._cmd_pub.publish(cmd)
            remaining = max(0, int(deadline - time.time()))
            if remaining != last_display:
                print(f"\r[{label}] 倒计时: {remaining:02d}s", end="", flush=True)
                last_display = remaining
            self._rate_sleep_or_interrupt(rate)
        print()
        self._raise_if_shutdown()
        rospy.loginfo("定点保持结束")
        return target_pos

    def hover(self, duration_s: float, rate_hz: float = 20.0, target_z_m: Optional[float] = None) -> List[float]:
        rospy.loginfo("开始悬停，持续 %.1fs", duration_s)
        target_pos = self.hold_position(duration_s, rate_hz=rate_hz, target_z_m=target_z_m, label="Hover")
        rospy.loginfo("悬停阶段结束")
        return target_pos

    @staticmethod
    def _normalize_angle_rad(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def transition_yaw(
        self,
        target_yaw_rad: float = 0.0,
        yaw_rate_rad_s: float = 0.25,
        hold_after_s: float = 1.0,
        target_z_m: Optional[float] = None,
        rate_hz: float = 20.0,
    ) -> None:
        if yaw_rate_rad_s <= 0:
            raise ValueError("yaw_rate_rad_s must be positive")

        target_pos = list(self._state.position)
        if len(target_pos) < 3:
            raise RuntimeError(f"yaw transition rejected because UAV position is invalid: {self._state_brief()}")
        if target_z_m is not None:
            target_pos[2] = float(target_z_m)
        target_pos = target_pos[:3]

        current_yaw = float(self._state.attitude[2]) if len(self._state.attitude) >= 3 else 0.0
        target_yaw = self._normalize_angle_rad(float(target_yaw_rad))
        yaw_error = self._normalize_angle_rad(target_yaw - current_yaw)
        duration_s = abs(yaw_error) / float(yaw_rate_rad_s)
        rospy.loginfo(
            "开始偏航过渡 current_yaw=%.2f target_yaw=%.2f delta=%.2f duration=%.1fs target_pos=%s",
            current_yaw,
            target_yaw,
            yaw_error,
            duration_s,
            [round(v, 3) for v in target_pos],
        )

        cmd = UAVControlCMD()
        cmd.cmd = UAVControlCMD.XyzPosYaw
        cmd.desired_pos = target_pos
        rate = rospy.Rate(rate_hz)
        start_time = time.time()
        last_display = None

        while not rospy.is_shutdown():
            elapsed_s = time.time() - start_time
            ratio = 1.0 if duration_s <= 1.0e-3 else min(1.0, elapsed_s / duration_s)
            desired_yaw = self._normalize_angle_rad(current_yaw + yaw_error * ratio)
            cmd.header.stamp = rospy.Time.now()
            cmd.desired_yaw = desired_yaw
            self._cmd_pub.publish(cmd)

            remaining = max(0, int(duration_s - elapsed_s))
            if remaining != last_display:
                print(f"\r[Yaw Transition] 倒计时: {remaining:02d}s yaw={desired_yaw:.2f}", end="", flush=True)
                last_display = remaining
            if ratio >= 1.0:
                break
            self._rate_sleep_or_interrupt(rate)

        hold_deadline = time.time() + max(0.0, float(hold_after_s))
        while not rospy.is_shutdown() and time.time() < hold_deadline:
            cmd.header.stamp = rospy.Time.now()
            cmd.desired_yaw = target_yaw
            self._cmd_pub.publish(cmd)
            self._rate_sleep_or_interrupt(rate)
        print()
        self._raise_if_shutdown()
        rospy.loginfo("偏航过渡结束")

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

    def visual_land(
        self,
        launch_file: str,
        auto_takeoff: bool,
        height_m: float,
        launch_args: Optional[Dict[str, object]] = None,
        remaps: Optional[Sequence[Dict[str, str]]] = None,
    ) -> Dict[str, object]:
        normalized_remaps = self._normalize_remaps(remaps)
        rospy.loginfo(
            "启动视觉降落 launch=%s auto_takeoff=%s height=%.2f remaps=%s",
            launch_file,
            auto_takeoff,
            height_m,
            normalized_remaps,
        )
        # 使用临时文件同时实现：1) 终端实时显示（无缓冲延迟） 2) 事后捕获输出用于失败检测
        # 之前用 stdout=PIPE 会导致 roslaunch SUMMARY 被缓冲，显示顺序与节点直接输出不一致
        tmp_fd, tmp_path = tempfile.mkstemp(prefix="sunray_vland_", suffix=".log", dir="/tmp")
        wrapper_path = ""
        try:
            if normalized_remaps:
                wrapper_path = self._write_visual_land_wrapper(launch_file, normalized_remaps)
                launch_command = ["roslaunch", wrapper_path]
            else:
                launch_command = ["roslaunch", "sunray_tutorial", launch_file]
            launch_command.extend(
                [
                    f"auto_takeoff:={'true' if auto_takeoff else 'false'}",
                    f"height:={height_m}",
                ]
            )
            if launch_args:
                for key, value in launch_args.items():
                    if value is None:
                        continue
                    if isinstance(value, bool):
                        value_text = "true" if value else "false"
                    else:
                        value_text = str(value)
                    launch_command.append(f"{key}:={value_text}")
            with os.fdopen(tmp_fd, "w") as tmp_file:
                process = subprocess.Popen(
                    launch_command,
                    stdout=tmp_file,
                    stderr=subprocess.STDOUT,
                    env={**os.environ, "PYTHONUNBUFFERED": "1"},
                )
                # 实时 tail 临时文件，让终端立即看到输出
                tail_proc = subprocess.Popen(
                    ["tail", "-f", "--pid", str(process.pid), tmp_path],
                    stdout=None,  # 直接输出到终端
                    stderr=None,
                )
                return_code = process.wait()
                tail_proc.wait(timeout=3)

            # 读取完整输出用于失败模式匹配
            with open(tmp_path, "r") as f:
                output_lines = [line.rstrip() for line in f.readlines()]
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            if wrapper_path:
                try:
                    os.unlink(wrapper_path)
                except OSError:
                    pass

        rospy.loginfo("视觉降落结束，返回码=%s", return_code)
        return {"return_code": return_code, "output_lines": output_lines[-300:], "remaps": normalized_remaps}

    @staticmethod
    def _normalize_remaps(remaps: Optional[Sequence[Dict[str, str]]]) -> List[Dict[str, str]]:
        if not remaps:
            return []
        normalized = []
        for index, remap in enumerate(remaps):
            if not isinstance(remap, dict):
                raise ValueError(f"visual landing remap #{index} must be a mapping")
            source = remap.get("from")
            target = remap.get("to")
            if not source or not target:
                raise ValueError(f"visual landing remap #{index} requires from/to")
            normalized.append({"from": str(source), "to": str(target)})
        return normalized

    @staticmethod
    def _write_visual_land_wrapper(launch_file: str, remaps: Sequence[Dict[str, str]]) -> str:
        fd, wrapper_path = tempfile.mkstemp(prefix="sunray_vland_wrapper_", suffix=".launch", dir="/tmp")
        include_file = f"$(find sunray_tutorial)/launch/{launch_file}"
        lines = ["<launch>"]
        for remap in remaps:
            lines.append(f"  <remap from={quoteattr(remap['from'])} to={quoteattr(remap['to'])} />")
        lines.append(f"  <include file={quoteattr(include_file)} pass_all_args=\"true\" />")
        lines.append("</launch>")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        return wrapper_path

    def stop_nodes(self, node_names: Sequence[str], timeout_s: float = 5.0) -> Dict[str, int]:
        results = {}
        for node_name in node_names:
            rospy.loginfo("停止 ROS 节点: %s", node_name)
            try:
                result = subprocess.run(
                    ["rosnode", "kill", node_name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=timeout_s,
                )
                results[node_name] = result.returncode
                if result.returncode != 0:
                    rospy.logwarn("停止 ROS 节点失败或节点不存在: %s output=%s", node_name, result.stdout.strip())
            except subprocess.TimeoutExpired:
                results[node_name] = -1
                rospy.logwarn("停止 ROS 节点超时: %s", node_name)
        return results

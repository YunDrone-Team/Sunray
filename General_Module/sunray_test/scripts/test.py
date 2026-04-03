#!/usr/bin/env python3
import rospy
import subprocess
import time
import signal
import sys
import time
import os
import json
import shutil
from time import sleep
from datetime import datetime, timedelta
from sunray_msgs.msg import UAVControlCMD, UAVSetup, UAVState
from sensor_msgs.msg import Image
from sensor_msgs.msg import BatteryState

front_cam_topic = "/cam_front_node/image_raw"
down_cam_topic = "/cam_down_node/image_raw"
battery_topic = "/uav1/mavros/battery"
rosbag_topics = [
    "/vrpn_client_node_1/uav1/pose",
    "/vrpn_client_node_1/uav1/twist",
    "/uav1/mavros/local_position/pose",
    # "/uav1/sunray/gazebo_pose"
]

INPUT_JSON = os.path.expanduser("~/Sunray/General_Module/sunray_test/config/test_result_basic.json")
OUTPUT_DIR = os.path.expanduser("~/Sunray/tests/output")
OUTPUT_JSON = os.path.join(OUTPUT_DIR, "test_result.json")

current_state = UAVState()
cmd_pub = None
setup_pub = None
hardware_test_flag = False
program_start_time = 0
program_end_time = 0
duration = 0
waypoint_list = [
    [0.5,-1.0, 1.0],
    [0.5, 1.0, 1.0],
    [0.0, 0.0, 1.6]
]

function_test = {
    "mocap_flight": "fail",
    "waypoint_flight": "fail",
    "apriltags_landmark": "fail"
}
hardware_test = {
    "front_cam": "fail",
    "down_cam": "fail",
    "battery": "fail"
}

def input_test_info():
    if not os.path.exists(INPUT_JSON):
        print(f"input json does not exist: {INPUT_JSON}")
        return

    with open(INPUT_JSON, "r") as f:
        data = json.load(f)

    sn = input("请输入设备SN: ")
    tester = input("请输入测试人员: ")

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data["test_info"]["sn"] = sn
    data["test_info"]["tester"] = tester
    data["test_info"]["time"] = current_time

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    with open(OUTPUT_JSON, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Test information has been written to JSON: {OUTPUT_JSON}")

class HardwareChecker:
    def __init__(self, json_path = OUTPUT_JSON):
        self.json_path = json_path

    def check_camera_device(self, topic_name):
        if "monocular" in topic_name.lower():
            return True
        return os.path.exists("/dev/video0")

    def check_topic_alive(self, topic_name, msg_type, timeout=2.0):
        last_time = {"t": None}

        def callback(msg):
            last_time["t"] = rospy.Time.now()

        sub = rospy.Subscriber(topic_name, msg_type, callback)
        rospy.sleep(timeout)
        sub.unregister()

        if last_time["t"] and (rospy.Time.now() - last_time["t"]).to_sec() < 1.0:
            return True
        return False

    def check_camera_ok(self, topic_name, msg_type, timeout=2.0):
        device_ok = self.check_camera_device(topic_name)
        topic_ok = self.check_topic_alive(topic_name, msg_type, timeout)

        return "pass" if (device_ok and topic_ok) else "fail"

    def check_battery_voltage(self, topic, threshold=13.0, timeout=2.0):
        voltage_data = {"v": None}

        def callback(msg):
            voltage_data["v"] = msg.voltage

        sub = rospy.Subscriber(topic, BatteryState, callback)
        rospy.sleep(timeout)
        sub.unregister()

        if voltage_data["v"] is None:
            return "fail", None

        result = "pass" if voltage_data["v"] >= threshold else "fail"
        return result, voltage_data["v"]

    def update_json(self, hardware_test, voltage):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except json.JSONDecodeError:
                print("JSON corrupted, recreate.")
                data = {}
        else:
            print("JSON does not exist, create a new file.")
            data = {}

        data.setdefault("hardware_test", {})

        data["hardware_test"].update(hardware_test)
        data["hardware_test"]["battery_voltage"] = voltage

        with open(self.json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=4)

        print("hardware_test has been written to JSON.")

def uav_init(rate): # UAV 初始化
    global setup_pub

    rospy.loginfo("Waiting for UAV connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    rospy.loginfo("UAV connected!")

    setup_cmd = UAVSetup()

    # 切换模式
    while current_state.control_mode != UAVSetup.CMD_CONTROL:
        setup_cmd.cmd = UAVSetup.SET_CONTROL_MODE
        setup_cmd.control_mode = "CMD_CONTROL"
        setup_pub.publish(setup_cmd)
        rospy.loginfo("Switching to CMD_CONTROL...")
        rospy.sleep(1.0)

    rospy.loginfo("CMD_CONTROL enabled")

    # 解锁
    while not current_state.armed:
        setup_cmd.cmd = UAVSetup.ARM
        setup_pub.publish(setup_cmd)
        rospy.loginfo("Arming...")
        rospy.sleep(1.0)

    rospy.loginfo("Armed!")

def uav_takeoff(): # 起飞
    global cmd_pub

    cmd = UAVControlCMD()

    while current_state.landed_state != 2:
        cmd.cmd = UAVControlCMD.Takeoff
        cmd_pub.publish(cmd)
        rospy.loginfo("Taking off...")
        rospy.sleep(4.0)

    rospy.loginfo("Takeoff success")

def uav_hover(rate,hover_time = 30): # 悬停
    global cmd_pub

    cmd = UAVControlCMD()

    rospy.loginfo("Start hovering")

    start_time = time.time()

    count_down(weeks=0, days=0, hours=0, minutes=0, seconds=hover_time)

    while not rospy.is_shutdown() and (time.time() - start_time < hover_time):
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        rate.sleep()

    rospy.loginfo("Hover finished")

    function_test["mocap_flight"] = "pass"

def uav_goto_xyz(rate, threshold=0.2, mode="input"): # 指点飞行
    global cmd_pub, current_state

    rospy.loginfo("开始测试指点飞行")

    index = 0

    while not rospy.is_shutdown():
        if mode == "input": #指点飞行的模式
            try:
                x = float(input(" x [m]: "))
                y = float(input(" y [m]: "))
                z = float(input(" z [m]: "))
            except ValueError:
                print("输入错误，请输入数字！")
                continue

            if x == 0 and y == 0 and z == 0:
                rospy.loginfo("退出指点飞行模式")
                break

            target = [x, y, z]

        elif mode == "list":
            if index >= len(waypoint_list):
                rospy.loginfo("列表路径执行完成")
                break

            target = waypoint_list[index]
            rospy.loginfo(f"目标点 {index}: {target}")
            index += 1

        else:
            rospy.logerr("未知模式！")
            break

        cmd = UAVControlCMD()
        cmd.cmd = UAVControlCMD.XyzPosYaw
        cmd.desired_pos = target
        cmd.desired_yaw = 0.0

        rospy.loginfo(f"Flying to {target}")

        stable_count = 0

        while not rospy.is_shutdown(): #到达判断

            cur = current_state.position

            dx = cur[0] - target[0]
            dy = cur[1] - target[1]
            dz = cur[2] - target[2]

            dist = (dx**2 + dy**2 + dz**2) ** 0.5

            # 稳定判断
            if dist < threshold:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count > 20:
                rospy.loginfo("已到达目标点")
                break

            cmd.header.stamp = rospy.Time.now()
            cmd_pub.publish(cmd)

            rate.sleep()

        rospy.loginfo("准备下一个目标点")

        time.sleep(1)
    
    function_test["waypoint_flight"] = "pass"


def landmark_detect(): #视觉降落(子进程)
    subprocess.run([
        "roslaunch",
        "sunray_tutorial",
        "auto_land_by_pose.launch",
        "auto_takeoff:=false",
        "height:=1.5"
    ])

    function_test["apriltags_landmark"] = "pass"


def uav_land(): #降落
    global cmd_pub
    cmd = UAVControlCMD()

    rospy.loginfo("Landing...")
    while not rospy.is_shutdown() and current_state.armed:
        cmd.cmd = UAVControlCMD.Land
        cmd_pub.publish(cmd)
        rospy.sleep(1.0)

    rospy.loginfo("Landed and disarmed")

def state_cb(msg): # 状态回调
    global current_state
    current_state = msg

def emergency_exit(signum, frame):
    global cmd_pub
    rospy.logwarn("EMERGENCY LANDING!")

    cmd = UAVControlCMD()
    for _ in range(30):
        cmd.cmd = UAVControlCMD.Land
        cmd_pub.publish(cmd)
        rospy.sleep(0.1)

    rospy.signal_shutdown("Emergency exit")
    sys.exit(0)

def output_dir_check(): #检查output文件夹
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"output directory has been created")
    else:
        print(f"output directory already exists")

def count_down(weeks=0, days=0, hours=0, minutes=0, seconds=0): #倒计时模块 #1
    remain_time = timedelta(weeks=weeks, days=days, hours=hours, minutes=minutes, seconds=seconds)
    while remain_time.total_seconds() > 0:
        time.sleep(1)
        remain_time -= timedelta(seconds=1)
        print("\rHover countdown：{}".format(remain_time), end="", flush=True)

def print_test(): #倒计时模块 #2
    for i in range(20):
        print("#", end="", flush=True)
        sleep(0.1)

def start_recording(topics, output_dir):
    global bag_proc

    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{output_dir}/sunray150_basic_{now}.bag"

    cmd = ["rosbag", "record", "-O", filename] + topics

    bag_proc = subprocess.Popen(cmd)

    rospy.loginfo(f"start recording rosbag: {filename}")

def stop_recording(): #停止录制rosbag
    global bag_proc

    if bag_proc is not None:
        bag_proc.terminate()
        bag_proc.wait()
        rospy.loginfo("stop recording rosbag")

def write_test_result():
    global duration
    with open(OUTPUT_JSON, "r", encoding="utf-8") as f:
        data = json.load(f)

    data["function_test"] = function_test
    data["test_info"]["duration"] = f"{duration:.3f}"

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)

    print("function test results have been written")

def safe_input(prompt):
    while True:
        value = input(prompt).strip()
        if value:
            return value
        print("输入不能为空，请重新输入！")


def archive_outputs():
    output_dir = OUTPUT_DIR

    if not os.path.exists(output_dir):
        print("output目录不存在:", output_dir)
        return

    print("当前output内容:", os.listdir(output_dir))

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    target_folder = os.path.join(output_dir, timestamp)
    os.makedirs(target_folder, exist_ok=True)

    moved = 0

    for name in os.listdir(OUTPUT_DIR):
        file_path = os.path.join(OUTPUT_DIR, name)

        if not os.path.isfile(file_path):
            continue

        target_path = os.path.join(target_folder, name)

        try:
            shutil.move(file_path, target_path)
            moved += 1
        except Exception as e:
            print("失败:", e)

    print("归档目录:", target_folder)


def generate_report():
    subprocess.run([
        "rosrun",
        "sunray_test",
        "report_output.py",
        ])
    time.sleep(3)
    

def main():
    global cmd_pub, setup_pub, duration, hardware_test_flag

    rospy.init_node("sunray_test")
    signal.signal(signal.SIGINT, emergency_exit)
    output_dir_check()
    input_test_info()
    program_start_time = time.time()

    checker = HardwareChecker()
    hardware_test = {}
    hardware_test["front_cam"] = checker.check_camera_ok(front_cam_topic, Image)
    hardware_test["down_cam"] = checker.check_camera_ok(down_cam_topic, Image)
    front = hardware_test["front_cam"] or "fail"
    down = hardware_test["down_cam"] or "fail"
    if front == "pass" and down == "pass":
        rospy.loginfo("前视摄像头正常，下视摄像头正常")
    elif front == "pass" and down == "fail":
        rospy.loginfo("前视摄像头正常，下视摄像头异常")
    elif front == "fail" and down == "pass":
        rospy.loginfo("前视摄像头异常，下视摄像头正常")
    else:
        rospy.loginfo("前视摄像头异常，下视摄像头异常")
    result, voltage = checker.check_battery_voltage(battery_topic)
    hardware_test["battery"] = result
    rospy.loginfo(f"Battery voltage: {voltage:.3f}V")
    print("Hardware test results:")
    for k, v in hardware_test.items():
        print(f"{k}: {v}")
    hardware_test_flag = all(v == "pass" for v in hardware_test.values())
    checker.update_json(hardware_test, voltage)

    if hardware_test_flag:
        uav_id = rospy.get_param("~uav_id", 1)
        uav_name = f"/uav{uav_id}"
        rospy.Subscriber(uav_name + "/sunray/uav_state", UAVState, state_cb)
        cmd_pub = rospy.Publisher(
            uav_name + "/sunray/uav_control_cmd",
            UAVControlCMD,
            queue_size=10)
        setup_pub = rospy.Publisher(
            uav_name + "/sunray/setup",
            UAVSetup,
            queue_size=10)
        rate = rospy.Rate(20)
        uav_init(rate)
        start_recording(rosbag_topics,OUTPUT_DIR)
        uav_takeoff()
        uav_hover(rate,hover_time = 20)
        uav_goto_xyz(rate,threshold = 0.3,mode = "list")
        landmark_detect()
        uav_land()
        stop_recording()

    program_end_time = time.time()
    duration = program_end_time - program_start_time
    write_test_result()
    generate_report()
    archive_outputs()

if __name__ == "__main__":
    main()


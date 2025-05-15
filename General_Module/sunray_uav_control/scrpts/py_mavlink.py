from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:localhost:14550')
master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
def Arm():
    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
    time.sleep(1)
    
def DisArm():
    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to disarm")
    # wait until disarming confirmed
    master.motors_disarmed_wait()
    print('Dis Armed!')

def send_manual_control(x, y, z, r, buttons=0):
    """
    发送手动控制指令（归一化值）
    :param x: 前后（-1000~1000，正值为前）
    :param y: 左右（-1000~1000，正值为右）
    :param z: 油门（0~1000，正值为升）
    :param r: 偏航（-1000~1000，正值为右转）
    :param buttons: 按钮位掩码（可选）
    """
    master.mav.manual_control_send(
        master.target_system,
        x, y, z, r, buttons
    )



# Arm()

while True:
    # 示例：悬停（油门 50%，其他中立）
    send_manual_control(0, 0, 400, 0)
    time.sleep(0.05)
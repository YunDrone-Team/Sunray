from pymavlink import mavutil

# 建立与 PX4 的连接
master = mavutil.mavlink_connection('udp:localhost:14541')

# 等待系统就绪
while not master.wait_heartbeat(timeout=5):
    print("等待心跳包超时。请确认 PX4 正在运行并连接正确。")
print("心跳包接收成功。")

# 发送 TRAJECTORY_REPRESENTATION_BEZIER 消息
bezier_msg = mavutil.mavlink.MAVLink_trajectory_representation_bezier_message(
    0, 0, 0,  # 系统 ID、组件 ID 和序号（对于此消息可以设为 0）
    0, 0, 0, 0, 0, 0,  # 保留（设为 0）
    5,  # 贝塞尔元组数量
    10.0, 0.0, 0.0,  # 第一个元组的位置（X、Y、Z）
    5.0, 10.0, 0.0,  # 第二个元组的位置（X、Y、Z）
    0.0, 0.0, 5.0,  # 速度、加速和抖动（设为 0）
    0, 0, 0, 0,  # 旗帜（设为 0）
    0, 0, 0, 0, 0, 0  # 保留（设为 0）
)
master.mav.trajectory_representation_bezier_send(bezier_msg)

# 关闭连接
master.close()
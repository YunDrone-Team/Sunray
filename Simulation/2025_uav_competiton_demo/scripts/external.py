import rospy 
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import ParamSet, ParamSetRequest

class Obs:
    def __init__(self, sim = False, auto_turnoff = False):
        self.obstacles = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.delivery = [0.0, 0.0]
        self.obstacles_sub = []

        self.vrpn_name = "vrpn_client_node_1"
        self.sim = sim

        self.auto_turnoff = auto_turnoff

        self.update = [0, 0, 0, 0]  # 用于更新障碍物位置的标志

    def open(self):
        if self.sim:
            self.obstacles_sub = [
                rospy.Subscriber('/obstacle_1/pose', PoseStamped, self.obstacle_cb, callback_args=0),
                rospy.Subscriber('/obstacle_2/pose', PoseStamped, self.obstacle_cb, callback_args=1),
                rospy.Subscriber('/obstacle_3/pose', PoseStamped, self.obstacle_cb, callback_args=2),
                rospy.Subscriber('/delivery_target/pose', PoseStamped, self.obstacle_cb, callback_args=3),
            ]
        else:
            self.obstacles_sub = [
                rospy.Subscriber(self.vrpn_name + '/obs1/pose', PoseStamped, self.obstacle_cb, callback_args=0),
                rospy.Subscriber(self.vrpn_name + '/obs2/pose', PoseStamped, self.obstacle_cb, callback_args=1),
                rospy.Subscriber(self.vrpn_name + '/obs3/pose', PoseStamped, self.obstacle_cb, callback_args=2),
                rospy.Subscriber(self.vrpn_name + '/delivery/pose', PoseStamped, self.obstacle_cb, callback_args=3),
            ]

    def obstacle_cb(self, msg, obstacle_id):
        self.update[obstacle_id] = 1  # 标记该障碍物需要更新
        if obstacle_id == 3:
            self.delivery = [msg.pose.position.x, msg.pose.position.y]
        else:
            self.obstacles[obstacle_id] = [msg.pose.position.x, msg.pose.position.y]
        # 如果所有障碍物都更新了，则关闭订阅
        if all(self.update) and self.auto_turnoff:
            rospy.loginfo("All obstacles updated, closing subscriptions.")
            self.close()
            self.update = [0, 0, 0, 0]

    def get_obstacles(self):
        return self.obstacles
    
    def get_delivery(self):
        return self.delivery
    
    def close(self):
        for sub in self.obstacles_sub:
            sub.unregister()

class Servo:
    def __init__(self, uav_name):
        "使用参数设置服务对舵机进行控制"
        self.uav_name = uav_name
        rospy.wait_for_service(self.uav_name + "/mavros/param/set")
        self.servo_ctrl = rospy.ServiceProxy(
            self.uav_name + "/mavros/param/set", ParamSet
        )
        self.servo_mode = ParamSetRequest() # 舵机模式 默认是aux1控制（407），使用offboard控制时，切换到最大值（2）
        self.servo_mode.param_id = "PWM_MAIN_FUNC5"
        self.servo_angel = ParamSetRequest() # 舵机角度 （200 - 2450）
        self.servo_angel.param_id = "PWM_MAIN_MAX5"

        "后续修改这两个参数以适配具体安装方式"
        self.servo_init_angle = 1600        # 初始角度
        self.servo_drop_angle = 2200        # 投放角度

    "舵机控制接口"
    "舵机使用指南："
    "使用servo_setangle需要先运行self.servo_setmode(2)把舵机模式改成最大值输出"
    # CONSTANT_MIN = 1
    # CONSTANT_MAX = 2
    # RC_AUX1 = 407
    def servo_setmode(self, modeID):
        if modeID == 1 or 2 or 407:
            self.servo_mode.value.integer = modeID
            try:
                if self.servo_ctrl.call(self.servo_mode).success:
                    rospy.loginfo(f"Turn Servo mode {modeID}!")
                else:
                    rospy.loginfo(f"Turn Servo mode False-->no_success")
            except rospy.ROSInterruptException:
                pass
        else:
            rospy.loginfo(f"Turn Servo mode False-->modeID_unuse")

    "舵机角度控制接口"
    # angle :1600 - 2200
    def servo_setangle(self, angle):
        if 1600 <= angle and angle <= 2200:
            self.servo_angel.value.integer = angle
            try:
                if self.servo_ctrl.call(self.servo_angel).success:
                    rospy.loginfo(f"Set Servo angle {angle}!")
                else:
                    rospy.loginfo(f"Set Servo angle False-->no_success")
            except rospy.ROSInterruptException:
                pass
        else:
            rospy.loginfo(f"Set Servo angle False-->angle_limit")

    "舵机初始化"
    def servo_init(self):
        self.servo_setmode(2)
        self.servo_setangle(self.servo_init_angle)

    "舵机投放"
    def servo_drop(self):
        self.servo_setangle(self.servo_drop_angle)

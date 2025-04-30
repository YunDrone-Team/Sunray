import socket
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QLineEdit, QPushButton, QLabel, QRadioButton, QHBoxLayout, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from message import *
from server import *

class State_Message:
    def __init__(self):
        self.head = 0
        self.length = 0
        self.msg_id = 0
        self.robot_id = 0
        self.check = 0
        self.time_stamp = 0
        self.uav_id = 0
        self.connected = 0
        self.armed = 0
        self.mode = ""
        self.location_source = 0
        self.odom_valid = 0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.attitude_rate = [0.0, 0.0, 0.0]
        self.battery_state = 0
        self.battery_percentage = 0
        self.control_mode = 0

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        try:
            server_address = ('192.168.0.29', 8969)
            self.tcpSocket = TcpSocketHandler(server_address)
            server_address = ('192.168.0.15', 8968)
            self.udpScket =  UdpReceiver(server_address)
            print("连接成功")
        except:
            print("连接失败")
        self.initUI()

    def initUI(self):
        layout = QHBoxLayout()

        # 左边显示模块
        left_layout = QVBoxLayout()
        module_layout = QHBoxLayout()
        
        module_layout = QHBoxLayout()
        self.inputs_display_1 = []
        for j in range(4):                   # 显示状态
            # if j == 0:
            #     input_display = QTextEdit()
            #     input_display.setText("0")
            #     input_display.setReadOnly(True)  # 设置为只读
            #     input_display.setFixedSize(30, 30)  # 设置大小为20x20像素
            #     input_display.setStyleSheet('background-color: white; border: 0px solid black; border-radius: 15px;')  # 设置背景颜色、边框样式和边框半径
            # else:
            input_display = QLineEdit()
            input_display.setText("")
            input_display.setReadOnly(True)  # 设置为只读
            input_display.setStyleSheet('border: none;')
            input_display.setAlignment(Qt.AlignCenter)
            module_layout.addWidget(input_display)
            self.inputs_display_1.append(input_display)
        left_layout.addLayout(module_layout)

        self.inputs_display_2 = []
        tags = ["X", "Y", "Z"]
        module_layout = QHBoxLayout()
        # self.showlab = QComboBox()
        # self.showlab.addItems(["POS", "VEL", "ATT"])
        # module_layout.addWidget(self.showlab)
        
        for j in range(3):
            input_display = QLineEdit()
            input_display.setText(tags[j])
            input_display.setReadOnly(True)  # 设置为只读
            input_display.setStyleSheet('border: none;')
            input_display.setAlignment(Qt.AlignCenter)
            module_layout.addWidget(input_display)
            self.inputs_display_2.append(input_display)
        left_layout.addLayout(module_layout)

        self.inputs_display_3 = []
        labels = ["POS", "VEL", "ATT"]
        for i in range(3):
            module_layout = QHBoxLayout()
            label = QLabel(labels[i])
            label.setAlignment(Qt.AlignCenter)
            # label.setStyleSheet("font-weight: font-size: 12px;")
            module_layout.addWidget(label)
            for j in range(3):
                input_display = QLineEdit()
                input_display.setReadOnly(True)  # 设置为只读
                input_display.setStyleSheet("background-color: lightgray;")  # 设置背景色
                input_display.setAlignment(Qt.AlignCenter)
                module_layout.addWidget(input_display)
                self.inputs_display_3.append(input_display)
            left_layout.addLayout(module_layout)

        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        layout.addWidget(left_widget)

        # 右边输入功能
        right_layout = QVBoxLayout()

        self.combo = QComboBox()
        self.combo.addItems(["XYZ_POS_NED", "XYZ_VEL_NED", "XYZ_POS_BODY", "XYZ_VEL_BODY", "XY_VEL_Z_POS_NED", "XY_VEL_Z_POS_BODY"])
        right_layout.addWidget(self.combo)

        self.inputs = []
        for i in range(4):
            self.inputs.append(QLineEdit())
            right_layout.addWidget(self.inputs[i])
        
        self.button = QPushButton("发送")
        right_layout.addWidget(self.button)

        self.radio1 = QRadioButton("base control")
        self.radio2 = QRadioButton("tutorial control")
        self.radio1.setChecked(True)
        right_layout.addWidget(self.radio1)
        right_layout.addWidget(self.radio2)

        # 添加下拉框和两个按钮
        self.dropdown = QComboBox()
        self.dropdown.addItems(["Arm", "Disarm", "Takeoff", "Land", "Hover"])
        right_layout.addWidget(self.dropdown)

        self.button1 = QPushButton("发送控制")
        self.button2 = QPushButton("切换offboard")
        self.button3 = QPushButton("紧急停止")
        right_layout.addWidget(self.button1)
        right_layout.addWidget(self.button2)
        right_layout.addWidget(self.button3)

        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        layout.addWidget(right_widget)

        self.setLayout(layout)

        self.radio2.toggled.connect(self.toggle_inputs)
        self.button.clicked.connect(self.send_move_command)
        self.button1.clicked.connect(self.send_control_command)
        self.button2.clicked.connect(self.send_offboard_command)
        self.button3.clicked.connect(self.send_emergency_stop_command)
        self.setGeometry(300, 300, 600, 200)  # 调整窗口大小以适应新的布局
        self.setWindowTitle("PyQt5示例")
        self.show()

    def send_move_command(self):
        combo_value = self.combo.currentText()
        input_values = [input.text() for input in self.inputs]
        radio_value = "base control" if self.radio1.isChecked() else "tutorial control"
        if(input_values[0] == "" or input_values[1] == "" or input_values[2] == "" or input_values[3] == ""):
            print("请输入所有参数")
            return
        else:
            print(f"Input Input: {input_values}"
            
        )
        print(f"Combo values: {combo_value}")
        
        msg = Control_Message()
        msg.head = 0xADFF
        msg.length = 41
        msg.msg_id = 102
        msg.robot_id = 1
        msg.time_stamp = 1234567
        msg.x = float(input_values[0])
        msg.y = float(input_values[1])
        msg.z = float(input_values[2])
        msg.yaw = float(input_values[3])* (3.141592653589793 / 180.0)
        msg.yaw_rate = False
        if combo_value == "XYZ_POS_NED":
            msg.type = 1
            msg.frame = 0
        elif combo_value == "XYZ_VEL_NED":
            msg.type = 2
            msg.frame = 0
        elif combo_value == "XYZ_POS_BODY":
            msg.type = 1
            msg.frame = 1
        elif combo_value == "XYZ_VEL_BODY":
            msg.type = 2
            msg.frame = 1
        elif combo_value == "XY_VEL_Z_POS_NED":
            msg.type = 3
            msg.frame = 0
        elif combo_value == "XY_VEL_Z_POS_BODY":
            msg.type = 3
            msg.frame = 1

        msg.check = msg.calculate_checksum_Control_Message()
        packed_msg = msg.pack()
        self.tcpSocket.sock.sendall(packed_msg)
        

    # 发送控制命令
    # DISARM 0
    # ARM 1
    # TAKEOFF 2
    # LAND 3
    # HOVER 4
    # KILL 5
    def send_control_command(self):
        msg = Vehicle_Message()
        msg.head = 0xADFF
        msg.msg_id = 104
        msg.length = 15
        msg.robot_id = 1
        msg.time_stamp = 1234567
        combo_value = self.dropdown.currentText()
        print(f"Combo value: {combo_value}")
        if combo_value == "Arm":
            msg.type = 1
            msg.check = msg.calculate_checksum_Vehicle_Message()
            packed_msg = msg.pack()
            self.tcpSocket.sock.sendall(packed_msg)
        elif combo_value == "Disarm":
            msg.time_stamp = 1234567
            msg.type = 0
            msg.check = msg.calculate_checksum_Vehicle_Message()
            packed_msg = msg.pack()
            self.tcpSocket.sock.sendall(packed_msg)
        elif combo_value == "Takeoff":
            msg.type = 2
            msg.check = msg.calculate_checksum_Vehicle_Message()
            packed_msg = msg.pack()
            self.tcpSocket.sock.sendall(packed_msg)
        elif combo_value == "Land":
            msg.type = 3
            msg.check = msg.calculate_checksum_Vehicle_Message()
            packed_msg = msg.pack()
            self.tcpSocket.sock.sendall(packed_msg)
        elif combo_value == "Hover":
            msg.type = 4
            msg.check = msg.calculate_checksum_Vehicle_Message()
            packed_msg = msg.pack()
            self.tcpSocket.sock.sendall(packed_msg)
    
    # 切入offboard模式
    def send_offboard_command(self):
        msg = Message()
        msg.head = 0xADFF
        msg.length = 15
        msg.msg_id = 103
        msg.robot_id = 1
        msg.time_stamp = 1234567
        msg.uav_mode = 1
        msg.check = msg.calculate_checksum_Mode_Message()
        print(msg.head, msg.length, msg.msg_id, msg.robot_id, msg.time_stamp, msg.uav_mode, msg.check)
        packed_msg = msg.pack()
        self.tcpSocket.sock.sendall(packed_msg)
    # 紧急停止
    def send_emergency_stop_command(self):
        msg = Vehicle_Message()
        msg.head = 0xADFF
        msg.msg_id = 104
        msg.length = 15
        msg.robot_id = 1
        msg.time_stamp = 1234567
        msg.type = 5
        msg.check = msg.calculate_checksum_Vehicle_Message()
        packed_msg = msg.pack()
        self.tcpSocket.sock.sendall(packed_msg)

    def toggle_inputs(self, checked):
        if checked:
            self.combo.clear()
            self.combo.addItems(["takeoff_hover_land", "block_pos", "circle_vel", "circle_z_pos"])
            for input in self.inputs:
                input.hide()
        else:
            self.combo.clear()
            self.combo.addItems(["XYZ_POS_NED", "XYZ_VEL_NED", "XYZ_POS_BODY", "XYZ_VEL_BODY", "XY_VEL_Z_POS_NED", "XY_VEL_Z_POS_BODY"])
            for input in self.inputs:
                input.show()

    def update_state(self):
        if self.udpScket.recv_flag:
            try:
                self.inputs_display_1[0].setText(str(self.udpScket.state.uav_id))
                if self.udpScket.state.connected:
                    self.inputs_display_1[0].setStyleSheet("background-color: green;")
                    self.inputs_display_1[0].setAlignment(Qt.AlignCenter)
                else:
                    self.inputs_display_1[0].setStyleSheet("background-color: red;")
                    self.inputs_display_1[0].setAlignment(Qt.AlignCenter)
                if self.udpScket.state.armed:
                    self.inputs_display_1[1].setText("armed")
                else:
                    self.inputs_display_1[1].setText("disarmed")
                self.inputs_display_1[2].setText(str(self.udpScket.state.mode.strip().decode('utf-8')))
                self.inputs_display_1[3].setText(str(round(float(self.udpScket.state.battery_state), 2)))

                self.inputs_display_3[0].setText(str(round(float(self.udpScket.state.position[0]),2)))
                self.inputs_display_3[1].setText(str(round(float(self.udpScket.state.position[1]),2)))
                self.inputs_display_3[2].setText(str(round(float(self.udpScket.state.position[2]),2)))

                self.inputs_display_3[3].setText(str(round(float(self.udpScket.state.velocity[0]),2)))
                self.inputs_display_3[4].setText(str(round(float(self.udpScket.state.velocity[1]),2)))
                self.inputs_display_3[5].setText(str(round(float(self.udpScket.state.velocity[2]),2)))

                self.inputs_display_3[6].setText(str(round(float(self.udpScket.state.attitude[0])/3.14*180,2)))
                self.inputs_display_3[7].setText(str(round(float(self.udpScket.state.attitude[1])/3.14*180,2)))
                self.inputs_display_3[8].setText(str(round(float(self.udpScket.state.attitude[2])/3.14*180,2)))
            except:
                pass
            

    def start_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_state)
        self.timer.start(10)  # 每秒更新一次

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.start_timer()
    sys.exit(app.exec_())
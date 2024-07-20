# sunray_wheeltec_robot

- `wheeltec_robot.h`定义了`WheeltecRobot`类
    - `WheeltecRobot`类中调用了`VIOBOT`类
- `wheeltec_robot_node.cpp`调用了`WheeltecRobot`类，是一个使用示例
    - `roslaunch sunray_wheeltec_robot wheeltec_robot.launch` 用于启动该节点
- `ugv_terminal_control.cpp`是测试脚本


## 运行

- 启动节点
    `./start.sh`
- 启动控制节点
    rosrun sunray_wheeltec_robot ugv_terminal_control
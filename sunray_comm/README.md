# Sunray

思锐无人机仿真与实践开发平台

### Supported Autopilot

- FMT
- PX4

### TODO

- 调试方向一：
    - 获取VIOBOT的位置及点云数据
    - 位置、速度、姿态等打印出来测试可靠性，就用ENU坐标系

- 调试方向二：
    - 打通FMT/PX4状态、模式等解析
    - 打通FMT/PX4的解锁、模式切换等指令
    - 使用PX4来测试vision pose数据（FMT逻辑目前未知）

- 重要：测试vision——pose 使用VIOBOT来确认位置
   - 如何考虑vio状态失效和重置问题？
   - 是否需要一个uav state集合？
- 无人机状态


 - 搞一个ICF5或者CUAV 5+
    - 刷写好固件和参数，使得USB默认连地面站，Telem2默认连机载电脑
    - 简易的测试文档
    - 这样就可以直接拿一个飞控进行测试了

- XML文件是手敲的，需要自定义
    - sunray.xml

- 接收
    - 无法配置的问题需要解决
        - 插上QGC可以设置频率，不接QGC不能设置频率
    - sunray话题自定义 排列组合的问题
    - 需要自定义哪些接收的数据

- 发送
    - 位置等指令的信息流 如何下发？ 以及怎么测试的问题？ 是不是要提前写sunray了？
    - PX4又该怎么测试？gazebo仿真？
    - 解锁用哪个MAVLink消息？
    - 切换模式用哪个MAVLink消息？
    - 紧急上锁用哪个MAVLink消息？
    - 重启飞控用哪个MAVLink消息？
    - QGC闪退：频率太高了吗？ QG连接上之后 突然收到了其他mavlink消息
    - 收到vision 的数据 但是设定的50Hz 只能收到20Hz 这个是串口波特率的问题 还是程序处理不过来的问题(收发要分开处理)
        - ROS没有实时处理 中间很多连续处理的 这意味着会有延迟

- VISION POSE TEST
    - FMT融合了哪些信息？PX4中又融合了哪些信息？
    - 位置、速度、三轴姿态角以及cov？

### 配置

```shell
## 下载Sunray-ROS源代码
git clone https://gitee.com/potato77/Sunray-Comm

## 编译
cd Sunray-Comm
./build.sh
```

```shell
## 环境变量配置
echo "source ~/Sunray-Comm/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
## 为了确认你的包路径已经设置，回显 ROS_PACKAGE_PATH 变量
echo $ROS_PACKAGE_PATH
```

### 运行

```shell
## 第一次连接可能需要端口赋权限
sudo chmod 777 /dev/ttyUSB0
```


```shell
## 连接FMT飞控,测试MAVLink消息收发情况
roslaunch sunray_comm sunray_comm_fmt.launch
```

```shell
## 连接PX4飞控,测试MAVLink消息收发情况
roslaunch sunray_comm sunray_comm_px4.launch
```

```shell
## 测试VISION_POSE话题融合效果
roslaunch sunray_comm sunray_comm_fmt.launch

roslaunch sunray_control vision_pose_test.launch
```
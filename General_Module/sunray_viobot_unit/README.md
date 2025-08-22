# viobot2自启动配置指南

## 创建服务：

在终端中输入：
```bash
sudo vim /etc/systemd/system/sunray.service
```

按 i 进入编辑模式，将下面这段复制进去
```bash
[Unit]
Description=Start Sunray Task

[Service]
Type=simple
User=PRR
ExecStart = /home/PRR/Sunray/General_Module/sunray_viobot_unit/SunrayTask.sh

[Install]
WantedBy = multi-user.target
```

按 esc 退出编辑模式，输入 :wq 保存退出

依次输入以下指令配置自启动服务
```bash
sudo systemctl daemon-reload
sudo systemctl enable sunray.service
sudo systemctl start sunray.service
```

检查服务是否开启，输入：
```bash
sudo systemctl status sunray.service
```

终端输出为：
```bash
PRR@PR-VIO:~$ sudo systemctl status sunray.service
● sunray.service - Start Sunray Task
     Loaded: loaded (/etc/systemd/system/sunray.service; enabled; vendor preset: enabled)
     Active: active (running) since Wed 2025-08-20 17:54:32 CST; 18min ago
   Main PID: 374 (SunrayTask.sh)
     CGroup: /system.slice/sunray.service
             ├─374 /bin/bash /home/PRR/Sunray/General_Module/sunray_viobot_unit/SunrayTask.sh
             ├─813 /usr/bin/python3 /opt/ros/noetic/bin/roslaunch sunray_viobot_unit auto_start.launch
             ├─836 /opt/ros/noetic/lib/mavros/mavros_node __name:=mavros __log:=/home/PRR/.ros/log/b018bafc-7dab-11f0-98f4-b5a00c8628ff/uav2-mavros-1.log
             ├─837 /home/PRR/Sunray/devel/lib/sunray_uav_control/external_fusion_node __name:=external_fusion __log:=/home/PRR/.ros/log/b018bafc-7dab-11f0-98f4-b5a00c8628ff/external_fusion-2>
             ├─838 /home/PRR/Sunray/devel/lib/sunray_uav_control/mocap_odom_node __name:=mocap_odom_node __log:=/home/PRR/.ros/log/b018bafc-7dab-11f0-98f4-b5a00c8628ff/mocap_odom_node-3.log
             └─839 /home/PRR/Sunray/devel/lib/sunray_viobot_unit/sunray_viobot_unit_node __name:=mav_link __log:=/home/PRR/.ros/log/b018bafc-7dab-11f0-98f4-b5a00c8628ff/mav_link-4.log

Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  EXT_ATT [X Y Z]: +0.00 +0.00 -34.54 [deg]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  Error between Vision Pose & PX4 State: 
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  POS_ERR [X Y Z]: -0.27 +1.45 +0.00 [m]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  VEL_ERR [X Y Z]: -0.01 -0.00 +0.00 [m/s]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  ATT_ERR [ YAW ]: -159.04 [deg]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  >>>>> CONTROL ERROR
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  POS CONTROL ERROR [X Y Z norm]: +nan +nan +nan +nan [ m ]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  VEL CONTROL ERROR [X Y Z]: +nan +nan +nan [m/s]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  ATT CONTROL ERROR [X Y Z]: -0.39 +1.31 -0.00 [deg]
Aug 20 18:12:50 PR-VIO SunrayTask.sh[837]:  Warning: The error between external state and px4 state is too large!
lines 1-22/22 (END)
```
说明服务启动成功

## 问题解答

### 如果出现服务无法正常开启：

检查SunrayTask.sh这个文件的绝对路径是不是：/home/PRR/Sunray/General_Module/sunray_viobot_unit/SunrayTask.sh，如果不是，修改/etc/systemd/system/sunray.service文件里 ExecStart 这个键的值，使他指向SunrayTask.sh的绝对路径

### 停止并禁用自启动：

```bash
sudo systemctl stop sunray.service
sudo systemctl disable sunray.service
```

### 查看输出日志：
```bash
sudo journalctl -u sunray.service
```
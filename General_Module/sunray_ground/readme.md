# sunray_ground

#### 介绍
思锐无人机仿真与实践开发平台----地面站模块

#### 运行环境
- ROS Noetic
- Python 3.x
- PyQt5
- 网络连接
- pip install PyQt5 

#### 运行
```shell
## 运行
roslaunch sunray_ground ground_control.launch

## 参数设置
tcp_ip 对应无人机机载端ip地址
udp_ip 对应地面站ip地址
tcp_port 对应无人机机载端端口
udp_port 对应地面站端口

```

#### 地面站UI使用
```shell
cd scripts
python3 ui.py

#配置 ui.py -->line34  第一个为机载端ip地址、端口，第二个为地面站ip地址、端口 要与launch文件中一致
    server_address = ('192.168.0.29', 8969)
    self.tcpSocket = TcpSocketHandler(server_address)
    server_address = ('192.168.0.15', 8968)
    self.udpScket =  UdpReceiver(server_address)

```

# sunray_viobot

- `sunray_viobot.h`及`sunray_viobot.cpp`定义了`VIOBOT`类
    - `VIOBOT`类订阅了stereo2算法发布的关键话题
- `sunray_viobot_node.cpp`调用了`VIOBOT`类，是一个使用示例
    - `roslaunch sunray_viobot sunray_viobot.launch` 用于启动该节点
- `viobot_demo.cpp`是原始文件，其中订阅了所有VIOBOT话题

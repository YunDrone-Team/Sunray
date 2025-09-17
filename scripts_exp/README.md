# Tmux是什么？

Tmux是一个终端复用器（terminal multiplexer），可以在一个终端窗口中运行多个会话（session），每个会话可以有多个窗口（window），每个窗口可以有多个窗格（pane）。

详见博客：https://blog.csdn.net/CSSDCC/article/details/121231906

# sunray_tmux说明

运行install_dependencies.sh脚本时，会自动安装tmux同时配置鼠标可用
需要自行配置tmux可以修改~/.tmux.conf文件
具体操作查看文档：https://think.leftshadow.com/docs/tmux/config/

# 常用的指令

## 关闭tmux
```bash
tmux kill-session -t sunray_tmux
```

## 列举所有tmux会话
```bash
tmux ls
```

## 连接tmux会话
```bash
tmux attach -t sunray_tmux
```

## 列出当前所有 Tmux 会话的信息
```bash
tmux info
```

## 重新加载当前的 Tmux 配置
```bash
$ tmux source-file ~/.tmux.conf
```

## 切换窗口
- ctrl+b c: 创建一个新窗口（状态栏会显示多个窗口的信息）
- ctrl+b p: 切换到上一个窗口（按照状态栏的顺序）
- ctrl+b n: 切换到下一个窗口
- ctrl+b w: 从列表中选择窗口（这个最好用）

## 打印当前tmux会话的所有窗口和窗格的名称
```bash
tmux list-panes -a -F "#{session_name}:#{window_name}.#{pane_index}"
# 打印结果：
# sunray_tmux:main.0
# sunray_tmux:main.1
# sunray_tmux:main.2
# sunray_tmux:main.3
# sunray_tmux:main.4
```

# sunray_tmux模板

```bash
#!/bin/bash

# 引入 TMUX 会话管理模块
source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# ===================== 配置区域 =====================
UAV_ID=2    # 无人机ID 不配置的话默认是1
SESSION_NAME=sunray_tmux  # 会话名称，统一使用sunray_tmux
FIRST_WINDOW="main.2" # 最后选择的窗口和窗格（格式：窗口名.窗格索引） 聚焦在第三行代码所在的终端
LAYOUT="even-horizontal" # 可选: tiled, even-horizontal, even-vertical, main-horizontal, main-vertical,默认 even-horizontal

# 自定义命令配置
declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 3 && roslaunch sunray_uav_control external_fusion.launch external_source:=4 enable_rviz:=false uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
    "
    # 一个窗口只能分六个窗格,超过六个窗格需要新建窗口,格式如下:
    # ["窗口名字"]="
    #     命令1
    #     命令2
    #     命令3
    # "
)
# 这样会创建四个窗口,名字如下,可用于 -t 的参数:
# sunray_tmux:main.0
# sunray_tmux:main.1
# sunray_tmux:main.2
# sunray_tmux:main.3
# 新建的窗口名字格式是这样的
# sunray_tmux:窗口名字.0
# sunray_tmux:窗口名字.1
# sunray_tmux:窗口名字.2

# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session

```

## 纯净版:
```bash
#!/bin/bash

# 引入 TMUX 会话管理模块
source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.0"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 3 && roslaunch sunray_uav_control external_fusion.launch external_source:=4 enable_rviz:=false uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
    "
    # ["窗口名字"]="
    #     命令1
    #     命令2
    #     命令3
    # "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session

```

## 动态配置

```bash
#!/bin/bash

source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# 根据参数动态配置
if [ "$1" == "uav1" ]; then
    UAV_ID=1
    SESSION_NAME="uav1_control"
elif [ "$1" == "uav2" ]; then
    UAV_ID=2
    SESSION_NAME="uav2_control"
fi

create_tmux_session
```

## 批量创建

```bash
#!/bin/bash

source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# 创建多个会话
for id in 1 2 3; do
    UAV_ID=$id
    SESSION_NAME="uav${id}_session"
    create_tmux_session
    echo "已创建会话: $SESSION_NAME"
done
```
 
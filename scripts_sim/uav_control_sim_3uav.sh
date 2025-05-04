#!/bin/bash
# 脚本：无人机控制测试脚本(3机)
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_3uav.launch vehicle:=sunray150_with_mid360; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator swarm_sim_rviz.launch; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \


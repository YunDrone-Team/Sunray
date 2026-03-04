#!/bin/bash
num=2

# start gazebo and ugv control
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_ugv_control wheeltec_robot.launch ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1 location_source:=2 odom_topic:=/sunray/odometry; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"' \

# start ego planner and goal2swarm
gnome-terminal --window -e 'bash -c "sleep 5.0; roslaunch sunray_planner_utils sunray_ego_ugv_swarm.launch ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 7.0; roslaunch sunray_planner_utils positionCmd2sunrayugv_swarm.launch; exec bash"' \
--tab -e 'bash -c "sleep 7.0; roslaunch sunray_planner_utils goal2swarm_ugv.launch uav_num:=${num} offset:=1.5 goal_topic:=goal use_hight:=false; exec bash"' \


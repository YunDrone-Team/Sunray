#!/bin/bash
# start gazebo
gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch web_cam web_cam.launch ; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control waypoint_mission_node.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"'

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_tutorial landmark_detection.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; rosrun rqt_image_view rqt_image_view; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_tutorial auto_land_by_pose.launch; exec bash"' \
# --tab -e 'bash -c "sleep 8.0; roslaunch sunray_tutorial auto_land.launch; exec bash"' \
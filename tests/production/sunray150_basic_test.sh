#!/bin/bash

#sim:
# gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_test_basic.launch; exec bash"' \
# --tab -e 'bash -c "sleep 1.0; roslaunch sunray_simulator sim_rviz.launch; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2 enable_rviz:=false; exec bash"' \
# --tab -e 'bash -c "sleep 3.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_test detection_dual.launch; exec bash"' \

# gnome-terminal --window -e 'bash -c "sleep 2.0; rosrun sunray_test test.py; exec bash"' \

# exp:
gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_test web_cam_dual.launch ; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion.launch external_source:=3; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_test detection_dual.launch; exec bash"' \

gnome-terminal --window -e  'bash -c "sleep 2.0; rosrun sunray_test test.py; exec bash"' \
--tab -e 'bash -c "sleep 5.0; rosrun rqt_image_view rqt_image_view; exec bash"' \


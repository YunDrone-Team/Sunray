#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch 2025_uav_competiton_demo position_pub.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2 enable_rviz:=true; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \

# gnome-terminal --window -e  'bash -c "sleep 15.0; roslaunch sunray_uav_control waypoint.launch ; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 8.0; roslaunch 2025_uav_competiton_demo competition_demo.launch; exec bash"' \
# gnome-terminal --window -e 'bash -c "sleep 8.0; rosrun future_2025_score future_2025_score.py; exec bash"'
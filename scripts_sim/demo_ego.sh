#!/bin/bash
# start gazebo
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_ego.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_ego_sim2.launch; exec bash"' \

# gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_ego_sim2.launch; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_map_generator map_generator.launch map_name:=planning_test rviz_enable:=false; exec bash"' \



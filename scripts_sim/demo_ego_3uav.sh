#!/bin/bash
# start gazebo
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_ego_3uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator swarm_sim_rviz.launch; exec bash"' \

# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

# gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_TV sunray_ego_sim.launch; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_map_generator map_generator.launch map_name:=planning_test rviz_enable:=false; exec bash"' \



#!/bin/bash
# start gazebo
# start ugv control
# start map

gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_1ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_ugv_control ugv_control_sim.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_TV sunray_ego_sim_ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_map_generator map_generator.launch map_name:=planning_test rviz_enable:=false; exec bash"' \

#!/bin/bash
gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_tutorial landmark_detection.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_tutorial auto_land_by_pose.launch; exec bash"' \

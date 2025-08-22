#!/bin/bash
sleep 15
source /home/PRR/Sunray/devel/setup.sh
# roslaunch sunray_viobot_unit auto_start.launch
roslaunch sunray_viobot_unit mavlink.launch

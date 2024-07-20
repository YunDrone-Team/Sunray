#!/bin/bash
# source /home/dji/Kongdijiqun/devel/setup.bash

sudo chmod 777 /dev/ttyACM0 & sleep 2s;
roslaunch sunray_viobot sunray_viobot.launch & sleep 2s;
roslaunch sunray_wheeltec_robot wheeltec_robot.launch


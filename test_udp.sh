#!/bin/bash
###
 # @Author: Yuhua.Qi fatmoonqyp@126.com
 # @Date: 2023-11-05 20:11:13
 # @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 # @LastEditTime: 2023-11-07 17:44:36
 # @FilePath: /Prometheus/home/amov/sunray/test_main.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

gnome-terminal --window -e 'bash -c "roslaunch communication uav_udp_broadcast.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch communication ugv_udp_broadcast.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun uav_control fake_ugv; exec bash"' \



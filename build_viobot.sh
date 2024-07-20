#!/bin/bash
###
 # @Author: Yuhua.Qi fatmoonqyp@126.com
 # @Date: 2024-06-21 12:01:39
 # @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 # @LastEditTime: 2024-06-21 12:21:41
 # @FilePath: /Prometheus/home/amov/Sunray/build_viobot.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

# 编译基础模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译VIOBOT模块
catkin_make --source sunray_viobot --build build/sunray_viobot

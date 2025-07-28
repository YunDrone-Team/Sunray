#!/bin/bash

# 编译ego-planner-swarm模块
catkin_make --source External_Module/ego-planner-swarm --build build/ego-planner
# 编译simulator_utils模块
catkin_make --source Simulation/simulator_utils --build build/simulator_utils
# 编译sunray_planner模块
catkin_make --source General_Module/sunray_planner_utils --build build/sunray_planner_utils
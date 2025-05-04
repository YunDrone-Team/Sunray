#!/bin/bash

# 编译ego-planner-swarm模块
catkin_make --source External_Module/ego-planner-swarm --build build/ego-planner
# 编译sunray_planner模块
catkin_make --source General_Module/sunray_planner --build build/sunray_planner
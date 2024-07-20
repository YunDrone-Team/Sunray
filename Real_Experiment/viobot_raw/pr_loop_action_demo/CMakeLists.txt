cmake_minimum_required(VERSION 2.8.3)
project(pr_loop_action)

set(CMAKE_BUILD_TYPE "Release")

# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)

# -DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosmsg
    std_msgs
    roslib
    actionlib
    actionlib_msgs
    loop_action
    message_generation
    message_runtime
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(add_keyframe_node
    src/pose_graph_action_demo.cpp
)

add_dependencies(add_keyframe_node ${${PROJECT_NAME}_EXPORTTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
target_link_libraries(add_keyframe_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## 添加可执行文件或者库文件，Mark executables and/or libraries for installation
install(TARGETS add_keyframe_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

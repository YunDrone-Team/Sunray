cmake_minimum_required(VERSION 3.0.2)
project(mav_link)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
)

catkin_package()

include_directories(
  include
  include/mavlink/include/mavlink/v2.0/c_library_v2/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/autopilot_interface.cpp
  src/mavlink_control.cpp
  src/serial_port.cpp
  src/mav_link_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

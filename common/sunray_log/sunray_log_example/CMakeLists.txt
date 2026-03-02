cmake_minimum_required(VERSION 3.0.2)
project(sunray_log_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../spdlog/include
)

# 将 localization_example.cpp 编译为独立库
add_library(localization_example
  src/localization_example.cpp
)

target_link_libraries(localization_example
  ${catkin_LIBRARIES}
)

# main.cpp 链接上面的库
add_executable(localization_example_node
  src/main.cpp
)

add_dependencies(localization_example_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(localization_example_node
  localization_example
  ${catkin_LIBRARIES}
)

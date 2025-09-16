cmake_minimum_required(VERSION 3.0.2)
project(undistort_image)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package( )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(undistort_image src/undistort_image.cpp)
target_link_libraries(undistort_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})




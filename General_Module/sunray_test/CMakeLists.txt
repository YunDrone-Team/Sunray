cmake_minimum_required(VERSION 3.0.2)
project(sunray_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  sunray_msgs
)

catkin_install_python(PROGRAMS
  scripts/run_suite.py
  scripts/internal_run.py
  scripts/show_config.py
  scripts/run_scenario.py
  scripts/generate_report.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

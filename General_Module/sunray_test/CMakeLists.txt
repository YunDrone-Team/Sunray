cmake_minimum_required(VERSION 3.0.2)
project(sunray_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  sunray_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  sunray_msgs
)

catkin_install_python(PROGRAMS
  scripts/livox_mid360_autoconfig.py
  scripts/run_test_dashboard.py
  scripts/generate_report.py
  scripts/vrpn_server_check.py
  scripts/camera_launch_guard.py
  scripts/camera_watchdog.py
  scripts/ensure_ros_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/ensure_runtime_deps.sh
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_dashboard_model.py)
  catkin_add_nosetests(test/test_dashboard_history.py)
  catkin_add_nosetests(test/test_dashboard_console.py)
  catkin_add_nosetests(test/test_dashboard_cli.py)
  catkin_add_nosetests(test/test_dashboard_tui.py)
  catkin_add_nosetests(test/test_dashboard_tools.py)
  catkin_add_nosetests(test/test_livox_mid360_autoconfig.py)
endif()

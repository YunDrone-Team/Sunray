//
#include <iostream>
#include <bitset>
#include <signal.h>

// Eigen
#include <Eigen/Eigen>

// ROS话题消息头文件

// sunray_msgs
#include <sunray_msgs/ExternalOdom.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>
#include <sunray_msgs/PX4State.h>
#include <sunray_msgs/TextInfo.h>
#include <sunray_msgs/UAVWayPoint.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVState.h>

#include <sunray_msgs/PositionCommand.h>
#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/TargetMsg.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include <sunray_msgs/RcState.h>
#include <sunray_msgs/OrcaSetup.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/OrcaCmd.h>

// std_msgs
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

// sensor_msgs
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// geometry_msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

// mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/GPSRAW.h>

// nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// others
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include <tf2_ros/transform_listener.h>
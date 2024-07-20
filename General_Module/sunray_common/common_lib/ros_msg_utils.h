// ROS话题消息头文件

// sunray_msgs
#include <sunray_msgs/ExternalOdom.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>
#include <sunray_msgs/TextInfo.h>

// #include <sunray_msgs/FMTState.h>
#include <sunray_msgs/Heartbeat.h>
#include <sunray_msgs/Attitude.h>
#include <sunray_msgs/LocalPositionNED.h>
#include <sunray_msgs/ArmCmd.h>
#include <sunray_msgs/AttitudeSetpoint.h>
#include <sunray_msgs/GlobalPositionSetpoint.h>
#include <sunray_msgs/LocalPositionSetpoint.h>
#include <sunray_msgs/VisionPositionEstimate.h>
#include <sunray_msgs/Target.h>

// std_msgs
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Int32.h"

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
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandHome.h>

// nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// others
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include "trajectory_builder.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "takeoff_and_track_circle_node");
	ros::NodeHandle nh;

	trajectoryBuilder tb;
    tb.init(nh);
	tb.circle();
	// tb.stop();
	ros::spin();

	return 0;
}
#include "tracking_control.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "tracking_controller_node");
	ros::NodeHandle nh;
	trackingController tc(nh);
	ros::spin();

	return 0;
}
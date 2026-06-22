#include <ros/ros.h>
#include <navigation_runner/safeAction.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "safe_action_node");
	ros::NodeHandle nh;
	safeAction::Agent d (nh);
	ros::spin();
	return 0;
}
#include "changing_formation.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "changing_formation_node");
	ros::NodeHandle nh;
	changingFormation tc(nh);
	ros::spin();

	return 0;
}
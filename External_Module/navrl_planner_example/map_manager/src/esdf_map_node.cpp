#include <ros/ros.h>
#include <map_manager/ESDFMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "ESDF_map_node");
	ros::NodeHandle nh;

	mapManager::ESDFMap m;
	m.initMap(nh);

	ros::spin();

	return 0;
}
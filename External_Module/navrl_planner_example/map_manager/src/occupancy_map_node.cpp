#include <ros/ros.h>
#include <map_manager/occupancyMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "occupancy_map_node");
	ros::NodeHandle nh;

	mapManager::occMap m;
	m.initMap(nh);

	ros::spin();

	return 0;
}
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  cout << GREEN << "["<< ros::this_node::getName() << "] -- EGO-planner-swarm INIT." << TAIL << endl;

  // ego-planner主程序入口
  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  ros::spin();

  return 0;
}

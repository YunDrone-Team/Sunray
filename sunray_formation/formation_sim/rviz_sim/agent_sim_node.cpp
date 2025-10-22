#include <ros/ros.h>
#include "agent_sim.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_sim_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    sleep(2.0);
    
    // rmtt仿真器
    AGENT_SIM agent_sim;
    agent_sim.init(nh);

    sleep(5.0);

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        agent_sim.mainloop();
        // 休眠
        rate.sleep();
    }

    return 0;
}

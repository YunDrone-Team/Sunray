#include "VioBot.h"

// 中断信号
void mySigintHandler(int sig)
{
    ROS_INFO("[viobot_node] exit...");
    mavlink_deinit();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viobot_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(50.0);

    Viobot viobot;
    viobot.init(nh);

    // 中断信号注册
    signal(SIGINT, mySigintHandler);

    mavlink_init(viobot.getUartName().c_str(), viobot.getBaudrate());
    mavlink_send_odometry_thread(); // 先保存再在另外的线程里面发送

    while (ros::ok())
    {
        ros::spinOnce();
        mavlink_save_odometry(viobot.getMavlinkOdom()); // 定时把里程计数据保存到缓冲区
        rate.sleep();
    }

    return 0;
}
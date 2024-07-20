/*
 * @Author: Yuhua.Qi fatmoonqyp@126.com
 * @Date: 2024-03-05 14:26:17
 * @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 * @LastEditTime: 2024-03-05 15:00:57
 * @FilePath: /Prometheus/home/amov/Sunray-Comm/ugv_planning/src/ugv_planing_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>

#include "planning_fsm.h"

using namespace ugv_planning;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ugv_planning_node");

    ros::NodeHandle nh("~");

    Planning_FSM planning_fsm;
    planning_fsm.init(nh);

    ros::spin();

    return 0;
}


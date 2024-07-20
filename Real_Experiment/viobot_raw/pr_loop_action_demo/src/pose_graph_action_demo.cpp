#include <actionlib/client/simple_action_client.h>
#include "loop_action/KeyFrameHandleAction.h"   

typedef actionlib::SimpleActionClient<loop_action::KeyFrameHandleAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState &state,
            const loop_action::KeyFrameHandleResultConstPtr &result)
{
    ROS_INFO("keyframe num: %d", result->keyframe_num); // 返回当前关键帧的数量
    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Send!");
}

// 收到feedback后调用的回调函数
void feedbackCb(const loop_action::KeyFrameHandleFeedbackConstPtr &feedback)
{
    ROS_INFO("Waiting!");
}

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        printf("请输入:rosrun pr_loop_action add_keyframe_node 1(or 2)!\n"),exit(0);
    }
    ros::init(argc, argv, "posegraph_action_client");
    // 定义一个客户端
    Client client("/pr_loop/keyframe_action", true);
    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    // 创建一个action的goal
    loop_action::KeyFrameHandleGoal goal;
    goal.function = atoi(argv[1]);   // 1：添加关键帧   2：保存全部关键帧
    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ros::spin();
    return 0;
}
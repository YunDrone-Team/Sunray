#include <ros/ros.h>
#include <thread>
#include <iostream>

#include <std_msgs/Bool.h>

#include "system_ctrl/algo_ctrl.h"
#include "system_ctrl/algo_status.h"
#include "system_ctrl/viobot_ctrl.h"

ros::Publisher pub_vio_ctrl;
ros::Publisher pub_sys_ctrl;
ros::Publisher pub_mono_ctrl;
ros::Publisher pub_stereo1_ctrl;
ros::Publisher pub_stereo2_ctrl;
ros::Publisher pub_for_bag;

void algo_status_callback(const system_ctrl::algo_status::ConstPtr &msg){
    std::cout << "algo_status: " << msg->algo_status << std::endl;
}

void sys_status_callback(const system_ctrl::viobot_ctrl::ConstPtr &msg){
    ROS_INFO("sys_status:");
    if(msg->image_select == 1) std::cout << "image_select: " << "left" << std::endl;
    else if(msg->image_select == 2) std::cout << "image_select: " << "right" << std::endl;
    else if(msg->image_select == 3) std::cout << "image_select: " << "left and right" << std::endl;
    else std::cout << "image_select: " << "OFF" << std::endl;

    if(msg->imu_raw == false) std::cout << "imu_raw: " << "OFF" << std::endl;
    else std::cout << "imu_raw: " << "ON" << std::endl;

    if(msg->tof_depth == false) std::cout << "tof_depth: " << "OFF" << std::endl;
    else std::cout << "tof_depth: " << "ON" << std::endl;

    if(msg->tof_enable == false) std::cout << "tof_enable: " << "OFF" << std::endl;
    else std::cout << "tof_enable: " << "ON" << std::endl;

    if(msg->tof_amp == false) std::cout << "tof_amp: " << "OFF" << std::endl;
    else std::cout << "tof_amp: " << "ON" << std::endl;

    if(msg->light == false) std::cout << "light: " << "OFF" << std::endl;
    else std::cout << "light: " << "ON" << std::endl;
}

void key_ctrl(){
    system_ctrl::viobot_ctrl viobot_set;
    viobot_set.image_select = 0;
    viobot_set.imu_raw = false;
    viobot_set.tof_enable = false;
    viobot_set.tof_amp = false;
    viobot_set.tof_depth = false;
    viobot_set.light = false;

    system_ctrl::algo_ctrl algo_set;
    algo_set.algo_enable = false;
    algo_set.algo_reboot = false;
    algo_set.algo_reset = false;

    ros::Rate r(10);
    int v;

    while(ros::ok()){
        std::cin >> v;
        if(v == 1){//设置时需要注意其他的状态位的情况
            ROS_INFO("algo_enable");
            algo_set.algo_enable = true;
            pub_vio_ctrl.publish(algo_set);
        }
        else if(v == 2){
            ROS_INFO("algo_disable");
            algo_set.algo_enable = false;
            pub_vio_ctrl.publish(algo_set);
        }
        if(v == 3){
            ROS_INFO("tof_enable");
            viobot_set.tof_enable = true;
            pub_sys_ctrl.publish(viobot_set);
        }
        if(v == 4){
            ROS_INFO("tof_disable");
            viobot_set.tof_enable = false;
            pub_sys_ctrl.publish(viobot_set);
        }
        if(v == 5){
            ROS_INFO("light_enable");
            viobot_set.light = true;
            pub_sys_ctrl.publish(viobot_set);
        }
        if(v == 6){
            ROS_INFO("light_disable");
            viobot_set.light = false;
            pub_sys_ctrl.publish(viobot_set);
        }
        if(v == 7){
            ROS_INFO("for_bag");
            std_msgs::Bool for_bag_flag;
            for_bag_flag.data = true;
            pub_for_bag.publish(for_bag_flag);
        }
        if(v == 8){
            ROS_INFO("no_for_bag");
            std_msgs::Bool for_bag_flag;
            for_bag_flag.data = false;
            pub_for_bag.publish(for_bag_flag);
        }
        r.sleep();
        ros::spinOnce(); 
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Viobot_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber sub_algo_status = nh.subscribe("/algo_status", 2, algo_status_callback);
    ros::Subscriber sub_sys_status = nh.subscribe("/sys_status", 2, sys_status_callback);
    
    pub_sys_ctrl = nh.advertise<system_ctrl::viobot_ctrl>("/system_ctrl", 2);
    pub_for_bag = nh.advertise<std_msgs::Bool>("/for_bag", 2);
    //单目版本
    pub_vio_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/vio_ctrl", 2);

    //双目版本的三个算法控制，只能开其中一个，需要关闭之前开了的算法才能开另一个算法
    pub_mono_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/mono_ctrl", 2);
    pub_stereo1_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/stereo1_ctrl", 2);
    pub_stereo2_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/stereo2_ctrl", 2);

    std::thread keyboard_ctrl{key_ctrl};
    ros::spin(); 
    keyboard_ctrl.join();
    ros::shutdown();
    return 0;
}

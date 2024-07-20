#ifndef MSG_STATUS_H
#define MSG_STATUS_H

//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <signal.h>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions

#include "common/mavlink.h"
#include "printf_utils.h"
#include "enum_utils.h"


using namespace std;

class MSG_STATUS
{
    public:
        //构造函数
        MSG_STATUS(){};

        // id
        int msg_id{0};

        // ------------------分割线：以下是收到消息状态-------------------
        // 收到消息的个数
        int recv_msg_count{0};
        // 收到上一帧消息的时间
        ros::Time last_recv_msg_time;
        // 收到当前帧消息的时间
        ros::Time recv_msg_time;
        // 收到消息的时间间隔，单位：秒
        double recv_time_diff_now{0.0};
        // 收到消息的平均时间间隔，单位：秒
        double recv_time_diff_average{0.0};
        // 收到信息的频率，单位：Hz
        double recv_msg_frequency{0.0};

        void recv_new_msg(ros::Time time)
        {
            recv_msg_time = time;
            // 计算时间差
            if(recv_msg_count == 0)
            {
                recv_time_diff_now = 0.0;
                recv_time_diff_average = recv_time_diff_now;
            }else
            {
                recv_time_diff_now = (recv_msg_time - last_recv_msg_time).toNSec()/1000000000.0;
                recv_time_diff_average = (recv_time_diff_average * recv_msg_count + recv_time_diff_now)/(recv_msg_count + 1);
            }
            // 计算频率
            recv_msg_frequency = 1.0 / recv_time_diff_average; 
            // 计数
            recv_msg_count = recv_msg_count + 1;
            // 将当前消息时间推入上一时刻消息
            last_recv_msg_time = recv_msg_time;
        };
        
        double get_recv_msg_frequency()
        {
            return recv_msg_frequency;
        };

        int get_recv_msg_count()
        {
            return recv_msg_count;
        };

        // ------------------分割线：以下是发送消息状态-------------------
        // 发送消息的个数
        int send_msg_count{0};
        // 发送上一帧消息的时间
        ros::Time last_send_msg_time;
        // 发送当前帧消息的时间
        ros::Time send_msg_time;
        // 发送消息的时间间隔，单位：秒
        double send_time_diff_now{0.0};
        // 发送消息的平均时间间隔，单位：秒
        double send_time_diff_average{0.0};
        // 发送信息的频率，单位：Hz
        double send_msg_frequency{0.0};

        void send_new_msg(ros::Time time)
        {
            send_msg_time = time;
            // 计算时间差
            if(send_msg_count == 0)
            {
                send_time_diff_now = 0.0;
                send_time_diff_average = send_time_diff_now;
            }else
            {
                send_time_diff_now = (send_msg_time - last_send_msg_time).toNSec()/1000000000.0;
                send_time_diff_average = (send_time_diff_average * send_msg_count + send_time_diff_now)/(send_msg_count + 1);
            }
            // 计算频率
            send_msg_frequency = 1.0 / send_time_diff_average; 
            // 计数
            send_msg_count = send_msg_count + 1;
            // 将当前消息时间推入上一时刻消息
            last_send_msg_time = send_msg_time;
        };

        double get_send_msg_frequency()
        {
            return send_msg_frequency;
        };

        int get_send_msg_count()
        {
            return send_msg_count;
        };
    private:        
  
};
#endif

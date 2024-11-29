#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include "printf_utils.h"

class RC_Input
{
public:
	float ch[4];
	float last_ch[4];
	RC_Input() {};
	void init(ros::NodeHandle &nh);
	void handle_rc_data(const mavros_msgs::RCIn::ConstPtr& pMsg);
	void printf_info();

private:
	int channel_arm;
	int channel_mode;
	int channel_land;
	int channel_kill;
	float value_arm;
	float value_mode_init;
	float value_mode_rc;
	float value_mode_cmd;
	float value_land;
	float value_kill;
	float arm_channel_value;
	float mode_channel_value;
	float land_channel_value;
	float kill_channel_value;
	float last_arm_channel_value;
	float last_mode_channel_value;
	float last_land_channel_value;
	float last_kill_channel_value;
	bool value_init;
	std::string rc_topic;

	mavros_msgs::RCIn msg;
	ros::Time rcv_stamp; // 收到遥控器消息的时间
	ros::Subscriber rc_sub;
	ros::Publisher setup_pub;

	bool check_validity();
};

void RC_Input::handle_rc_data(const mavros_msgs::RCIn::ConstPtr& pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();
	ch[0] = msg.channels[0];
	ch[1] = msg.channels[1];
	ch[2] = msg.channels[2];
	ch[3] = msg.channels[3];

	arm_channel_value = (msg.channels[channel_arm - 1] - 1500) / 500.0;
	mode_channel_value = (msg.channels[channel_mode - 1] - 1500) / 500.0;
	land_channel_value = (msg.channels[channel_land - 1] - 1500) / 500.0;
	land_channel_value = (msg.channels[channel_kill - 1] - 1500) / 500.0;

	if (!value_init)
	{
		last_ch[0] = ch[0];
		last_ch[1] = ch[1];
		last_ch[2] = ch[2];
		last_ch[3] = ch[3];
		last_arm_channel_value = arm_channel_value;
		last_mode_channel_value = mode_channel_value;
		last_land_channel_value = land_channel_value;
		last_kill_channel_value = kill_channel_value;
		value_init = true;
		return;
	}

	if (!check_validity())
	{
		if (abs(arm_channel_value - last_arm_channel_value) > 0.25)
		{
			if (abs(arm_channel_value - value_arm) < 0.25)
			{
				// 解锁
			}
			else if (abs(arm_channel_value - value_arm) < 0.25)
			{
				// 锁定
			}
		}
		if (abs(mode_channel_value - last_mode_channel_value) > 0.25)
		{
			if (abs(mode_channel_value - value_mode_init) < 0.25)
			{
				// INIT模式
			}
			else if (abs(mode_channel_value - value_mode_rc) < 0.25)
			{
				// RC_CONTROL模式
			}
			else if (abs(mode_channel_value - value_mode_cmd) < 0.25)
			{
				// CMD_CONTROL模式
			}
		}
		if (abs(land_channel_value - last_land_channel_value) > 0.25)
		{
			if (abs(land_channel_value - value_land) < 0.25)
			{
				// 降落
			}
		}
		if (abs(kill_channel_value - last_kill_channel_value) > 0.25)
		{
			if (abs(kill_channel_value - value_kill) < 0.25)
			{
				// 紧急停止
			}
		}
	}
	else
	{
		std::cout << "RC Input is invalid!" << std::endl;
	}
}

bool RC_Input::check_validity()
{
	if (arm_channel_value >= -1.1 && arm_channel_value <= 1.1 &&
		mode_channel_value >= -1.1 && mode_channel_value <= 1.1 &&
		kill_channel_value >= -1.1 && kill_channel_value <= 1.1 &&
		kill_channel_value >= -1.1 && kill_channel_value <= 1.1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void RC_Input::init(ros::NodeHandle &nh)
{
	rc_sub = nh.subscribe<mavros_msgs::RCIn>(rc_topic, 10, &RC_Input::handle_rc_data, this);
	setup_pub = nh.advertise<mavros_msgs::RCIn>(rc_topic, 10);
}
#endif

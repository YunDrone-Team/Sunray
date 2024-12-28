#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <sunray_msgs/RcState.h>

class RC_Input
{
public:
	float ch[4];
	float last_ch[4];
	RC_Input() {};
	void init(ros::NodeHandle &nh);
	void handle_rc_data(const mavros_msgs::RCIn::ConstPtr &pMsg);
	void printf_info();
	void set_last_rc_data();

private:
	int uav_id;			   // 无人机编号
	int channel_arm;	   // 解锁通道
	int channel_mode;	   // 模式通道
	int channel_land;	   // 降落通道
	int channel_kill;	   // 紧急停桨通道
	float value_arm;	   // 解锁通道的档位值 二档（-1 1） 三档（-1 0 1） 对应（1000 2000）（1000  1500 2000）
	float value_disarm;	   // 上锁通道的档位值
	float value_mode_init; // INIT模式通道的初始档位值
	float value_mode_rc;   // RC_CONTRIL模式通道的档位值
	float value_mode_cmd;  // CMD模式通道的档位值
	float value_land;	   // 降落通道的档位值
	float value_kill;	   // 紧急停桨通道的档位值
	float arm_channel_value;
	float mode_channel_value;
	float land_channel_value;
	float kill_channel_value;
	float last_arm_channel_value;
	float last_mode_channel_value;
	float last_land_channel_value;
	float last_kill_channel_value;
	bool value_init;	  // 是否已经初始化 接收到数据后置为true
	std::string rc_topic; // 遥控器话题
	std::string uav_name; // 无人机名称
	sunray_msgs::RcState rc_state_msg;

	mavros_msgs::RCIn msg;
	ros::NodeHandle nh_;
	ros::Time rcv_stamp;		 // 收到遥控器消息的时间
	ros::Subscriber rc_sub;		 // 【订阅】遥控器话题
	ros::Publisher rc_state_pub; // 【发布】遥控器状态话题

	bool check_validity();
};

void RC_Input::handle_rc_data(const mavros_msgs::RCIn::ConstPtr &pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();
	ch[0] = msg.channels[0];
	ch[1] = msg.channels[1];
	ch[2] = msg.channels[2];
	ch[3] = msg.channels[3];

	// 归一化数据【-1 1】
	arm_channel_value = (msg.channels[channel_arm - 1] - 1500) / 500.0;
	mode_channel_value = (msg.channels[channel_mode - 1] - 1500) / 500.0;
	land_channel_value = (msg.channels[channel_land - 1] - 1500) / 500.0;
	kill_channel_value = (msg.channels[channel_kill - 1] - 1500) / 500.0;
	if (!value_init)
	{
		set_last_rc_data();
		value_init = true;
		return;
	}
	rc_state_msg.header.stamp = ros::Time::now();
	rc_state_msg.channel[0] = (msg.channels[0] - 1500) / 500.0;
	rc_state_msg.channel[1] = (msg.channels[1] - 1500) / 500.0;
	rc_state_msg.channel[2] = (msg.channels[2] - 1500) / 500.0;
	rc_state_msg.channel[3] = (msg.channels[3] - 1500) / 500.0;
	rc_state_msg.channel[5] = arm_channel_value;
	rc_state_msg.channel[6] = mode_channel_value;
	rc_state_msg.channel[7] = land_channel_value;
	rc_state_msg.channel[8] = kill_channel_value;

	rc_state_msg.arm_state = 0;
	rc_state_msg.mode_state = 0;
	rc_state_msg.land_state = 0;
	rc_state_msg.kill_state = 0;

	if (check_validity())
	{
		// 【0.25】作为检测通道变化的阈值 变化超过0.25则认为通道变化
		if (abs(arm_channel_value - last_arm_channel_value) > 0.25)
		{
			if (abs(arm_channel_value - value_arm) < 0.25)
			{
				// 解锁
				rc_state_msg.arm_state = 2;
			}
			else if (abs(arm_channel_value - value_disarm) < 0.25)
			{
				// 锁定
				rc_state_msg.arm_state = 1;
			}
		}
		if (abs(mode_channel_value - last_mode_channel_value) > 0.25)
		{
			if (abs(mode_channel_value - value_mode_init) < 0.25)
			{
				// INIT模式
				rc_state_msg.mode_state = 1;
			}
			else if (abs(mode_channel_value - value_mode_rc) < 0.25)
			{
				// RC_CONTROL模式
				rc_state_msg.mode_state = 2;
			}
			else if (abs(mode_channel_value - value_mode_cmd) < 0.25)
			{
				// CMD_CONTROL模式
				rc_state_msg.mode_state = 3;
			}
		}
		if (abs(land_channel_value - last_land_channel_value) > 0.25)
		{
			if (abs(land_channel_value - value_land) < 0.25)
			{
				// 降落
				rc_state_msg.land_state = 1;
			}
		}
		if (abs(kill_channel_value - last_kill_channel_value) > 0.25)
		{
			if (abs(kill_channel_value - value_kill) < 0.25)
			{
				// 紧急停止
				rc_state_msg.kill_state = 1;
			}
		}
	}
	else
	{
		std::cout << "RC Input is invalid!" << std::endl;
	}
	rc_state_pub.publish(rc_state_msg);
	set_last_rc_data();
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
	nh_ = nh;
	nh.param<int>("uav_id", uav_id, 1);
	nh.param<int>("channel_arm", channel_arm, 5);
	nh.param<int>("channel_mode", channel_mode, 6);
	nh.param<int>("channel_land", channel_land, 7);
	nh.param<int>("channel_kill", channel_kill, 8);
	nh.param<float>("value_arm", value_arm, 1);
	nh.param<float>("value_disarm", value_disarm, -1);
	nh.param<float>("value_mode_init", value_mode_init, -1);
	nh.param<float>("value_mode_rc", value_mode_rc, 0);
	nh.param<float>("value_mode_cmd", value_mode_cmd, 1);
	nh.param<float>("value_land", value_land, 1);
	nh.param<float>("value_kill", value_kill, 1);
	nh.param<std::string>("rc_topic", rc_topic, "/uav1/sunray/fake_rc_in");
	nh.param<std::string>("uav_name", uav_name, "uav");

	std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
	rc_sub = nh.subscribe<mavros_msgs::RCIn>(rc_topic, 10, &RC_Input::handle_rc_data, this);
	rc_state_pub = nh.advertise<sunray_msgs::RcState>(topic_prefix + "/sunray/rc_state", 10);
}

void RC_Input::set_last_rc_data()
{
	last_ch[0] = ch[0];
	last_ch[1] = ch[1];
	last_ch[2] = ch[2];
	last_ch[3] = ch[3];
	last_arm_channel_value = arm_channel_value;
	last_mode_channel_value = mode_channel_value;
	last_land_channel_value = land_channel_value;
	last_kill_channel_value = kill_channel_value;
}
#endif

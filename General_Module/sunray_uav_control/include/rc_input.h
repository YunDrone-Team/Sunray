#ifndef RC_INPUT_H
#define RC_INPUT_H

#include "ros_msg_utils.h"

class RC_Input
{
public:
	RC_Input(){};
	~RC_Input(){};

	ros::NodeHandle nh;

	float ch[4];			// 油门、滚转、俯仰、偏航四个通道
	float last_ch[4];
	void init(ros::NodeHandle &nh, int uav_id, string uav_name);
	void handle_rc_data(const mavros_msgs::RCIn::ConstPtr &pMsg);
	void printf_info();
	void set_last_rc_data();

private:
	int channel_arm;	   // 解锁通道
	int channel_mode;	   // 模式通道
	int channel_land;	   // 降落通道
	int channel_kill;	   // 紧急停桨通道

	float value_arm;	   // 解锁通道的档位值 
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
	
	mavros_msgs::RCIn msg;
	sunray_msgs::RcState rc_state_msg;

	ros::Subscriber rc_sub;		 // 【订阅】遥控器话题
	ros::Publisher rc_state_pub; // 【发布】遥控器状态话题

	bool check_validity();
};

// 类初始化函数
void RC_Input::init(ros::NodeHandle &nh_, int uav_id = 1, string uav_name = "uav")
{
	nh = nh_;
	
	// 【参数】 解锁通道（2段式开关）
	nh.param<int>("channel_arm", channel_arm, 5);
	// 【参数】 飞行模式切换通道（3段式开关） 
	nh.param<int>("channel_mode", channel_mode, 6);
	// 【参数】 降落（切换到LAND_CONTROL控制模式）通道（2段式开关） 
	nh.param<int>("channel_land", channel_land, 7);
	// 【参数】 紧急上锁（强行停桨）通道（2段式开关） 
	nh.param<int>("channel_kill", channel_kill, 8);

	string topic_prefix = "/" + uav_name + to_string(uav_id);
	// 【订阅】遥控器输入 -  遥控器 -> 飞控 -> mavros -> 本节点
	rc_sub = nh.subscribe<mavros_msgs::RCIn>(topic_prefix + "/sunray/fake_rc_in", 10, &RC_Input::handle_rc_data, this);
	// 【发布】遥控器指令状态 - 本节点 -> uav_control节点
	rc_state_pub = nh.advertise<sunray_msgs::RcState>(topic_prefix + "/sunray/rc_state", 10);

	// 初始化通道读数
	// 2段式开关：遥控器读数为 [1000 2000] 对应[-1 1] 
	// 3段式开关：遥控器读数为 [1000 1500 2000] 对应 [-1 0 1]
	value_arm = 1;
	value_disarm = -1;
	value_mode_init = -1;
	value_mode_rc = 0;
	value_mode_cmd = 1;
	value_land = 1;
	value_kill = 1;
}

// 遥控器输入话题回调函数
void RC_Input::handle_rc_data(const mavros_msgs::RCIn::ConstPtr &pMsg)
{
	msg = *pMsg;

	// 读取油门、滚转、俯仰、偏航四个通道的数据，并进行归一化处理，取值范围是[-1，1]
	ch[0] = (msg.channels[0] - 1500) / 500.0;
	ch[1] = (msg.channels[1] - 1500) / 500.0;
	ch[2] = (msg.channels[2] - 1500) / 500.0;
	ch[3] = (msg.channels[3] - 1500) / 500.0;

	// 读取开关通道的读数，并进行归一化【-1 1】
	arm_channel_value = (msg.channels[channel_arm - 1] - 1500) / 500.0;
	mode_channel_value = (msg.channels[channel_mode - 1] - 1500) / 500.0;
	land_channel_value = (msg.channels[channel_land - 1] - 1500) / 500.0;
	kill_channel_value = (msg.channels[channel_kill - 1] - 1500) / 500.0;

	// 第一次进入进行数据初始化
	if (!value_init)
	{
		set_last_rc_data();
		value_init = true;
		return;
	}

	rc_state_msg.header.stamp = ros::Time::now();
	rc_state_msg.channel[0] = ch[0];
	rc_state_msg.channel[1] = ch[1];
	rc_state_msg.channel[2] = ch[2];
	rc_state_msg.channel[3] = ch[3];
	rc_state_msg.channel[5] = arm_channel_value;
	rc_state_msg.channel[6] = mode_channel_value;
	rc_state_msg.channel[7] = land_channel_value;
	rc_state_msg.channel[8] = kill_channel_value;

	rc_state_msg.arm_state = 0;
	rc_state_msg.mode_state = 0;
	rc_state_msg.land_state = 0;
	rc_state_msg.kill_state = 0;

	// 检查数据有效性后
	if (check_validity())
	{
		// 解锁指令判断：0.25作为检测通道变化的阈值，变化超过0.25则认为通道变化
		if (abs(arm_channel_value - last_arm_channel_value) > 0.25)
		{
			if (abs(arm_channel_value - value_arm) < 0.25)
			{
				// 解锁无人机
				rc_state_msg.arm_state = 2;
			}
			else if (abs(arm_channel_value - value_disarm) < 0.25)
			{
				// 无人机上锁
				rc_state_msg.arm_state = 1;
			}
		}

		// 飞行模式切换指令判断：0.25作为检测通道变化的阈值，变化超过0.25则认为通道变化
		if (abs(mode_channel_value - last_mode_channel_value) > 0.25)
		{
			if (abs(mode_channel_value - value_mode_init) < 0.25)
			{
				// 切换到INIT控制模式
				rc_state_msg.mode_state = 1;
			}
			else if (abs(mode_channel_value - value_mode_rc) < 0.25)
			{
				// 切换到RC_CONTROL控制模式
				rc_state_msg.mode_state = 2;
			}
			else if (abs(mode_channel_value - value_mode_cmd) < 0.25)
			{
				// 切换到CMD_CONTROL控制模式
				rc_state_msg.mode_state = 3;
			}
		}

		// 降落指令判断
		if (abs(land_channel_value - last_land_channel_value) > 0.25)
		{
			if (abs(land_channel_value - value_land) < 0.25)
			{
				// 降落
				rc_state_msg.land_state = 1;
			}
		}

		// 紧急上锁指令判断
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

	// 发布为自定义话题
	rc_state_pub.publish(rc_state_msg);
	// 数据记忆
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
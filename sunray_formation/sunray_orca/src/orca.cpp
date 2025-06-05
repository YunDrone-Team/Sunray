#include "orca.h"

void ORCA::init(ros::NodeHandle &nh)
{
	nh.param<int>("agent_id", agent_id, 1);
	nh.param<std::string>("agent_name", agent_name, "uav");
	// 【参数】智能体数量
	nh.param<int>("agent_num", agent_num, 1);
	// 【参数】最大偏航角速率
	nh.param<float>("max_yaw_rate", max_yaw_rate, M_PI / 3);
	// 【参数】无人机之间的假想感知距离
	nh.param<float>("orca_params/neighborDist", orca_params.neighborDist, 1.5);
	orca_params.maxNeighbors = agent_num;
	// 【参数】数字越大，无人机响应相邻无人机的碰撞越快，但速度可选的自由度越小
	nh.param<float>("orca_params/timeHorizon", orca_params.timeHorizon, 2.0);
	// 【参数】数字越大，无人机响应障碍物的碰撞越快，但速度可选的自由度越小
	nh.param<float>("orca_params/timeHorizonObst", orca_params.timeHorizonObst, 2.0);
	// 【参数】无人机体积半径
	nh.param<float>("orca_params/radius", orca_params.radius, 0.3);
	// 【参数】无人机最大速度
	nh.param<float>("orca_params/maxSpeed", orca_params.maxSpeed, 0.5);
	// 【参数】时间步长 不确定有啥用
	nh.param<float>("orca_params/time_step", orca_params.time_step, 0.1);

	// 初始化订阅器和发布器
	string topic_prefix = "/" + agent_name + std::to_string(agent_id);

	cmd_pub = nh.advertise<sunray_msgs::OrcaCmd>(topic_prefix + "/orca_cmd", 10);
	goal_pub = nh.advertise<visualization_msgs::Marker>(topic_prefix + "/orca/goal", 10);
	for (int i = 0; i < agent_num; i++)
	{
		topic_prefix = "/" + agent_name + std::to_string(i + 1);
		// 【订阅】无人机状态数据
		agent_state_sub[i] = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/orca/agent_state", 1,
															  boost::bind(&ORCA::agent_state_cb, this, _1, i));
		// 【订阅】订阅控制指令
		cmd_sub[i] = nh.subscribe<sunray_msgs::OrcaSetup>(topic_prefix + "/orca/setup", 1,
														  boost::bind(&ORCA::setup_cb, this, _1, i));
	}

	// 打印定时器
	check_timer = nh.createTimer(ros::Duration(1.0), &ORCA::checkAgentState, this);
	orca_state = sunray_msgs::OrcaCmd::INIT;
	agent_state_ready = false;
	goal_reached_printed = false;
	odom_valid.resize(agent_num, false);

	// 【函数】打印参数
	printf_param();

	// ORCA算法初始化 - 添加智能体
	setup_agents();
	// ORCA算法初始化 - 添加障碍物
	// setup_obstacles();

	node_name = ros::this_node::getName();
}

bool ORCA::orca_run()
{
	OrcaCmd.header.stamp = ros::Time::now();
	OrcaCmd.state = orca_state;
	OrcaCmd.goal_pos[0] = goal_pos[0];
	OrcaCmd.goal_pos[1] = goal_pos[1];
	OrcaCmd.goal_pos[2] = goal_pos[2];
	OrcaCmd.goal_yaw = goal_yaw;
	OrcaCmd.linear[2] = 0.0;
	OrcaCmd.angular[0] = 0.0;
	OrcaCmd.angular[1] = 0.0;
	int idx = agent_id - 1;
	if (!start_flag || orca_state == sunray_msgs::OrcaCmd::INIT || !agent_state_ready)
	{
		return false;
	}

	// 判断是否达到目标点附近
	if (!arrived_goal)
	{
		arrived_goal = reachedGoal(idx);
	}

	// 更新RVO中的位置和速度
	for (int i = 0; i < agent_num; ++i)
	{
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pose.pose.position.x, agent_state[i].pose.pose.position.y);
		sim->setAgentPosition(i, pos); // 更新RVO仿真中的位置
		RVO::Vector2 vel = RVO::Vector2(agent_state[i].twist.twist.linear.x, agent_state[i].twist.twist.linear.y);
		sim->setAgentVelocity(i, vel); // 更新RVO仿真中的速度
	}
	if (orca_state == sunray_msgs::OrcaCmd::ARRIVED || orca_state == sunray_msgs::OrcaCmd::STOP)
	{
		OrcaCmd.linear[0] = 0.0;
		OrcaCmd.linear[1] = 0.0;
		OrcaCmd.angular[2] = 0.0;
		cmd_pub.publish(OrcaCmd);
		return true;
	}
	// 计算每一个智能体的期望速度
	sim->computeVel();

	if (arrived_goal)
	{
		if (!goal_reached_printed)
		{
			cout << agent_name + std::to_string(agent_id) << " Arrived." << endl;
			goal_reached_printed = true;
		}
		orca_state = sunray_msgs::OrcaCmd::ARRIVED;
		OrcaCmd.state = orca_state;
		OrcaCmd.linear[0] = 0.0;
		OrcaCmd.linear[1] = 0.0;
		OrcaCmd.angular[2] = 0.0;
		cmd_pub.publish(OrcaCmd);
		return true;
	}
	// 如果没有达到目标点附近，则使用ORCA算法计算期望速度
	else
	{
		// 获得期望速度 注：这个速度是ENU坐标系的，并将其转换为机体系速度指令
		RVO::Vector2 vel = sim->getAgentVelCMD(idx);
		orca_state = sunray_msgs::OrcaCmd::RUN;
		OrcaCmd.state = orca_state;
		OrcaCmd.linear[0] = vel.x();
		OrcaCmd.linear[1] = vel.y();
		// double current_yaw = tf::getYaw(agent_state[idx].pose.pose.orientation);
		double current_yaw = 0;
		// double target_yaw = atan2(vel.y(), vel.x());
		double target_yaw = atan2(goal_pos[1] - agent_state[idx].pose.pose.position.y, goal_pos[0] - agent_state[idx].pose.pose.position.x);
		float angle = this->calculateOptimalTurn(current_yaw, target_yaw);
		OrcaCmd.angular[2] = angle;
		if (angle > 0 && angle > max_yaw_rate)
		{
			OrcaCmd.angular[2] = max_yaw_rate;
		}
		else if (angle < 0 && angle < -max_yaw_rate)
		{
			OrcaCmd.angular[2] = -max_yaw_rate;
		}
		cmd_pub.publish(OrcaCmd);
		return false;
	}
}

void ORCA::setup_agents()
{
	// 设置算法参数
	// sim->setAgentDefaults(1.5f, 10, 2.0f, 2.0f, 0.5f, 0.5f);
	// sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
	sim->setAgentDefaults(orca_params.neighborDist, orca_params.maxNeighbors, orca_params.timeHorizon,
						  orca_params.timeHorizonObst, orca_params.radius, orca_params.maxSpeed);
	// 设置时间间隔（这个似乎没有用？）
	sim->setTimeStep(orca_params.time_step);
	// 添加智能体
	for (int i = 0; i < agent_num; i++)
	{
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pose.pose.position.x, agent_state[i].pose.pose.position.y);
		sim->addAgent(pos);
		cout << BLUE << node_name << ": ORCA add agents_" << i + 1 << " at [" << agent_state[i].pose.pose.position.x << "," << agent_state[i].pose.pose.position.y << "]" << TAIL << endl;
	}

	cout << BLUE << node_name << ": Set agents success!" << TAIL << endl;
}

void ORCA::setup_obstacles()
{
	// 声明障碍物（凸多边形），障碍物建立规则：逆时针依次添加多边形的顶点
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

	// 障碍物示例：中心在原点，边长为1的正方体
	// obstacle1.push_back(RVO::Vector2(0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(0.5f, -0.5f));

	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	// obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	// obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	// obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	// obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

	// 在算法中添加障碍物
	// sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	// sim->addObstacle(obstacle3);
	// sim->addObstacle(obstacle4);

	// 在算法中处理障碍物信息
	sim->processObstacles();

	cout << BLUE << node_name << ":  Set obstacles success!" << TAIL << endl;
}


bool ORCA::reachedGoal(int i)
{
	bool xy_arrived{false}, yaw_arrived{false};
	RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
	float xy_distance = (agent_state[i].pose.pose.position.x - rvo_goal.x()) * (agent_state[i].pose.pose.position.x - rvo_goal.x()) +
						(agent_state[i].pose.pose.position.y - rvo_goal.y()) * (agent_state[i].pose.pose.position.y - rvo_goal.y());
	if (xy_distance < 0.15f * 0.15f)
	{
		xy_arrived = true;
		return true;
	}

	// if (abs(agent_state[i].attitude[2] - 0.0) / M_PI * 180 < 3.0f)
	// {
	// 	yaw_arrived = true;
	// }

	// if (xy_arrived && yaw_arrived)
	// {
	// 	return true;
	// }
	return false;
}

void ORCA::agent_state_cb(const nav_msgs::Odometry::ConstPtr &msg, int i)
{
	agent_state[i] = *msg;
}

void ORCA::setup_cb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int i)
{
	if (msg->cmd == sunray_msgs::OrcaSetup::GOAL)
	{
		sim->setAgentGoal(i, RVO::Vector2(msg->desired_pos[0], msg->desired_pos[1]));

		if (agent_id - 1 == i)
		{
			goal_pos[0] = msg->desired_pos[0];
			goal_pos[1] = msg->desired_pos[1];
			goal_pos[2] = msg->desired_pos[2];
			goal_yaw = msg->desired_yaw;
			goal_reached_printed = false;
			arrived_goal = false;
			start_flag = true;
			visualization_msgs::Marker goal_marker;
			goal_marker.header.frame_id = "world";
			goal_marker.header.stamp = ros::Time::now();
			goal_marker.ns = "goal";
			goal_marker.id = agent_id;
			goal_marker.type = visualization_msgs::Marker::SPHERE;
			goal_marker.action = visualization_msgs::Marker::ADD;
			goal_marker.pose.position.x = goal_pos[0];
			goal_marker.pose.position.y = goal_pos[1];
			goal_marker.pose.position.z = goal_pos[2];
			goal_marker.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
			goal_marker.scale.x = 0.2;
			goal_marker.scale.y = 0.2;
			goal_marker.scale.z = 0.2;
			goal_marker.color.a = 1.0;
			// 根据uav_id生成对应的颜色
			goal_marker.color.r = static_cast<float>(((agent_id) * 123) % 256) / 255.0;
			goal_marker.color.g = static_cast<float>(((agent_id) * 456) % 256) / 255.0;
			goal_marker.color.b = static_cast<float>(((agent_id) * 789) % 256) / 255.0;
			goal_marker.mesh_use_embedded_materials = false;
			goal_pub.publish(goal_marker);
		}
	}
	else if (msg->cmd == sunray_msgs::OrcaSetup::GOAL_RUN)
	{
		sim->setAgentGoal(i, RVO::Vector2(msg->desired_pos[0], msg->desired_pos[1]));

		if (agent_id - 1 == i)
		{
			goal_pos[0] = msg->desired_pos[0];
			goal_pos[1] = msg->desired_pos[1];
			goal_pos[2] = msg->desired_pos[2];
			goal_yaw = msg->desired_yaw;
			orca_state = sunray_msgs::OrcaCmd::RUN;
			goal_reached_printed = false;
			arrived_goal = false;
			start_flag = true;
			visualization_msgs::Marker goal_marker;
			goal_marker.header.frame_id = "world";
			goal_marker.header.stamp = ros::Time::now();
			goal_marker.ns = "goal";
			goal_marker.id = agent_id;
			goal_marker.type = visualization_msgs::Marker::SPHERE;
			goal_marker.action = visualization_msgs::Marker::ADD;
			goal_marker.pose.position.x = goal_pos[0];
			goal_marker.pose.position.y = goal_pos[1];
			goal_marker.pose.position.z = goal_pos[2];
			goal_marker.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
			goal_marker.scale.x = 0.2;
			goal_marker.scale.y = 0.2;
			goal_marker.scale.z = 0.2;
			goal_marker.color.a = 1.0;
			// 根据uav_id生成对应的颜色
			goal_marker.color.r = static_cast<float>(((agent_id) * 123) % 256) / 255.0;
			goal_marker.color.g = static_cast<float>(((agent_id) * 456) % 256) / 255.0;
			goal_marker.color.b = static_cast<float>(((agent_id) * 789) % 256) / 255.0;
			goal_marker.mesh_use_embedded_materials = false;
			goal_pub.publish(goal_marker);
		}
	}
	else if (msg->cmd == sunray_msgs::OrcaSetup::STOP)
	{
		orca_state = sunray_msgs::OrcaCmd::STOP;
	}
	else if (msg->cmd == sunray_msgs::OrcaSetup::RUN)
	{
		orca_state = sunray_msgs::OrcaCmd::RUN;
		start_flag = true;
	}
	else
	{
		cout << RED << node_name << ": Unknown cmd!" << msg->cmd << TAIL << endl;
	}
}

void ORCA::printf_param()
{
	cout << GREEN << ">>>>>>>>>>>>>>>>>>> ORCA Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
	cout << GREEN << "agent_num    : " << agent_num << TAIL << endl;

	// ORCA算法参数
	cout << GREEN << "neighborDist : " << orca_params.neighborDist << " [m]" << TAIL << endl;
	cout << GREEN << "maxNeighbors : " << orca_params.maxNeighbors << TAIL << endl;
	cout << GREEN << "timeHorizon : " << orca_params.timeHorizon << " [s]" << TAIL << endl;
	cout << GREEN << "timeHorizonObst : " << orca_params.timeHorizonObst << " [s]" << TAIL << endl;
	cout << GREEN << "radius : " << orca_params.radius << " [m]" << TAIL << endl;
	cout << GREEN << "maxSpeed : " << orca_params.maxSpeed << " [m/s]" << TAIL << endl;
	cout << GREEN << "time_step : " << orca_params.time_step << " [s]" << TAIL << endl;
}

// 计算最优转向角度差（角速度）
double ORCA::calculateOptimalTurn(double current, double target)
{
	// 计算角度差 (delta_yaw)
	double delta_yaw = target - current;

	// 调整角度差到 [-180, 180] 范围内
	while (delta_yaw > M_PI)
	{
		delta_yaw -= M_PI * 2;
	}
	while (delta_yaw < -M_PI)
	{
		delta_yaw += M_PI * 2;
	}

	// 确定最优旋转方向
	if (delta_yaw > 0.0)
	{
		return delta_yaw;
	}
	else if (delta_yaw < 0.0)
	{
		return -delta_yaw; // 取绝对值
	}
	else
	{
		return 0.0;
	}
}

// 定时检查是否所有无人机/无人车状态都正常
void ORCA::checkAgentState(const ros::TimerEvent &e)
{
	agent_state_ready = true;
	for (int i = 0; i < agent_num; i++)
	{
		odom_valid[i] = true;
		// 检查所有无人机/无人车的里程计回调时间
		if ((ros::Time::now() - agent_state[i].header.stamp).toSec() > 1)
		{
			odom_valid[i] = false;
			agent_state_ready = false;
			cout << RED << node_name << ": Agent " << i + 1 << " has not received odometry data for more than 1 second!" << TAIL << endl;
		}
	}
}
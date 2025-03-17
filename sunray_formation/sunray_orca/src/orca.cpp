#include "orca.h"

void ORCA::init(ros::NodeHandle &nh)
{
	nh.param<int>("uav_id", uav_id, 1);
	nh.param<std::string>("uav_name", uav_name, "uav");
	// 【参数】智能体类型
	nh.param<int>("agent_type", agent_type, 1);
	// 【参数】智能体数量
	nh.param<int>("uav_num", uav_num, 1);
	// 【参数】高度类型 其他:固定高度 1:动态高度
	nh.param<int>("height_type", height_type, 0);
	// 【参数】智能体高度
	nh.param<float>("agent_height", agent_height, 1.0);
	// 【参数】无人机之间的假想感知距离
	nh.param<float>("orca_params/neighborDist", orca_params.neighborDist, 1.5);
	orca_params.maxNeighbors = uav_num;
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
	goal_reached_printed.resize(uav_num, false);
	string topic_prefix = "/" + uav_name + std::to_string(uav_id);
	control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
	for (int i = 1; i < uav_num + 1; i++)
	{
		topic_prefix = "/" + uav_name + std::to_string(i);
		// 【订阅】无人机状态数据
		uav_state_sub[i] = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, boost::bind(&ORCA::uav_state_cb, this, _1, i - 1));
		// 【订阅】无人机的目标点
		goals_sub[i] = nh.subscribe<geometry_msgs::PoseStamped>("/goal_" + std::to_string(i), 10, boost::bind(&ORCA::goal_cb, this, _1, i - 1));
	}

	// 打印定时器
	debug_timer = nh.createTimer(ros::Duration(10.0), &ORCA::debugCb, this);

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
	if (!start_flag)
	{
		return false;
	}

	// 判断是否达到目标点附近
	for (int i = 0; i < uav_num; ++i)
	{
		if (!arrived_goal[i])
		{
			arrived_goal[i] = reachedGoal(i);
		}
	}

	// 更新RVO中的位置和速度
	for (int i = 0; i < uav_num; ++i)
	{
		RVO::Vector2 pos = RVO::Vector2(uav_state[i].position[0], uav_state[i].position[1]);
		sim->setAgentPosition(i, pos); // 更新RVO仿真中的位置
	}
	// 计算每一个智能体的期望速度
	sim->computeVel();

	// 如果达到目标点附近，则使用直接控制直接移动到目标点
	int idx = uav_id - 1;
	if (arrived_goal[idx])
	{
		uav_control_cmd.header.stamp = ros::Time::now();
		uav_control_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
		uav_control_cmd.desired_pos[0] = sim->getAgentGoal(idx).x();
		uav_control_cmd.desired_pos[1] = sim->getAgentGoal(idx).y();
		uav_control_cmd.desired_pos[2] = agent_height;
		if(height_type == 1)
		{
			uav_control_cmd.desired_pos[2] = goals[idx].pose.position.z;
		}
		uav_control_cmd.desired_yaw = 0.0;
		control_cmd_pub.publish(uav_control_cmd);
		if (!goal_reached_printed[idx])
		{
			cout << " Arrived." << endl;
			goal_reached_printed[idx] = true;
		}
		return true;
	}
	// 如果没有达到目标点附近，则使用ORCA算法计算期望速度
	else
	{
		// 获得期望速度 注：这个速度是ENU坐标系的，并将其转换为机体系速度指令
		RVO::Vector2 vel = sim->getAgentVelCMD(idx);
		uav_control_cmd.header.stamp = ros::Time::now();
		uav_control_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
		uav_control_cmd.desired_vel[0] = vel.x();
		uav_control_cmd.desired_vel[1] = vel.y();
		uav_control_cmd.desired_pos[2] = agent_height;
		if(height_type == 1)
		{
			uav_control_cmd.desired_pos[2] = goals[idx].pose.position.z;
		}
		uav_control_cmd.desired_yaw = 0.0;
		control_cmd_pub.publish(uav_control_cmd);
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
	for (int i = 0; i < uav_num; i++)
	{
		RVO::Vector2 pos = RVO::Vector2(uav_state[i].position[0], uav_state[i].position[1]);
		sim->addAgent(pos);
		cout << BLUE << node_name << ": ORCA add agents_" << i + 1 << " at [" << uav_state[i].position[0] << "," << uav_state[i].position[1] << "]" << TAIL << endl;
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

void ORCA::debugCb(const ros::TimerEvent &e)
{
	// 固定的浮点显示
	cout.setf(ios::fixed);
	// setprecision(n) 设显示小数精度为n位
	cout << setprecision(2);
	// 左对齐
	cout.setf(ios::left);
	// 强制显示小数点
	cout.setf(ios::showpoint);
	// 强制显示符号
	cout.setf(ios::showpos);
}

bool ORCA::reachedGoal(int i)
{
	bool xy_arrived{false}, z_arrived{false}, yaw_arrived{false};
	RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
	float xy_distance = (uav_state[i].position[0] - rvo_goal.x()) * (uav_state[i].position[0] - rvo_goal.x()) + (uav_state[i].position[1] - rvo_goal.y()) * (uav_state[i].position[1] - rvo_goal.y());
	if (xy_distance < 0.15f * 0.15f)
	{
		xy_arrived = true;
	}

	if (abs(uav_state[i].position[2] - agent_height) < 0.08f)
	{
		z_arrived = true;
	}

	if (abs(uav_state[i].attitude[2] - 0.0) / M_PI * 180 < 3.0f)
	{
		yaw_arrived = true;
	}

	if (xy_arrived && z_arrived && yaw_arrived)
	{
		return true;
	}
	return false;
}

void ORCA::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i)
{
	uav_state[i] = *msg;
}

void ORCA::goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int i)
{
	start_flag = true;
	arrived_goal[i] = false;
	goals[i] = *msg;
	// 高度固定
	// goals[i].pose.position.z = agent_height;

	goal_reached_printed[i] = false;
	cout << goals[i].pose.position.x << "," << goals[i].pose.position.y << endl;
	sim->setAgentGoal(i, RVO::Vector2(goals[i].pose.position.x, goals[i].pose.position.y));
	cout << BLUE << node_name << ": Set agents_" << i + 1 << " goal at [" << goals[i].pose.position.x << "," << goals[i].pose.position.y << "]" << TAIL << endl;
}

void ORCA::printf_param()
{
	cout << GREEN << ">>>>>>>>>>>>>>>>>>> ORCA Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
	cout << GREEN << "uav_num    : " << uav_num << TAIL << endl;
	cout << GREEN << "agent_height : " << agent_height << TAIL << endl;

	// ORCA算法参数
	cout << GREEN << "neighborDist : " << orca_params.neighborDist << " [m]" << TAIL << endl;
	cout << GREEN << "maxNeighbors : " << orca_params.maxNeighbors << TAIL << endl;
	cout << GREEN << "timeHorizon : " << orca_params.timeHorizon << " [s]" << TAIL << endl;
	cout << GREEN << "timeHorizonObst : " << orca_params.timeHorizonObst << " [s]" << TAIL << endl;
	cout << GREEN << "radius : " << orca_params.radius << " [m]" << TAIL << endl;
	cout << GREEN << "maxSpeed : " << orca_params.maxSpeed << " [m/s]" << TAIL << endl;
	cout << GREEN << "time_step : " << orca_params.time_step << " [s]" << TAIL << endl;
}
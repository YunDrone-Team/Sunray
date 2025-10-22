#include "agent_sim.h"

// agent_sim仿真节点：
// 仿真逻辑：在仿真模式中，每一个无人车/无人机都需要单独启动一个本节点，来模拟真实无人机/车的运动
// 本节点通过订阅智能体指令话题以及底层控制指令话题，根据期望的指令仿真得到无人机的实时位置，并伪装发布为动捕更新的位置发回至智能体控制节点

void AGENT_SIM::init(ros::NodeHandle& nh)
{
    node_name = ros::this_node::getName();

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent.agent_id, 0);
    // 【参数】智能体初始位置
    nh.param<double>("init_pos_x", agent.fake_mocap.pose.position.x, 0.0);
    nh.param<double>("init_pos_y", agent.fake_mocap.pose.position.y, 0.0);
    nh.param<double>("init_pos_z", agent.fake_mocap.pose.position.z, 0.01);
    nh.param<double>("init_yaw", agent.agent_yaw, 0.0);
    // 【参数】智能体固定高度
    nh.param<float>("agent_height", agent_height, 0.1);

    string agent_name = "/uav_" + std::to_string(agent.agent_id);

    // 【订阅】智能体控制指令(Mavros消息) uav_control_node -> 本节点 
    agent_cmd_sub = nh.subscribe<mavros_msgs::PositionTarget>(agent_name +"/mavros/setpoint_raw/local", 1, &AGENT_SIM::agent_cmd_cb, this); 
       
    // 【发布】伪装成动捕的位置数据发布 本节点 -> external_fusion
    mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1);

    // 【定时器】定时计算fake_odom
    fake_odom_timer = nh.createTimer(ros::Duration(dt), &AGENT_SIM::update_agent_pos, this);

    // 初始化无人机初始位置
    agent.fake_mocap.header.stamp = ros::Time::now();
    agent.fake_mocap.header.frame_id = "world";
    agent.fake_mocap.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent.agent_yaw);
    mocap_pos_pub.publish(agent.fake_mocap);

    cout << GREEN << node_name << " ---------------> init! " << TAIL << endl;

    cout << GREEN << "agent.agent_id: " << agent.agent_id << TAIL << endl;
    cout << GREEN << "init_pos_x: " << agent.fake_mocap.pose.position.x << TAIL << endl;
    cout << GREEN << "init_pos_y: " << agent.fake_mocap.pose.position.y << TAIL << endl;
    cout << GREEN << "init_pos_z: " << agent.fake_mocap.pose.position.z << TAIL << endl;
    cout << GREEN << "init_yaw: " << agent.agent_yaw << TAIL << endl;
    cout << GREEN << "agent_height: " << agent_height << TAIL << endl;
}

// 输入：thrust_enu_sp，euler_sp, 上一时刻的位姿
// 输出：fake_odom
void AGENT_SIM::mainloop()
{
    if(!agent.get_fake_odom)
    {
        return;
    }

    // 发布agent.fake_mocap
    agent.fake_mocap.header.stamp = ros::Time::now();
    agent.fake_mocap.header.frame_id = "world";
    agent.fake_mocap.pose.position.x = agent.fake_pos[0];
    agent.fake_mocap.pose.position.y = agent.fake_pos[1];
    agent.fake_mocap.pose.position.z = agent.fake_pos[2];
    agent.fake_mocap.pose.orientation = agent.fake_quat;
    mocap_pos_pub.publish(agent.fake_mocap);
}

// 无人机指令回调函数，输入是Mavros控制话题，输出是记录话题中的控制模式和控制量
void AGENT_SIM::agent_cmd_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    agent.local_cmd = *msg;
    agent.get_local_cmd = true;

    if(msg->type_mask == 0x4000)
    {
        agent.pos_sp << 0.0 , 0.0 , 0.0;
        agent.vel_sp << 0.0 , 0.0 , 0.0;
        agent.euler_sp << 0.0 , 0.0 , 0.0;
        agent.throttle_sp = 0.0;
        agent.fake_thrust << 0.0 , 0.0 , agent.quad_mass * agent.gravity;
        agent.pos_control_type = POS_TYPE::IDLE;
        agent.get_local_cmd = false;
    }else if(msg->type_mask == 0b100111111000)
    {
        // send_pos_setpoint + yaw
        agent.pos_sp << msg->position.x , msg->position.y , msg->position.z;
        agent.vel_sp << 0.0 , 0.0 , 0.0;
        agent.euler_sp(2) = msg->yaw;
        agent.pos_control_type = POS_TYPE::XYZ_POS;
    }else if(msg->type_mask == 0b100111000111)
    {
        // send_vel_setpoint + yaw
        agent.pos_sp << 0.0 , 0.0 , 0.0;
        agent.vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        agent.euler_sp(2) = msg->yaw;
        agent.pos_control_type = POS_TYPE::XYZ_VEL;
    }else if(msg->type_mask == 0b100111000011)
    {
        // send_vel_xy_pos_z_setpoint + yaw
        agent.pos_sp << 0.0 , 0.0 , msg->position.z;
        agent.vel_sp << msg->velocity.x , msg->velocity.y , 0.0;
        agent.euler_sp(2) = msg->yaw;
        agent.pos_control_type = POS_TYPE::XY_VEL_Z_POS;
    }else if(msg->type_mask == 0b100111000000)
    {
        // send_pos_vel_xyz_setpoint + yaw
        agent.pos_sp << msg->position.x , msg->position.y , msg->position.z;
        agent.vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        agent.euler_sp(2) = msg->yaw;
        agent.pos_control_type = POS_TYPE::XYZ_POS_VEL;
    }else
    {
        agent.get_local_cmd = false;
        cout << RED << "wrong pos_cmd type_mask."<< TAIL << endl;
    }

    // 模拟位置控制，得到期望推力和期望姿态
    fake_pos_control();
}

// 输入：fake_pos，fake_vel
// 输出：thrust_enu_sp，fake_Rb
void AGENT_SIM::fake_pos_control()
{
    if(agent.pos_control_type == POS_TYPE::IDLE)
    {
        agent.euler_sp(0)  = 0.0;
        agent.euler_sp(1)  = 0.0;
        agent.thrust_enu_sp << 0.0 , 0.0, 0.0;
        return;
    }

    Eigen::Vector3d pos_error,vel_error;
    Eigen::Vector3d u_cmd;
    if(agent.pos_control_type ==  POS_TYPE::XYZ_POS)
    {
        pos_error = agent.pos_sp - agent.fake_pos;
        vel_error  = agent.k_pos * pos_error - agent.fake_vel;
        u_cmd = agent.k_vel * vel_error;
    }else if(agent.pos_control_type ==  POS_TYPE::XYZ_VEL)
    {
        vel_error  = agent.vel_sp - agent.fake_vel;
        u_cmd = agent.k_vel * vel_error;
    }else if(agent.pos_control_type ==  POS_TYPE::XY_VEL_Z_POS)
    {
        pos_error(2) = agent.pos_sp(2) - agent.fake_pos(2);
        vel_error(0)  = agent.vel_sp(0) - agent.fake_vel(0);
        vel_error(1)  = agent.vel_sp(1) - agent.fake_vel(1);
        vel_error(2)  = agent.k_pos * pos_error(2) - agent.fake_vel(2);
        u_cmd = agent.k_vel * vel_error;
    }else if(agent.pos_control_type ==  POS_TYPE::XYZ_POS_VEL)
    {
        // 目前以位置为准（针对Land）
        pos_error = agent.pos_sp - agent.fake_pos;
        vel_error  = agent.k_pos * pos_error - agent.fake_vel;
        u_cmd = agent.k_vel * vel_error;
    } 

	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    Eigen::Vector3d F_des;
    u_cmd(2) = u_cmd(2) + agent.gravity;
	F_des = u_cmd * agent.quad_mass;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * agent.quad_mass * agent.gravity)
	{
		F_des = F_des / F_des(2) * (0.5 * agent.quad_mass * agent.gravity);
	}
	else if (F_des(2) > 2 * agent.quad_mass * agent.gravity)
	{
		F_des = F_des / F_des(2) * (2 * agent.quad_mass * agent.gravity);
	}

    double tilt_angle_max = 3.14;
	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(tilt_angle_max)))
	{
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(tilt_angle_max)))
	{
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = geometry_utils::rotz(agent.fake_euler(2));
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    agent.euler_sp(0)  = std::atan2(-fy, fz);
    agent.euler_sp(1)  = std::atan2( fx, fz);
    // 缺偏航角动态（位置控制认为直接赋值，euler_sp(2)已经在前面赋值过了）
    //agent.euler_sp(2)  = agent.euler_sp(2);
    
    agent.thrust_enu_sp = F_des;
}

// 定时器，根据位置控制的结果定时计算无人机的实时位置
// 输入：thrust_enu_sp，euler_sp, 上一时刻的位姿
// 输出：fake_pos、fake_Rb
void AGENT_SIM::update_agent_pos(const ros::TimerEvent &e)
{
    if(!agent.get_local_cmd)
    {
        return;
    }

    // 计算期望推力
    agent.fake_thrust = agent.thrust_enu_sp;
    agent.fake_thrust(2) = agent.fake_thrust(2) - agent.quad_mass*agent.gravity;
    // 计算加速度,速度,位置
    agent.fake_acc = agent.fake_thrust /agent.quad_mass;
    agent.fake_vel = agent.fake_vel + agent.fake_acc * dt;
    agent.fake_pos = agent.fake_pos + agent.fake_vel * dt;
    // 计算姿态角(无角度动态,可以考虑加入一个一阶滤波)
    agent.fake_euler = agent.euler_sp;
    agent.fake_quat2 = quaternion_from_rpy(agent.fake_euler);
    agent.fake_quat.x = agent.fake_quat2.x();
    agent.fake_quat.y = agent.fake_quat2.y();
    agent.fake_quat.z = agent.fake_quat2.z();
    agent.fake_quat.w = agent.fake_quat2.w();
    agent.fake_Rb = agent.fake_quat2.toRotationMatrix();

    agent.get_fake_odom = true;
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
geometry_msgs::Quaternion AGENT_SIM::ros_quaternion_from_rpy(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    geometry_msgs::Quaternion ros_q;

    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    ros_q.w = q.w();

    return ros_q;    
}




#include <string>
#include <iostream>
#include <thread>
#include "ground_control.h"

void GroundControl::init(ros::NodeHandle &nh)
{

    nh.param<int>("uav_num", uav_num, 1);

    nh.param<string>("uav_name", uav_name, "uav");
    nh.param<string>("tcp_ip", tcp_ip, "0.0.0.0");
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<string>("tcp_port", tcp_port, "8969");
    nh.param<string>("udp_port", udp_port, "8968");
    State_Message state_msg;
    state_msg.head = 0;
    state_msg.length = 0;
    state_msg.msg_id = 0;
    state_msg.robot_id = 0;
    state_msg.payload.time_stamp = 0;
    state_msg.payload.uav_id = 0;
    state_msg.payload.connected = false;
    state_msg.payload.armed = false;
    state_msg.payload.mode = "";
    state_msg.payload.location_source = 0;
    state_msg.payload.odom_valid = false;
    state_msg.payload.position = {0, 0, 0};
    state_msg.payload.velocity = {0, 0, 0};
    state_msg.payload.attitude = {0, 0, 0};
    state_msg.payload.pos_setpoint = {0, 0, 0};
    state_msg.payload.vel_setpoint = {0, 0, 0};
    state_msg.payload.att_setpoint = {0, 0, 0};
    state_msg.payload.attitude_rate = {0, 0, 0};
    state_msg.payload.battery_state = 0;
    state_msg.payload.battery_percentage = 0;
    state_msg.payload.control_mode = 0;
    state_msg.check = 0;

    for (int i = 1; i < uav_num + 1; i++)
    {
        std::string topic_prefix = "/" + uav_name + std::to_string(i);
        control_cmd_pub.push_back(nh.advertise<sunray_msgs::UAVControlCMD>(
            topic_prefix + "/sunray/uav_control_cmd", 1));
        
        uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(
            topic_prefix + "/sunray/uav_state_cmd", 1, boost::bind(&GroundControl::uav_state_cb, this, _1, i)));

        uav_setup_pub.push_back(nh.advertise<sunray_msgs::UAVSetup>(
            topic_prefix + "/sunray/setup", 1));

        stateMessage.push_back(state_msg);
    }

    recvMsgTimer = nh.createTimer(ros::Duration(0.02), &GroundControl::recvMsgCb, this);
    sendMsgTimer = nh.createTimer(ros::Duration(0.1), &GroundControl::sendMsgCb, this);

    tcp_server.m_strIp = tcp_ip;
    tcp_server.m_uPort = stoul(tcp_port);
    while(!tcp_server.InitServer() && ros::ok())
    {
        std::cout << "TCP网络服务端初始化失败！\n";
        std::cout << "正在重试...";
        ros::Duration(5).sleep();
    }
    tcp_server.startListening();
    tcp_server.startWaitForClient();
    std::cout << "TCP网络服务端初始化成功！\n";
    // if (tcp_server.InitServer())
    // {
    //     std::cout << "TCP网络服务端初始化成功！\n";
    //     tcp_server.startListening();
    //     tcp_server.startWaitForClient();
    // }
    // else
    // {
    //     std::cout << "TCP网络服务端初始化失败！\n";
    // }
    udp_server.m_strIp = udp_ip;
    udp_server.m_uPort = stoul(udp_port);
    while (!udp_server.InitUDPClient() && ros::ok())
    {
        std::cout << "UDP网络服务端初始化失败！\n";
        std::cout << "正在重试...";
        ros::Duration(5).sleep();
    }
    std::cout << "UDP网络服务端初始化成功！\n";
}

void GroundControl::parseTcpMessage(char *message)
{
    // char *msg = message;
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;

    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pitch;
    uint8_t frame;
    float yaw_rate;

    extract_message_header(message, &head, &length, &msg_id, &robot_id, &check);
    std::cout << "head: " << head << " length: " << length << " msg_id: " << static_cast<int>(msg_id) << " robot_id: " << static_cast<int>(robot_id) << " check: " << check << std::endl;
    if (msg_id == 1)
    {
        Heartbeat_Message heartbeat_msg;
        unpack_Heartbeat_Message(message, &heartbeat_msg);
        last_time_stamp = heartbeat_msg.time_stamp;
    }

    else if (msg_id == 101)
    {
    }

    else if (msg_id == CONTROL_MESSAGE)
    {
        Control_Message control_msg;
        unpack_Control_Message(message, &control_msg);
        std::cout << "check:" << calculate_checksum_Control_Message(&control_msg) << std::endl;
        if (control_msg.check == calculate_checksum_Control_Message(&control_msg))
        {
            uint32_t time_stamp = control_msg.payload.time_stamp;
            x = control_msg.payload.x;
            y = control_msg.payload.y;
            z = control_msg.payload.z;
            yaw = control_msg.payload.yaw;
            roll = control_msg.payload.roll;
            pitch = control_msg.payload.pitch;
            frame = control_msg.payload.frame;
            yaw_rate = control_msg.payload.yaw_rate;
            std::cout << "time_stamp: " << time_stamp << " x: " << x << " y: " << y << " z: " << z << " yaw: " << yaw << " roll: " << roll << " pitch: " << pitch << " frame: " << static_cast<int>(frame) << " yaw_rate: " << yaw_rate << std::endl;
            if (control_msg.payload.type == CONTROL_TYPE_XYZ_POS)
            {
                std::cout << "xyz_pos" << std::endl;
                if (frame == 0)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS;
                }
                else if (frame == 1)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS_BODY;
                }
                else
                {
                    return;
                }
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.desired_pos[0] = x;
                uav_cmd.desired_pos[1] = y;
                uav_cmd.desired_pos[2] = z;
                uav_cmd.desired_yaw = yaw;
                uav_cmd.enable_yawRate = yaw_rate;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (control_msg.payload.type == CONTROL_TYPE_XYZ_VEL)
            {
                std::cout << "xyz_vel" << std::endl;
                if (frame == 0)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL;
                }
                else if (frame == 1)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL_BODY;
                }
                else
                {
                    return;
                }
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.desired_vel[0] = x;
                uav_cmd.desired_vel[1] = y;
                uav_cmd.desired_vel[2] = z;
                uav_cmd.desired_yaw = yaw;
                uav_cmd.enable_yawRate = yaw_rate;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }

            else if (control_msg.payload.type == CONTROL_TYPE_XY_VEL_Z_POS)
            {
                std::cout << "xy_vel_z_pos" << std::endl;
                if (frame == 0)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
                }
                else if (frame == 1)
                {
                    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY;
                }
                else
                {
                    return;
                }
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY;
                uav_cmd.desired_vel[0] = x;
                uav_cmd.desired_vel[1] = y;
                uav_cmd.desired_pos[2] = z;
                uav_cmd.desired_yaw = yaw;
                uav_cmd.enable_yawRate = yaw_rate;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (control_msg.payload.type == CONTROL_TYPE_XYZ_ATT)
            {
            }
            else if (control_msg.payload.type == CONTROL_TYPE_TRAJECTORY)
            {
            }
        }
    }
    else if (msg_id == MODE_MESSAGE)
    {
        Mode_Message mode_msg;

        unpack_Mode_Message(message, &mode_msg);
        std::cout << "check:" << calculate_checksum_Mode_Message(&mode_msg) << std::endl;
        if (mode_msg.check == calculate_checksum_Mode_Message(&mode_msg))
        {
            uint32_t time_stamp = mode_msg.payload.time_stamp;
            if (mode_msg.payload.uav_mode == MODE_TYPE_OFFBOARD)
            {
                setup.cmd = 4;
                setup.control_state = "CMD_CONTROL";
                uav_setup_pub.at(robot_id - 1).publish(setup);
            }
        }
    }
    else if (msg_id == VEHICLE_MESSAGE)
    {
        Vehicle_Message vehicle_msg;
        unpack_Vehicle_Message(message, &vehicle_msg);
        if (vehicle_msg.check == calculate_checksum_Vehicle_Message(&vehicle_msg))
        {
            uint32_t time_stamp = vehicle_msg.payload.time_stamp;
            if (vehicle_msg.payload.type == VEHICLE_DISARM)
            {
                std::cout << "disarm" << std::endl;
                setup.cmd = 0;
                uav_setup_pub[robot_id - 1].publish(setup);
            }
            else if (vehicle_msg.payload.type == VEHICLE_ARM)
            {
                std::cout << "arm" << std::endl;
                setup.cmd = 1;
                uav_setup_pub[robot_id - 1].publish(setup);
            }
            else if (vehicle_msg.payload.type == VEHICLE_TAKEOFF)
            {
                std::cout << "takeoff" << std::endl;
                uav_cmd.cmd = 1;
                uav_cmd.cmd_id = 0;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == VEHICLE_LAND)
            {
                std::cout << "land" << std::endl;
                uav_cmd.cmd = 3;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == VEHICLE_HOVER)
            {
                std::cout << "hover" << std::endl;
                uav_cmd.cmd = 2;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == VEHICLE_KILL)
            {
                std::cout << "kill" << std::endl;
                setup.cmd = 5;
                uav_setup_pub[robot_id - 1].publish(setup);
            }
        }
    }
}

void GroundControl::recvMsgCb(const ros::TimerEvent &e)
{
    if (tcp_server.HasMsg())
    {
        std::cout << "has msg" << std::endl;
        char *msg = tcp_server.GetMsg();
        if (msg != nullptr)
        {
            parseTcpMessage(msg);
            delete[] msg;
        }
    }
}

void GroundControl::sendMsgCb(const ros::TimerEvent &e)
{
    // std::cout << "send msg" << std::endl;
    for (int i = 0; i < uav_num; i++)
    {
        // std::cout << "send msg to " << i << std::endl;
        // if ((ros::Time::now() - ros::Time::fromSec(static_cast<double>(stateMessage.at(i).payload.time_stamp))).toSec() > 3.0)
        // {
        //     stateMessage.at(i).payload.connected = false;
        // }
        // char *msg = encode_State_Message(&stateMessage.at(i));
        char *msg = pack_State_Message(stateMessage.at(i).head, stateMessage.at(i).length, stateMessage.at(i).msg_id, stateMessage.at(i).robot_id,stateMessage.at(i).check, stateMessage.at(i).payload);
        udp_server.SendUDPMsg(msg);
        
    }
}

void GroundControl::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id)
{
    int index = robot_id - 1;
    stateMessage.at(index).head = 0xAD21;
    stateMessage.at(index).length = 72;
    stateMessage.at(index).msg_id = STATE_MESSAGE;
    stateMessage.at(index).robot_id = robot_id;
    stateMessage.at(index).payload.time_stamp = static_cast<uint32_t>(ros::Time::now().toSec());
    stateMessage.at(index).payload.uav_id = robot_id;
    stateMessage.at(index).payload.connected = msg->connected;
    stateMessage.at(index).payload.armed = msg->armed;
    std::string mode = msg->mode;
    if (mode.length() > 15)
    {
        mode = mode.substr(0, 15);
    }
    else if (mode.length() < 15)
    {
        mode.append(15 - mode.length(), ' ');
    }
    stateMessage.at(index).payload.mode = mode;
    stateMessage.at(index).payload.location_source = msg->location_source;
    stateMessage.at(index).payload.odom_valid = msg->odom_valid;
    stateMessage.at(index).payload.position = {msg->position[0], msg->position[1], msg->position[2]};
    stateMessage.at(index).payload.velocity = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    stateMessage.at(index).payload.attitude = {msg->attitude[0], msg->attitude[1], msg->attitude[2]};
    stateMessage.at(index).payload.pos_setpoint = {msg->pos_setpoint[0], msg->pos_setpoint[1], msg->pos_setpoint[2]};
    stateMessage.at(index).payload.vel_setpoint = {msg->vel_setpoint[0], msg->vel_setpoint[1], msg->vel_setpoint[2]};
    stateMessage.at(index).payload.att_setpoint = {msg->att_setpoint[0], msg->att_setpoint[1], msg->att_setpoint[2]};
    stateMessage.at(index).payload.battery_state = msg->battery_state;
    stateMessage.at(index).payload.battery_percentage = msg->battery_percetage;
    stateMessage.at(index).payload.control_mode = msg->control_mode;
    stateMessage.at(index).check = calculate_checksum_State_Message(&stateMessage.at(index));
}
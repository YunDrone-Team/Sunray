#include <string>
#include <iostream>
#include <thread>
#include "ground_control.h"

void GroundControl::init(ros::NodeHandle &nh)
{

    nh.param<int>("uav_num", uav_num, 1);

    nh.param<string>("uav_name", uav_name, "uav");

    for (int i = 1; i < uav_num + 1; i++)
    {
        std::string topic_prefix = "/" + uav_name + std::to_string(i);
        control_cmd_pub.push_back(nh.advertise<sunray_msgs::UAVControlCMD>(
            topic_prefix + "/sunray/uav_control_cmd", 1));

        uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(
            topic_prefix + "/sunray/uav_state", 1, boost::bind(&GroundControl::uav_state_cb, this, _1, i)));

        uav_setup_pub.push_back(nh.advertise<sunray_msgs::UAVSetup>(
            topic_prefix + "/sunray/setup", 1));

        // stateMessage.push_back(sunray_msgs::UAVState());
    }

    recvMsgTimer = nh.createTimer(ros::Duration(0.02), &GroundControl::recvMsgCb, this);
    // sendMsgTimer = nh.createTimer(ros::Duration(0.1), &GroundControl::sendMsgCb, this);

    server.m_uPort = 8969;
    if (server.InitServer())
    {
        std::cout << "TCP网络服务端初始化成功！\n";
        server.startListening();
        server.startWaitForClient();
    }
    else
    {
        std::cout << "TCP网络服务端初始化失败！\n";
    }
    udp_server.m_strIp = "192.168.25.22";
    udp_server.m_uPort = 8968;

    udp_server.InitUDPClient();
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
                setup.cmd = 3;
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
            if (vehicle_msg.payload.type == DISARM)
            {
                std::cout << "disarm" << std::endl;
                setup.cmd = 0;
                setup.arming = false;
                uav_setup_pub[robot_id - 1].publish(setup);
            }
            else if (vehicle_msg.payload.type == ARM)
            {
                std::cout << "arm" << std::endl;
                setup.cmd = 0;
                setup.arming = true;
                uav_setup_pub[robot_id - 1].publish(setup);
            }
            else if (vehicle_msg.payload.type == TAKEOFF)
            {
                std::cout << "takeoff" << std::endl;
                uav_cmd.cmd = 1;
                uav_cmd.cmd_id = 0;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == LAND)
            {
                std::cout << "land" << std::endl;
                uav_cmd.cmd = 3;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == HOVER)
            {
                std::cout << "hover" << std::endl;
                uav_cmd.cmd = 2;
                control_cmd_pub[robot_id - 1].publish(uav_cmd);
            }
            else if (vehicle_msg.payload.type == KILL)
            {
            }
        }
    }
}

void GroundControl::recvMsgCb(const ros::TimerEvent &e)
{
    if (server.HasMsg())
    {
        std::cout << "has msg" << std::endl;
        char *msg = server.GetMsg();
        if (msg != nullptr)
        {
            parseTcpMessage(msg);
            delete[] msg;
        }
    }
}

// void GroundControl::sendMsgCb(const ros::TimerEvent &e)
// {
//     for (int i = 0; i < uav_num; i++)
//     {
//         if ((ros::Time::now() - ros::Time::fromSec(static_cast<double>(stateMessage.at(i).payload.time_stamp))).toSec() > 3.0)
//         {
//             stateMessage.at(i).payload.connected = false;
//         }
//         char *msg =encode_State_Message(stateMessage.[i]);
//         udp_server.SendUDPMsg(msg);
//     }
// }

void GroundControl::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id)
{
    stateMessage.at(robot_id).head = 0xAD21;
    stateMessage.at(robot_id).length = 72;
    stateMessage.at(robot_id).msg_id = STATE_MESSAGE;
    stateMessage.at(robot_id).robot_id = robot_id;
    stateMessage.at(robot_id).payload.time_stamp = static_cast<uint32_t>(ros::Time::now().toSec());
    stateMessage.at(robot_id).payload.uav_id = robot_id;
    stateMessage.at(robot_id).payload.connected = msg->connected;
    stateMessage.at(robot_id).payload.armed = msg->armed;
    stateMessage.at(robot_id).payload.mode = msg->mode;
    stateMessage.at(robot_id).payload.location_source = msg->location_source;
    stateMessage.at(robot_id).payload.odom_valid = msg->odom_valid;
    stateMessage.at(robot_id).payload.position = {msg->position[0], msg->position[1], msg->position[2]};
    stateMessage.at(robot_id).payload.velocity = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    stateMessage.at(robot_id).payload.attitude = {msg->attitude[0], msg->attitude[1], msg->attitude[2], msg->attitude[3]};
    stateMessage.at(robot_id).payload.battery_state = msg->battery_state;
    stateMessage.at(robot_id).payload.battery_percentage = msg->battery_percetage;
    stateMessage.at(robot_id).payload.control_mode = msg->control_mode;
}
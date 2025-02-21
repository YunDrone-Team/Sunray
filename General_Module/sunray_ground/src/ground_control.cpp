#include <string>
#include <iostream>
#include <thread>
#include "ground_control.h"

void GroundControl::init(ros::NodeHandle &nh)
{

    nh.param<int>("uav_num", uav_num, 3);
    nh.param<int>("simulation_num", simulation_num, 1);
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "uav");
    nh.param<string>("tcp_ip", tcp_ip, "0.0.0.0");
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<string>("tcp_port", tcp_port, "8969");
    nh.param<int>("udp_port", udp_port, 9696);
    nh.param<int>("udp_ground_port", udp_ground_port, 9999);

    for (int i = uav_id; i < uav_id + simulation_num; i++)
    {
        std::string topic_prefix = "/" + uav_name + std::to_string(i);
        control_cmd_pub.push_back(nh.advertise<sunray_msgs::UAVControlCMD>(
            topic_prefix + "/sunray/uav_control_cmd", 1));

        uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(
            topic_prefix + "/sunray/uav_state", 1, boost::bind(&GroundControl::uav_state_cb, this, _1, i)));

        uav_setup_pub.push_back(nh.advertise<sunray_msgs::UAVSetup>(
            topic_prefix + "/sunray/setup", 1));

        uav_waypoint_pub.push_back(nh.advertise<sunray_msgs::UAVWayPoint>(
            topic_prefix + "/sunray/uav_waypoint", 1));

        udpData[i - 1].state.init();
    }

    for (int i = 1; i <= uav_num; i++)
    {
        if (uav_id == i)
            continue;

        std::string topic_prefix = "/" + uav_name + std::to_string(i);
        uav_state_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UAVState>(
                                                   topic_prefix + "/sunray/uav_state", 1))));

        // uav_state_pub.push_back(nh.advertise<sunray_msgs::UAVState>(
        //     topic_prefix + "/sunray/uav_state", 1));
    }

    // executiveDemo();

    sendMsgTimer = nh.createTimer(ros::Duration(0.1), &GroundControl::sendMsgCb, this);
    HeartbeatTimer = nh.createTimer(ros::Duration(0.5), &GroundControl::HeartRate, this);

    int back = tcpServer.Bind(static_cast<unsigned short>(std::stoi(tcp_port)));
    // qDebug()<<"TCPServer绑定端口号结果： "<<back;
    HeartbeatState = false; // 心跳包 TCPLinkState

    tcpServer.Listen(30);
    tcpServer.sigTCPServerReadData.connect(boost::bind(&GroundControl::TCPServerCallBack, this, _1));
    tcpServer.sigLinkState.connect(boost::bind(&GroundControl::TCPLinkState, this, _1, _2));
    tcpServer.setRunState(true);

    udpSocket = CommunicationUDPSocket::getInstance();
    udpSocket->Bind(static_cast<unsigned short>(udp_port));
    udpSocket->sigUDPUnicastReadData.connect(boost::bind(&GroundControl::UDPCallBack, this, _1));
    // udpSocket->InitSocket();
    udpSocket->setRunState(true);
}

void GroundControl::TCPLinkState(bool state, std::string IP)
{
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);
    if (state)
        GSIPHash.insert(IP);
    else
        GSIPHash.erase(IP);
}

uint8_t GroundControl::getPX4ModeEnum(std::string modeStr)
{
    uint8_t back;
    if (modeStr == "MANUAL")
        back = PX4ModeType::ManualType;
    else if (modeStr == "STABILIZED")
        back = PX4ModeType::StabilizedType;
    else if (modeStr == "ACRO")
        back = PX4ModeType::AcroType;
    else if (modeStr == "RATTITUDE")
        back = PX4ModeType::RattitudeType;
    else if (modeStr == "ALTITUDE")
        back = PX4ModeType::AltitudeType;
    else if (modeStr == "OFFBOARD")
        back = PX4ModeType::OffboardType;
    else if (modeStr == "POSITION")
        back = PX4ModeType::PositionType;
    else if (modeStr == "HOLD")
        back = PX4ModeType::HoldType;
    else if (modeStr == "MISSION")
        back = PX4ModeType::MissionType;
    else if (modeStr == "RETURN")
        back = PX4ModeType::ReturnType;
    else if (modeStr == "FOLLOW ME")
        back = PX4ModeType::FollowMeType;
    else if (modeStr == "PRECISION LAND")
        back = PX4ModeType::PrecisionLandType;
    else
        back = 0;
    return back;
}

void GroundControl::UDPCallBack(ReceivedParameter readData)
{
    int back;
    // std::cout << " GroundControl::UDPCallBack: " << (int)readData.messageID << std::endl;

    // std::cout << "readData.communicationType: " << (int)readData.communicationType << "  " << readData.messageID << std::endl;
    switch (readData.messageID)
    {
    case MessageID::SearchMessageID:
    { /* code */
        // std::cout << "MessageID::SearchMessageID: " << (int)readData.communicationType << " readData.ip: " << readData.ip << " readData.port: " << readData.port << std::endl;
        unionData backData;
        for (int i = uav_id; i < uav_id + simulation_num; i++)
        {
            backData.ack.init();
            backData.ack.robotID = i;
            backData.ack.uavID = i;
            backData.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            back = udpSocket->sendUDPData(codec.coder(MessageID::ACKMessageID, backData), readData.ip, (uint16_t)readData.data.search.port);
            std::lock_guard<std::mutex> lock(_mutexUDP);
            // udp_ground_port = readData.data.search.port;
            // udp_ip = readData.ip;
            // std::cout << "发送结果: " << back << " readData.data.search.port " << readData.data.search.port << std::endl;
        }

        break;
    }
    case MessageID::StateMessageID:
        // std::cout << " case MessageID::StateMessageID: " << (int)readData.data.state.uavID << std::endl;
        if (uav_id == readData.data.state.uavID)
            return;
        // std::cout << "uav_id: " << uav_id << " readData.data.state.uavID " << (int)readData.data.state.uavID<<" readData.messageID "<<readData.messageID << std::endl;
        SynchronizationUAVState(readData.data.state);
        break;
    default:
        break;
    }
}

pid_t GroundControl::CheckChildProcess(pid_t pid)
{

    // WIFEXITED(status)：若子进程正常退出，该宏会返回 true。
    // WEXITSTATUS(status)：当 WIFEXITED(status) 为 true 时，此宏用于获取子进程的退出状态码。
    // WIFSIGNALED(status)：若子进程是因为接收到信号而终止，该宏会返回 true。
    // WTERMSIG(status)：当 WIFSIGNALED(status) 为 true 时，此宏用于获取终止子进程的信号编号。
    // WIFSTOPPED(status)：若子进程被暂停，该宏会返回 true。
    // WSTOPSIG(status)：当 WIFSTOPPED(status) 为 true 时，此宏用于获取使子进程暂停的信号编号。

    // 检测子进程状态，
    int status;
    pid_t terminated_pid = waitpid(pid, &status, WNOHANG);
    if (terminated_pid == pid)
    {
        if (WIFEXITED(status))
        {
            std::cout << "Child process (PID: " << pid << ") exited normally with status " << WEXITSTATUS(status) << std::endl;
        }
        else if (WIFSIGNALED(status))
        {
            std::cout << "Child process (PID: " << pid << ") was terminated by signal " << WTERMSIG(status) << std::endl;
        }
    }
    else if (terminated_pid == 0)
    {
        // 子进程还在运行
        // std::cout << "Child process (PID: " << pid << ") is still running." << std::endl;
    }
    else
    {
        perror("waitpid");
    }
    return terminated_pid;
}

pid_t GroundControl::OrderCourse(std::string orderStr)
{
    pid_t pid = fork();
    if (pid == -1)
    {
        perror("fork failed");
        // return EXIT_FAILURE;
    }
    else if (pid == 0)
    {

        std::string temp = "bash -c \"cd /home/yundrone/Sunray && . devel/setup.sh && ";
        temp += orderStr;
        std::cout << "OrderCourse： " << temp << std::endl;

        // const char *command = "bash -c \"cd /home/yundrone/Sunray && . devel/setup.sh && roslaunch sunray_tutorial run_demo.launch\"";
        const char *command = temp.c_str();
        execlp("bash", "bash", "-c", command, (char *)NULL);
        // 如果execlp返回，说明执行失败
        perror("OrderCourse Error!");
        _exit(EXIT_FAILURE);
    }
    else
    {

        demoPID = pid;
        printf("This is the parent process. Child PID: %d\n", pid);
    }
    return pid;
}

void GroundControl::executiveDemo(std::string orderStr)
{
    pid_t pid = fork();
    if (pid == -1)
    {
        perror("fork failed");
        // return EXIT_FAILURE;
        return;
    }
    else if (pid == 0)
    {
        // 子进程
        // 注意：这里的命令字符串需要仔细构造，以确保它能在bash中正确执行
        // 由于source是bash的内建命令，不能直接在execlp中调用它
        // 但是，可以使用. (点命令) 来代替source
        // 另外，cd命令也需要在同一个shell中执行，所以我们不能简单地用&&连接命令
        // 而是需要将它们放在一个bash -c参数中

        std::string temp = "bash -c \"cd /home/yundrone/Sunray && . devel/setup.sh && ";
        temp += orderStr;
        std::cout << "executiveDemo： " << temp << std::endl;

        // const char *command = "bash -c \"cd /home/yundrone/Sunray && . devel/setup.sh && roslaunch sunray_tutorial run_demo.launch\"";
        const char *command = temp.c_str();
        execlp("bash", "bash", "-c", command, (char *)NULL);
        // 如果execlp返回，说明执行失败
        perror("execlp failed");
        _exit(EXIT_FAILURE);
    }
    else
    {
        // 父进程
        int status;

        demoPID = pid;
        printf("This is the parent process. Child PID: %d\n", pid);
    }
}

void GroundControl::TCPServerCallBack(ReceivedParameter readData)
{
    // 需要上锁，这里是TCP服务端的线程，这些数据基本只有这个函数调用，所以目前不上锁
    //  std::cout << "TCP网络服务端收到数据：" << readData.data << std::endl;
    //  std::cout << "TCP网络服务端收到数据长度：" << readData.data.size() << std::endl;
    //  std::cout << "TCP网络服务端收到数据长度：" << readData.data.length() << std::endl;
    //  std::cout << "TCP网络服务端收到数据长度：" << readData.data.size() << std::endl;

    float roll;
    float pitch;
    uint32_t time_stamp;
    uint8_t robot_id;

    std::cout << "GroundControl::TCPServerCallBack:" << readData.messageID << std::endl;
    switch (readData.messageID)
    {
    case MessageID::HeartbeatMessageID:
        // std::cout << "TCPServer心跳包接收到机器人id： " << (int)readData.data.heartbeat.robotID << std::endl;
        // std::cout << "TCPServer心跳包接收到时间戳 " << readData.data.heartbeat.timestamp << std::endl;
        // std::cout << "TCPServer心跳包接收到IP " << readData.ip << std::endl;

        HeartbeatState = true; // 心跳包
        GSIPHash.insert(readData.ip);

        break;
    case MessageID::ControlMessageID:
        // std::cout << "TCPServer参数控制接收到机器人id： " << (int)readData.data.contro.robotID << std::endl;
        // std::cout << "TCPServer接收到参数控制数据" << std::endl;
        // std::cout << "TCPServer参数控制接收到时间戳 " << readData.data.contro.timestamp << std::endl;
        // std::cout << "TCPServer控制模式 " << (int)readData.data.contro.controlMode << std::endl;
        // std::cout << "x " << readData.data.contro.position.x << std::endl;
        // std::cout << "y " << readData.data.contro.position.y << std::endl;
        // std::cout << "z " << readData.data.contro.position.z << std::endl;
        // std::cout << "yaw " << readData.data.contro.yaw << std::endl;
        // std::cout << "roll " << readData.data.contro.roll << std::endl;
        // std::cout << "pitch " << readData.data.contro.pitch << std::endl;
        // std::cout << "yawRate " << readData.data.contro.yawRate << std::endl;

        time_stamp = readData.data.contro.timestamp;

        roll = readData.data.contro.roll;
        pitch = readData.data.contro.pitch;
        robot_id = readData.data.contro.robotID;

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = readData.data.contro.controlMode;

        uav_cmd.desired_pos[0] = readData.data.contro.position.x;
        uav_cmd.desired_pos[1] = readData.data.contro.position.y;
        uav_cmd.desired_pos[2] = readData.data.contro.position.z;

        uav_cmd.desired_vel[0] = readData.data.contro.velocity.x;
        uav_cmd.desired_vel[1] = readData.data.contro.velocity.y;
        uav_cmd.desired_vel[2] = readData.data.contro.velocity.z;

        uav_cmd.desired_yaw = readData.data.contro.yaw;
        uav_cmd.desired_yaw_rate = readData.data.contro.yawRate;

        control_cmd_pub[robot_id - uav_id].publish(uav_cmd);

        break;
    case MessageID::VehicleMessageID:
        // std::cout << "TCPServer模式切换接收到机器人id： " << (int)readData.data.vehicle.robotID << std::endl;

        // std::cout << "TCPServer接收到模式切换数据" << std::endl;
        // std::cout << "TCPServer模式切换接收到时间戳 " << readData.data.vehicle.timestamp << std::endl;
        // std::cout << "TCPServersunray控制模式 " << (int)readData.data.vehicle.sunray_mode << std::endl;
        // std::cout << "TCPServerpx4控制模式 " << (int)readData.data.vehicle.px4_mode << std::endl;
        robot_id = readData.data.vehicle.robotID;
        setup.header.stamp = ros::Time::now();
        if (readData.data.vehicle.sunray_mode == VehicleControlType::SetControlMode)
        {
            setup.control_state = "CMD_CONTROL";
        }
        setup.cmd = readData.data.vehicle.sunray_mode;

        // std::cout << "uav_setup_pub " << uav_setup_pub.size() << " robot_id -uav_id " << (robot_id - uav_id) << " robot_id:" << (int)robot_id << " uav_id " << uav_id << std::endl;

        uav_setup_pub.at(robot_id - uav_id).publish(setup);
        break;
    case MessageID::DemoMessageID:
        // std::cout << "TCPServer Demo接收到机器人id： " << (int)readData.data.demo.robotID << std::endl;
        // std::cout << "TCPServer Demo： " << readData.data.demo.demoStr << std::endl;
        // std::cout << "TCPServer Demo size： " << readData.data.demo.demoSize << std::endl;

        if (readData.data.demo.demoState == true)
        {
            // if (demoPID <= 0 && readData.data.demo.demoSize > 0)
            //     executiveDemo(readData.data.demo.demoStr);

            if (readData.data.demo.demoSize > 0)
            {
                auto it = nodeMap.find(readData.data.demo.demoStr);
                if (it != nodeMap.end())
                {
                    std::cout << "该节点已启动： " << readData.data.demo.demoSize << std::endl;
                }
                else
                {
                    pid_t back = OrderCourse(readData.data.demo.demoStr);
                    if (back > 0)
                        nodeMap.insert(std::make_pair(readData.data.demo.demoStr, back));
                }
            }
        }
        else
        {
            if (readData.data.demo.demoSize > 0)
            {
                std::cout << "TCPServer 关闭Demo： " << std::endl;

                auto it = nodeMap.find(readData.data.demo.demoStr);
                if (it != nodeMap.end())
                {
                    pid_t temp = it->second;
                    if (kill(temp, SIGTERM) != 0)
                    {
                        perror("kill failed!");
                    }
                    else
                    {
                        printf("Sent SIGTERM to child process %d\n", temp);
                        nodeMap.erase(readData.data.demo.demoStr);
                    }
                }
                else
                {
                    std::cout << "该节点未启动： " << readData.data.demo.demoStr << std::endl;
                }
            }
        }

        break;
    case MessageID::WaypointMessageID:
    {
        robot_id = readData.data.waypointData.robotID;

        // std::cout << "MessageID::WaypointMessageID 1 "<< std::endl;
        sunray_msgs::UAVWayPoint waypoint_msg;
        waypoint_msg.header.stamp = ros::Time::now();
        waypoint_msg.wp_num = readData.data.waypointData.wpNum;
        waypoint_msg.wp_type = readData.data.waypointData.wpType;
        waypoint_msg.wp_end_type = readData.data.waypointData.wpEndType;
        waypoint_msg.wp_yaw_type = readData.data.waypointData.wpYawType;
        waypoint_msg.wp_takeoff = readData.data.waypointData.wpTakeoff;
        waypoint_msg.wp_move_vel = readData.data.waypointData.wpMoveVel;
        waypoint_msg.wp_vel_p = readData.data.waypointData.wpVelP;
        waypoint_msg.z_height = readData.data.waypointData.wpHeight;

        // std::cout << "MessageID::WaypointMessageID 2 "<<readData.data.waypointData.Waypoint1.X<< std::endl;

        waypoint_msg.wp_point_1 = {readData.data.waypointData.Waypoint1.X, readData.data.waypointData.Waypoint1.Y,
                                   readData.data.waypointData.Waypoint1.Z, readData.data.waypointData.Waypoint1.Yaw};
        waypoint_msg.wp_point_2 = {readData.data.waypointData.Waypoint2.X, readData.data.waypointData.Waypoint2.Y,
                                   readData.data.waypointData.Waypoint2.Z, readData.data.waypointData.Waypoint2.Yaw};
        waypoint_msg.wp_point_3 = {readData.data.waypointData.Waypoint3.X, readData.data.waypointData.Waypoint3.Y,
                                   readData.data.waypointData.Waypoint3.Z, readData.data.waypointData.Waypoint3.Yaw};
        waypoint_msg.wp_point_4 = {readData.data.waypointData.Waypoint4.X, readData.data.waypointData.Waypoint4.Y,
                                   readData.data.waypointData.Waypoint4.Z, readData.data.waypointData.Waypoint4.Yaw};
        waypoint_msg.wp_point_5 = {readData.data.waypointData.Waypoint5.X, readData.data.waypointData.Waypoint5.Y,
                                   readData.data.waypointData.Waypoint5.Z, readData.data.waypointData.Waypoint5.Yaw};
        waypoint_msg.wp_point_6 = {readData.data.waypointData.Waypoint6.X, readData.data.waypointData.Waypoint6.Y,
                                   readData.data.waypointData.Waypoint6.Z, readData.data.waypointData.Waypoint6.Yaw};
        waypoint_msg.wp_point_7 = {readData.data.waypointData.Waypoint7.X, readData.data.waypointData.Waypoint7.Y,
                                   readData.data.waypointData.Waypoint7.Z, readData.data.waypointData.Waypoint7.Yaw};
        waypoint_msg.wp_point_8 = {readData.data.waypointData.Waypoint8.X, readData.data.waypointData.Waypoint8.Y,
                                   readData.data.waypointData.Waypoint8.Z, readData.data.waypointData.Waypoint8.Yaw};
        waypoint_msg.wp_point_9 = {readData.data.waypointData.Waypoint9.X, readData.data.waypointData.Waypoint9.Y,
                                   readData.data.waypointData.Waypoint9.Z, readData.data.waypointData.Waypoint9.Yaw};
        waypoint_msg.wp_point_10 = {readData.data.waypointData.Waypoint10.X, readData.data.waypointData.Waypoint10.Y,
                                    readData.data.waypointData.Waypoint10.Z, readData.data.waypointData.Waypoint10.Yaw};
        waypoint_msg.wp_circle_point = {readData.data.waypointData.wpCirclePointX, readData.data.waypointData.wpCirclePointY};

        uav_waypoint_pub[robot_id - uav_id].publish(waypoint_msg);

        // std::cout << "MessageID::WaypointMessageID end "<< std::endl;

        // uav_waypoint_pub          control_cmd_pub[robot_id - uav_id].publish(uav_cmd);
        break;
    }
    default:
        break;
    }
    // std::cout << "GroundControl::TCPServerCallBack end" << std::endl;
}

bool GroundControl::SynchronizationUAVState(StateData Data)
{

    auto it = uav_state_pub.find(Data.uavID);

    if (it == uav_state_pub.end())
    {
        std::cout << "UAV" + std::to_string(Data.uavID) + " topic Publisher not found!" << std::endl;
        return false;
    }

    sunray_msgs::UAVState msg;
    msg.uav_id = Data.uavID;
    msg.connected = Data.connected;
    msg.armed = Data.armed;
    msg.mode = Data.mode;
    msg.location_source = Data.locationSource;
    msg.odom_valid = Data.odom_valid;
    msg.position[0] = Data.position.x;
    msg.position[1] = Data.position.y;
    msg.position[2] = Data.position.z;
    msg.velocity[0] = Data.velocity.x;
    msg.velocity[1] = Data.velocity.y;
    msg.velocity[2] = Data.velocity.z;
    msg.attitude[0] = Data.attitude.x;
    msg.attitude[1] = Data.attitude.y;
    msg.attitude[2] = Data.attitude.z;
    msg.attitude_q.w = Data.attitudeQuaternion.w;
    msg.attitude_q.x = Data.attitudeQuaternion.x;
    msg.attitude_q.y = Data.attitudeQuaternion.y;
    msg.attitude_q.z = Data.attitudeQuaternion.z;
    msg.attitude_rate[0] = Data.attitudeRate.x;
    msg.attitude_rate[1] = Data.attitudeRate.y;
    msg.attitude_rate[2] = Data.attitudeRate.z;
    msg.pos_setpoint[0] = Data.posSetpoint.x;
    msg.pos_setpoint[1] = Data.posSetpoint.y;
    msg.pos_setpoint[2] = Data.posSetpoint.z;
    msg.vel_setpoint[0] = Data.velSetpoint.x;
    msg.vel_setpoint[1] = Data.velSetpoint.y;
    msg.vel_setpoint[2] = Data.velSetpoint.z;
    msg.att_setpoint[0] = Data.attSetpoint.x;
    msg.att_setpoint[1] = Data.attSetpoint.y;
    msg.att_setpoint[2] = Data.attSetpoint.z;
    msg.battery_state = Data.batteryState;
    msg.battery_percetage = Data.batteryPercentage;
    msg.control_mode = Data.controlMode;
    msg.move_mode = Data.moveMode;

    it->second.publish(msg);
    return true;
}

void GroundControl::sendMsgCb(const ros::TimerEvent &e)
{
    //  std::cout << "send msg" << std::endl;
    for (int i = uav_id; i < uav_id + simulation_num; i++)
    {

        std::lock_guard<std::mutex> lock(_mutexUDP);
        // std::cout << "UDP目的ip： " << udp_ip << " UDP目的端口： "<<udp_port<< std::endl;
        //  std::cout << "udpData[i] " << (int)udpData[i].state.uavID << " i "<<i<< std::endl;

        //  int back = udpSocket->sendUDPData(codec.coder(MessageID::StateMessageID, udpData[i - 1]), udp_ip, udp_port);

        // 无人机机间组播链路发送
        int back = udpSocket->sendUDPMulticastData(codec.coder(MessageID::StateMessageID, udpData[i - 1]), udp_port);

        // 地面站组播链路发送
        udpSocket->sendUDPMulticastData(codec.coder(MessageID::StateMessageID, udpData[i - 1]), udp_ground_port);

        // UDP单播地面站发送
        //  udpSocket->sendUDPData(codec.coder(MessageID::StateMessageID, udpData[i - 1]), udp_ip, udp_ground_port);

        std::vector<std::string> tempVec;
        for (const auto &ip : GSIPHash)
        {
            int sendBack = udpSocket->sendUDPData(codec.coder(MessageID::StateMessageID, udpData[i - 1]), ip, udp_ground_port);
            if (sendBack < 0)
                tempVec.push_back(ip);
        }

        for (const auto &ip : tempVec)
            GSIPHash.erase(ip);
        tempVec.clear();

        // std::cout << "udp状态发送结果： " << back<<" port "<<udp_port << std::endl;
    }
}

void GroundControl::HeartRate(const ros::TimerEvent &e)
{
    // std::cout << "GroundControl::HeartRate: "<<HeartbeatState << std::endl;
    if (HeartbeatState)
    {

        unionData Heartbeatdata;

        for (int i = uav_id; i < simulation_num + uav_id; i++)
        {
            Heartbeatdata.state.robotID = i;
            tcpServer.allSendData(codec.coder(MessageID::HeartbeatMessageID, Heartbeatdata));
        }
        // Heartbeatdata.state.robotID = uav_id;
        // tcpServer.allSendData(codec.coder(MessageID::HeartbeatMessageID, Heartbeatdata));

        std::vector<std::string> keysToRemove;
        for (const auto &pair : nodeMap)
        {
            if (pair.second == CheckChildProcess(pair.second))
                keysToRemove.push_back(pair.first);
        }

        for (const auto &key : keysToRemove)
            nodeMap.erase(key);
    }
}

void GroundControl::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id)
{

    int index = robot_id - 1;
    udpData[index].state.init();
    udpData[index].state.robotID = robot_id;
    udpData[index].state.uavID = robot_id;
    udpData[index].state.connected = msg->connected;
    // std::cout << "uav_state_cb udpData[i] " << (int)udpData[index].state.uavID << " index "<<index<< std::endl;
    udpData[index].state.armed = msg->armed;
    udpData[index].state.mode = getPX4ModeEnum(msg->mode);
    udpData[index].state.locationSource = msg->location_source;
    udpData[index].state.odom_valid = msg->odom_valid;
    udpData[index].state.position.x = msg->position[0];
    udpData[index].state.position.y = msg->position[1];
    udpData[index].state.position.z = msg->position[2];

    // std::cout << "position z: "<<msg->position[2] << std::endl;
    udpData[index].state.velocity.x = msg->velocity[0];
    udpData[index].state.velocity.y = msg->velocity[1];
    udpData[index].state.velocity.z = msg->velocity[2];
    udpData[index].state.attitude.x = msg->attitude[0];
    udpData[index].state.attitude.y = msg->attitude[1];
    udpData[index].state.attitude.z = msg->attitude[2];

    // std::cout << "期望姿态 x: "<<msg->att_setpoint[0] << std::endl;
    // std::cout << "期望姿态 y: "<<msg->att_setpoint[1] << std::endl;
    // std::cout << "期望姿态 z: "<<msg->att_setpoint[2] << std::endl;

    // udpData[index].state.attitudeRate.x=msg->att_setpoint[0];
    // udpData[index].state.attitudeRate.y=msg->att_setpoint[1];
    // udpData[index].state.attitudeRate.z=msg->att_setpoint[2];

    udpData[index].state.posSetpoint.x = msg->pos_setpoint[0];
    udpData[index].state.posSetpoint.y = msg->pos_setpoint[1];
    udpData[index].state.posSetpoint.z = msg->pos_setpoint[2];

    // udpData[index].state.posSetpoint.x = std::nan("");
    // udpData[index].state.posSetpoint.y = std::nan("");
    // udpData[index].state.posSetpoint.z = std::nan("");

    udpData[index].state.velSetpoint.x = msg->vel_setpoint[0];
    udpData[index].state.velSetpoint.y = msg->vel_setpoint[1];
    udpData[index].state.velSetpoint.z = msg->vel_setpoint[2];

    udpData[index].state.attSetpoint.x = msg->att_setpoint[0];
    udpData[index].state.attSetpoint.y = msg->att_setpoint[1];
    udpData[index].state.attSetpoint.z = msg->att_setpoint[2];

    // udpData[index].state.attSetpoint.x = std::nan("");
    // udpData[index].state.attSetpoint.y = std::nan("");
    // udpData[index].state.attSetpoint.z = std::nan("");

    udpData[index].state.attitudeQuaternion.w = msg->attitude_q.w;
    udpData[index].state.attitudeQuaternion.x = msg->attitude_q.x;
    udpData[index].state.attitudeQuaternion.y = msg->attitude_q.y;
    udpData[index].state.attitudeQuaternion.z = msg->attitude_q.z;

    udpData[index].state.batteryState = msg->battery_state;
    udpData[index].state.batteryPercentage = msg->battery_percetage;
    udpData[index].state.controlMode = msg->control_mode;
    udpData[index].state.moveMode = msg->move_mode;

    std::string mode = msg->mode;
    if (mode.length() > 15)
    {
        mode = mode.substr(0, 15);
    }
    else if (mode.length() < 15)
    {
        mode.append(15 - mode.length(), ' ');
    }
}
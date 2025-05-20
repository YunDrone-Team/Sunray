#include <string>
#include <iostream>
#include <thread>
#include "communication_bridge.h"

void communication_bridge::init(ros::NodeHandle &nh)
{
    nh.param<bool>("is_simulation", is_simulation, false);      // 【参数】确认当前是仿真还是真机实验
    nh.param<int>("uav_experiment_num", uav_experiment_num, 0); // 【参数】无人机真机数量（用于智能体之间的通信）
    nh.param<int>("uav_simulation_num", uav_simulation_num, 0); // 【参数】仿真无人机数量（用于仿真时，一次性订阅多个无人机消息）
    nh.param<int>("uav_id", uav_id, 1);                         // 【参数】无人机编号(真机为本机，仿真为1)
    nh.param<string>("uav_name", uav_name, "uav");              // 【参数】无人机名字前缀

    nh.param<int>("ugv_experiment_num", ugv_experiment_num, 0); // 【参数】无人车真车数量
    nh.param<int>("ugv_simulation_num", ugv_simulation_num, 0); // 【参数】仿真无人车数量
    nh.param<int>("ugv_id", ugv_id, 1);                         // 【参数】无人车编号(真机为本机，仿真为1)
    nh.param<string>("ugv_name", ugv_name, "ugv");              // 【参数】无人车名字前缀

    nh.param<string>("tcp_port", tcp_port, "8969");          // 【参数】TCP绑定端口
    nh.param<int>("udp_port", udp_port, 9696);               // 【参数】UDP机载端口（绑定监听端口），要和组播目标端口（udp_ground_port）要一致
    nh.param<int>("udp_ground_port", udp_ground_port, 9999); // 【参数】组播目标端口，用于机间通信

    // 情况枚举：
    // CASE1（真机）:只有一台无人机的时候 uav_id =本机ID   uav_experiment_num=1 uav_simulation_num=0，不提及的默认都为0
    // CASE2（真机）:只有一台无人车的时候 ugv_id =本机ID   ugv_experiment_num=1 ugv_simulation_num=0，不提及的默认都为0
    // CASE3（真机）:3台无人机(uav_id=1\2\3,robot_id=1\2\3)、2台无人车(uav_id=1\2,robot_id=1\2)
    // uav_id =本机ID   uav_experiment_num=3 uav_simulation_num=0
    // ugv_id =本机ID   ugv_experiment_num=2 uav_simulation_num=0
    // CASE1（仿真）:只有一台无人机的时候 uav_id =1   uav_experiment_num=0 uav_simulation_num=1，不提及的默认都为0
    // CASE2（仿真）:3台无人机 uav_id =1   uav_experiment_num=0 uav_simulation_num=3，不提及的默认都为0
    // CASE3（仿真）:只有一台无人车的时候 ugv_id =1   ugv_experiment_num=0 ugv_simulation_num=1，不提及的默认都为0
    // CASE4（仿真）:3台无人车 ugv_id =1   ugv_experiment_num=0 ugv_simulation_num=3，不提及的默认都为0
    // CASE5（仿真）:3台无人机、2台无人车
    // uav_id =1   uav_experiment_num=0 uav_simulation_num=3
    // ugv_id =1   ugv_experiment_num=0 ugv_simulation_num=2

    if (is_simulation)
    {
        // 无人机Sunray与地面站之间的通信（此部分仅针对仿真）
        for (int i = uav_id; i < uav_id + uav_simulation_num; i++)
        {
            //  无人机名字 = 无人机名字前缀 + 无人机ID
            std::string topic_prefix = "/" + uav_name + std::to_string(i);
            // 【订阅】无人机状态 uav_control_node --ROS topic--> 本节点 --UDP--> 地面站
            uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, boost::bind(&communication_bridge::uav_state_cb, this, _1, i)));
            // 【发布】无人机控制指令 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            control_cmd_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1))));
            // 【发布】无人机设置指令 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            uav_setup_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1))));
            // 【发布】无人机航点数据 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            uav_waypoint_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 1))));
        }

        // 无人车Sunray与地面站之间的通信（此部分仅针对仿真）
        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            //  无人车名字 = 无人车名字前缀 + 无人车ID
            std::string topic_prefix = "/" + ugv_name + std::to_string(i);
            // 【订阅】无人车状态 ugv_control_node --ROS topic--> 本节点 --UDP--> 地面站
            ugv_state_sub.push_back(nh.subscribe<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 1, boost::bind(&communication_bridge::ugv_state_cb, this, _1, i)));
            // 【发布】无人车控制指令 地面站 --TCP--> 本节点 --ROS topic--> ugv_control_node
            ugv_controlCMD_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 1))));
        }
    }
    else
    {
        // 无人机Sunray和地面站之间的通信（此部分仅针对真机）
        if (uav_experiment_num > 0)
        {
            //  无人机名字 = 无人机名字前缀 + 无人机ID
            std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
            // 【订阅】无人机状态 uav_control_node --ROS topic--> 本节点 --UDP--> 地面站/其他Sunray智能体
            uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, boost::bind(&communication_bridge::uav_state_cb, this, _1, uav_id)));
            // 【发布】无人机控制指令 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            control_cmd_pub.insert(std::make_pair(uav_id, (nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1))));
            // 【发布】无人机设置指令 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            uav_setup_pub.insert(std::make_pair(uav_id, (nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1))));
            // 【发布】无人机航点数据 地面站 --TCP--> 本节点 --ROS topic--> uav_control_node
            uav_waypoint_pub.insert(std::make_pair(uav_id, (nh.advertise<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 1))));
        }

        // 无人车Sunray和地面站之间的通信（此部分仅针对真机）
        if (ugv_experiment_num > 0)
        {
            //  无人车名字 = 无人车名字前缀 + 无人车ID
            std::string topic_prefix = "/" + ugv_name + std::to_string(ugv_id);
            // 【订阅】无人车状态 ugv_control_node --ROS topic--> 本节点 --UDP--> 地面站/其他Sunray智能体
            ugv_state_sub.push_back(nh.subscribe<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 1, boost::bind(&communication_bridge::ugv_state_cb, this, _1, ugv_id)));
            // 【发布】无人车控制指令 地面站 --TCP--> 本节点 --ROS topic--> ugv_control_node
            ugv_controlCMD_pub.insert(std::make_pair(ugv_id, (nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 1))));
        }

        // 无人机Sunray和其他Sunray（包括无人机和车）之间的通信（此部分仅针对真机）
        for (int i = 1; i <= uav_experiment_num; i++)
        {
            if (uav_id == i)
                continue;
            //  无人机名字 = 无人机名字前缀 + 无人机ID
            std::string topic_prefix = "/" + uav_name + std::to_string(i);
            // 【发布】无人机状态 其他Sunray智能体 --UDP--> 本节点 --ROS topic--> 本机其他节点
            uav_state_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1))));
        }

        // 无人机Sunray和其他Sunray（包括无人机和车）之间的通信（此部分仅针对真机）
        for (int i = 1; i <= ugv_experiment_num; i++)
        {
            if (ugv_id == i)
                continue;
            //  无人车名字 = 无人车名字前缀 + 无人车ID
            std::string topic_prefix = "/" + ugv_name + std::to_string(i);
            // 【发布】无人车状态  其他Sunray智能体 --UDP--> 本节点 --ROS topic--> 本机其他节点
            ugv_state_pub.insert(std::make_pair(i, (nh.advertise<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 1))));
        }
    }

    // 【定时器】 定时发送TCP心跳包到地面站
    HeartbeatTimer = nh.createTimer(ros::Duration(0.3), &communication_bridge::sendHeartbeatPacket, this);
    // 【定时器】 定时检测启动的子进程是否存活
    CheckChildProcessTimer = nh.createTimer(ros::Duration(0.3), &communication_bridge::CheckChildProcessCallBack, this);

    // 【TCP服务器】 绑定TCP服务器端口
    int back = tcpServer.Bind(static_cast<unsigned short>(std::stoi(tcp_port)));
    // 【TCP服务器】 传入解码器
    tcpServer.setDecoderInterfacePtr(new Codec);
    // 【TCP服务器】 TCP服务器设置监听模式和最大连接数
    tcpServer.Listen(30);
    // 【TCP服务器】 TCP通信：接收TCP消息的回调函数 - 包括：地面站传过来的各种指令
    tcpServer.sigTCPServerReadData.connect(boost::bind(&communication_bridge::TCPServerCallBack, this, _1));
    // 【TCP服务器】 TCP服务器状态信号连接回调函数
    tcpServer.sigLinkState.connect(boost::bind(&communication_bridge::TCPLinkState, this, _1, _2));
    // 【TCP服务器】 TCP服务器启动
    tcpServer.setRunState(true);

    // 【UDP通信】 获取UDP通信对象
    udpSocket = CommunicationUDPSocket::getInstance();
    // 【UDP通信】 设置解码器
    udpSocket->setDecoderInterfacePtr(new Codec);
    // 【UDP通信】 绑定UDP监听端口
    udpSocket->Bind(static_cast<unsigned short>(udp_port));
    // 【UDP通信】 UDP通信：接收UDP消息的回调函数 - 包括：地面站搜索在线智能体、机间通信
    udpSocket->sigUDPUnicastReadData.connect(boost::bind(&communication_bridge::UDPCallBack, this, _1));
    // 【UDP通信】 UDP通信启动
    udpSocket->setRunState(true);

    // 【心跳包】 默认心跳包关闭
    HeartbeatState = false;
}

void communication_bridge::TCPLinkState(bool state, std::string IP)
{
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);
    if (state)
    {
        GSIPHash.insert(IP);
        HeartbeatState = true;
    }
    else
    {
        GSIPHash.erase(IP);
        if (GSIPHash.empty())
            HeartbeatState = false;
    }
}

uint8_t communication_bridge::getPX4ModeEnum(std::string modeStr)
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
    else if (modeStr == "ALTCTL")
        back = PX4ModeType::AltitudeType;
    else if (modeStr == "OFFBOARD")
        back = PX4ModeType::OffboardType;
    else if (modeStr == "POSCTL")
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

void communication_bridge::UDPCallBack(ReceivedParameter readData)
{
    int back;
    // std::cout << " GroundControl::UDPCallBack: " << (int)readData.messageID << std::endl;

    switch (readData.messageID)
    {
    // 搜索在线智能体 - SearchData（#200）
    case MessageID::SearchMessageID:
    {
        // std::cout << "MessageID::SearchMessageID: " << (int)readData.communicationType << " readData.ip: " << readData.ip << " readData.port: " << readData.port << std::endl;
        unionData backData;
        // 仿真无人机应答
        for (int i = uav_id; i < uav_id + uav_simulation_num; i++)
        {
            backData.ack.init();
            backData.ack.robotID = i;
            backData.ack.ID = i;
            backData.ack.agentType = 0;
            backData.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            // std::cout << "应答数据: " << int(backData.ack.ID) << std::endl;
            std::lock_guard<std::mutex> lock(_mutexUDP);
            back = udpSocket->sendUDPData(codec.coder(MessageID::ACKMessageID, backData), readData.ip, (uint16_t)readData.data.search.port);
            // std::cout << "发送结果: " << back << " readData.data.search.port " << readData.data.search.port << std::endl;
        }
        // 仿真无人车应答
        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            backData.ack.init();
            backData.ack.robotID = i;
            backData.ack.ID = i;
            backData.ack.agentType = 1;
            backData.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            std::lock_guard<std::mutex> lock(_mutexUDP);
            back = udpSocket->sendUDPData(codec.coder(MessageID::ACKMessageID, backData), readData.ip, (uint16_t)readData.data.search.port);
        }
        // 真机无人机应答
        if (uav_experiment_num > 0)
        {
            backData.ack.init();
            backData.ack.robotID = uav_id;
            backData.ack.ID = uav_id;
            backData.ack.agentType = 0;
            backData.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            back = udpSocket->sendUDPData(codec.coder(MessageID::ACKMessageID, backData), readData.ip, (uint16_t)readData.data.search.port);
            std::lock_guard<std::mutex> lock(_mutexUDP);
        }
        // 真机无人车应答
        if (ugv_experiment_num > 0)
        {
            backData.ack.init();
            backData.ack.robotID = ugv_id;
            backData.ack.ID = ugv_id;
            backData.ack.agentType = 1;
            backData.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            back = udpSocket->sendUDPData(codec.coder(MessageID::ACKMessageID, backData), readData.ip, (uint16_t)readData.data.search.port);
            std::lock_guard<std::mutex> lock(_mutexUDP);
        }

        break;
    }
    // 无人机状态 - StateData（#2
    case MessageID::StateMessageID:
        // std::cout << " case MessageID::StateMessageID: " << (int)readData.data.state.uavID << std::endl;
        if (uav_id == readData.data.state.uavID || is_simulation)
            return;
        SynchronizationUAVState(readData.data.state);
        break;
    // 无人车状态 - UGVStateData（#20）
    case MessageID::UGVStateMessageID:
        if (ugv_id == readData.data.ugvState.ugvID || is_simulation)
            return;
        SynchronizationUGVState(readData.data.ugvState);
        break;
    default:
        break;
    }
}

pid_t communication_bridge::CheckChildProcess(pid_t pid)
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

pid_t communication_bridge::executeScript(std::string scriptStr, std::string filePath)
{
    pid_t pid = fork();
    if (pid == -1)
        perror("fork failed");
    else if (pid == 0)
    {
        std::string cdCommand = "cd " + getSunrayPath() + filePath;
        std::string fullCommand = cdCommand + " && ./" + scriptStr;

        // 构建打开新终端并执行命令的字符串
        std::string terminalCommand = "gnome-terminal -- bash -c \"" + fullCommand + "; exec bash\"";
        const char *command = terminalCommand.c_str();
        execlp("bash", "bash", "-c", command, (char *)NULL);

        // 如果execlp返回，说明执行失败
        perror("OrderCourse Error!");
        _exit(EXIT_FAILURE);

        // 设置环境变量和参数
        // char *const envp[] = {NULL};
        // char *const argv[] = {(char*)"bash", (char*)"-c", (char*)fullCommand.c_str(), NULL};

        // // 直接执行命令，不打开新终端
        // execve("/bin/bash", argv, envp);

        // // 如果execve返回，说明执行失败
        // perror("OrderCourse Error!");
        // _exit(EXIT_FAILURE);
    }
    else
        printf("This is the parent process. Child PID: %d\n", pid);

    return pid;
}

pid_t communication_bridge::OrderCourse(std::string orderStr)
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

void communication_bridge::executiveDemo(std::string orderStr)
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
        // 另外，cd命令也需要在同一个shell中执行，所以我们不能简单地用&&连接命令
        // 而是需要将它们放在一个bash -c参数中

        std::string temp = "bash -c \"cd /home/yundrone/Sunray && . devel/setup.sh && ";
        temp += orderStr;
        // std::cout << "executiveDemo： " << temp << std::endl;
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

void communication_bridge::TCPServerCallBack(ReceivedParameter readData)
{
    // 需要上锁，这里是TCP服务端的线程，这些数据基本只有这个函数调用，所以目前不上锁
    float roll;
    float pitch;
    uint32_t time_stamp;
    uint8_t robot_id;

    // std::cout << "communication_bridge::TCPServerCallBack:" << readData.messageID << std::endl;
    switch (readData.messageID)
    {
    // 无人机控制指令 - ControlData（#102）
    case MessageID::ControlMessageID:
    {
        time_stamp = readData.data.control.timestamp;

        roll = readData.data.control.roll;
        pitch = readData.data.control.pitch;
        robot_id = readData.data.control.robotID;

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = readData.data.control.controlMode;
        uav_cmd.desired_pos[0] = readData.data.control.position.x;
        uav_cmd.desired_pos[1] = readData.data.control.position.y;
        uav_cmd.desired_pos[2] = readData.data.control.position.z;
        uav_cmd.desired_vel[0] = readData.data.control.velocity.x;
        uav_cmd.desired_vel[1] = readData.data.control.velocity.y;
        uav_cmd.desired_vel[2] = readData.data.control.velocity.z;
        uav_cmd.desired_yaw = readData.data.control.yaw;
        uav_cmd.desired_yaw_rate = readData.data.control.yawRate;

        auto it = control_cmd_pub.find(robot_id);
        if (it == control_cmd_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(uav_cmd);
        break;
    }
    // 无人机模式切换 - VehicleData（#103)
    case MessageID::VehicleMessageID:
    {
        robot_id = readData.data.vehicle.robotID;
        setup.header.stamp = ros::Time::now();
        if (readData.data.vehicle.sunray_mode == VehicleControlType::SetControlMode)
        {
            setup.control_mode = "CMD_CONTROL";
        }
        setup.cmd = readData.data.vehicle.sunray_mode;
        auto it = uav_setup_pub.find(robot_id);
        if (it == uav_setup_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(setup);
        break;
    }
    // 无人机demo - DemoData（#202）
    case MessageID::DemoMessageID:
        if (readData.data.demo.demoState == true)
        {
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
                    std::cout << "该节点未启动： " << readData.data.demo.demoStr << std::endl;
            }
        }
        break;
    // 功能脚本 - ScriptData（#203）
    case MessageID::ScriptMessageID:
        if (readData.data.agentScrip.scriptState == true)
        {
            if (readData.data.agentScrip.scripType == 0)
                executeScript(readData.data.agentScrip.scriptStr, "/scripts_sim/");
            else if (readData.data.agentScrip.scripType == 1)
                executeScript(+readData.data.agentScrip.scriptStr, "/scripts_exp/");
        }
        break;
    // 无人机航点 - WaypointData（#104）
    case MessageID::WaypointMessageID:
    {
        robot_id = readData.data.waypointData.robotID;
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

        auto it = uav_waypoint_pub.find(robot_id);
        if (it == uav_waypoint_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(waypoint_msg);
        break;
    }
    // 无人车控制指令 - UGVControlData（#120）
    case MessageID::UGVControlMessageID:
    {
        robot_id = readData.data.ugvControl.robotID;

        auto it = ugv_controlCMD_pub.find(robot_id);
        if (it == ugv_controlCMD_pub.end())
        {
            std::cout << "controlCMD UGV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        sunray_msgs::UGVControlCMD msg;
        msg.cmd = readData.data.ugvControl.controlMode;
        msg.yaw_type = readData.data.ugvControl.yawType;
        msg.desired_pos[0] = readData.data.ugvControl.desiredPos.x;
        msg.desired_pos[1] = readData.data.ugvControl.desiredPos.y;
        msg.desired_vel[0] = readData.data.ugvControl.desiredVel.x;
        msg.desired_vel[1] = readData.data.ugvControl.desiredVel.y;
        msg.desired_yaw = readData.data.ugvControl.desiredYaw;
        msg.angular_vel = readData.data.ugvControl.angularVel;
        it->second.publish(msg);
        break;
    }
    default:
        break;
    }
    // std::cout << "communication_bridge::TCPServerCallBack end" << std::endl;
}

bool communication_bridge::SynchronizationUGVState(UGVStateData Data)
{
    auto it = ugv_state_pub.find(Data.ugvID);

    if (it == ugv_state_pub.end())
    {
        std::cout << "UGV" + std::to_string(Data.ugvID) + " topic Publisher not found!" << std::endl;
        return false;
    }

    sunray_msgs::UGVState msg;
    msg.ugv_id = Data.ugvID;
    msg.location_source = Data.locationSource;
    msg.connected = Data.connected;
    msg.odom_valid = Data.locationSource;
    msg.odom_valid = Data.odom_valid;
    msg.position[0] = Data.position.x;
    msg.position[1] = Data.position.y;
    msg.velocity[0] = Data.velocity.x;
    msg.velocity[1] = Data.velocity.y;
    msg.yaw = Data.yaw;

    msg.pos_setpoint[0] = Data.posSetpoint.x;
    msg.pos_setpoint[1] = Data.posSetpoint.y;
    msg.vel_setpoint[0] = Data.velSetpoint.x;
    msg.vel_setpoint[1] = Data.velSetpoint.y;
    msg.yaw_setpoint = Data.yawSetpoint;
    msg.battery_state = Data.batteryState;
    msg.battery_percentage = Data.batteryPercentage;
    msg.control_mode = Data.controlMode;

    it->second.publish(msg);
    return true;
}

bool communication_bridge::SynchronizationUAVState(StateData Data)
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
    msg.battery_percentage = Data.batteryPercentage;
    msg.control_mode = Data.controlMode;
    msg.move_mode = Data.moveMode;

    it->second.publish(msg);
    return true;
}

void communication_bridge::CheckChildProcessCallBack(const ros::TimerEvent &e)
{
    std::vector<std::string> keysToRemove;
    for (const auto &pair : nodeMap)
    {
        if (pair.second == CheckChildProcess(pair.second))
            keysToRemove.push_back(pair.first);
    }

    for (const auto &key : keysToRemove)
        nodeMap.erase(key);
}

void communication_bridge::sendHeartbeatPacket(const ros::TimerEvent &e)
{

    if (HeartbeatState)
    {
        unionData Heartbeatdata;
        for (int i = uav_id; i < uav_simulation_num + uav_id; i++)
        {
            Heartbeatdata.heartbeat.robotID = i;
            Heartbeatdata.heartbeat.agentType = UAVType;
            tcpServer.allSendData(codec.coder(MessageID::HeartbeatMessageID, Heartbeatdata));
        }

        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            Heartbeatdata.heartbeat.robotID = i;
            Heartbeatdata.heartbeat.agentType = UGVType;
            tcpServer.allSendData(codec.coder(MessageID::HeartbeatMessageID, Heartbeatdata));
        }
    }
}

// UAVState回调函数 - 将读取到的数据按照id顺序存储在uavStateData数组中
void communication_bridge::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id)
{
    int index = robot_id - 1;
    uavStateData[index].state.init();
    uavStateData[index].state.robotID = robot_id;
    uavStateData[index].state.uavID = robot_id;
    uavStateData[index].state.connected = msg->connected;
    uavStateData[index].state.armed = msg->armed;
    uavStateData[index].state.mode = getPX4ModeEnum(msg->mode);
    uavStateData[index].state.locationSource = msg->location_source;
    uavStateData[index].state.odom_valid = msg->odom_valid;
    uavStateData[index].state.position.x = msg->position[0];
    uavStateData[index].state.position.y = msg->position[1];
    uavStateData[index].state.position.z = msg->position[2];
    uavStateData[index].state.velocity.x = msg->velocity[0];
    uavStateData[index].state.velocity.y = msg->velocity[1];
    uavStateData[index].state.velocity.z = msg->velocity[2];
    uavStateData[index].state.attitude.x = msg->attitude[0];
    uavStateData[index].state.attitude.y = msg->attitude[1];
    uavStateData[index].state.attitude.z = msg->attitude[2];

    uavStateData[index].state.posSetpoint.x = msg->pos_setpoint[0];
    uavStateData[index].state.posSetpoint.y = msg->pos_setpoint[1];
    uavStateData[index].state.posSetpoint.z = msg->pos_setpoint[2];

    uavStateData[index].state.velSetpoint.x = msg->vel_setpoint[0];
    uavStateData[index].state.velSetpoint.y = msg->vel_setpoint[1];
    uavStateData[index].state.velSetpoint.z = msg->vel_setpoint[2];

    uavStateData[index].state.attSetpoint.x = msg->att_setpoint[0];
    uavStateData[index].state.attSetpoint.y = msg->att_setpoint[1];
    uavStateData[index].state.attSetpoint.z = msg->att_setpoint[2];

    uavStateData[index].state.attitudeQuaternion.w = msg->attitude_q.w;
    uavStateData[index].state.attitudeQuaternion.x = msg->attitude_q.x;
    uavStateData[index].state.attitudeQuaternion.y = msg->attitude_q.y;
    uavStateData[index].state.attitudeQuaternion.z = msg->attitude_q.z;

    uavStateData[index].state.batteryState = msg->battery_state;
    uavStateData[index].state.batteryPercentage = msg->battery_percentage;
    uavStateData[index].state.controlMode = msg->control_mode;
    uavStateData[index].state.moveMode = msg->move_mode;

    std::string mode = msg->mode;
    if (mode.length() > 15)
        mode = mode.substr(0, 15);
    else if (mode.length() < 15)
        mode.append(15 - mode.length(), ' ');

    // 本节点 --UDP--> 其他无人机 无人机状态信息组播链路发送
    if (uav_id > 0 && !is_simulation && uav_id == robot_id)
        int back = udpSocket->sendUDPMulticastData(codec.coder(MessageID::StateMessageID, uavStateData[uav_id - 1]), udp_port);

    std::vector<std::string> tempVec;

    // 发送数据到地面站 本节点 --UDP--> 地面站
    // 仿真情况下，所有无人机状态都通过本节点回传（仿真时只有一个通信节点，真机时有N个机载通信节点）
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    for (const auto &ip : GSIPHash)
    {
        // 无人机状态 - StateData（#2）
        int sendBack = udpSocket->sendUDPData(codec.coder(MessageID::StateMessageID, uavStateData[robot_id - 1]), ip, udp_ground_port);
        if (sendBack < 0)
            tempVec.push_back(ip);
    }

    for (const auto &ip : tempVec)
        GSIPHash.erase(ip);
    tempVec.clear();
}

void communication_bridge::ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int robot_id)
{
    int index = robot_id - 1;
    ugvStateData[index].ugvState.init();
    ugvStateData[index].ugvState.robotID = robot_id;
    ugvStateData[index].ugvState.ugvID = robot_id;
    ugvStateData[index].ugvState.locationSource = msg->location_source;
    ugvStateData[index].ugvState.connected = msg->connected;
    ugvStateData[index].ugvState.odom_valid = msg->odom_valid;
    ugvStateData[index].ugvState.position.x = msg->position[0];
    ugvStateData[index].ugvState.position.y = msg->position[1];
    ugvStateData[index].ugvState.velocity.x = msg->velocity[0];
    ugvStateData[index].ugvState.velocity.y = msg->velocity[1];
    ugvStateData[index].ugvState.yaw = msg->yaw;
    ugvStateData[index].ugvState.posSetpoint.x = msg->pos_setpoint[0];
    ugvStateData[index].ugvState.posSetpoint.y = msg->pos_setpoint[1];
    ugvStateData[index].ugvState.velSetpoint.x = msg->vel_setpoint[0];
    ugvStateData[index].ugvState.velSetpoint.y = msg->vel_setpoint[1];
    ugvStateData[index].ugvState.yawSetpoint = msg->yaw_setpoint;
    ugvStateData[index].ugvState.batteryState = msg->battery_state;
    ugvStateData[index].ugvState.batteryPercentage = msg->battery_percentage;
    ugvStateData[index].ugvState.controlMode = msg->control_mode;

    // 本节点 --UDP--> 其他无人车 无人车状态信息组播链路发送
    if (ugv_id > 0 && !is_simulation && ugv_id == robot_id)
        int back = udpSocket->sendUDPMulticastData(codec.coder(MessageID::UGVStateMessageID, ugvStateData[ugv_id - 1]), udp_port);

    std::vector<std::string> tempVec;
    // 发送数据到地面站 本节点 --UDP--> 地面站
    // 仿真情况下，所有无人机状态都通过本节点回传（仿真时只有一个通信节点，真机时有N个机载通信节点）
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    for (const auto &ip : GSIPHash)
    {
        // 无人车状态 - UGVStateData（#20）
        int sendBack = udpSocket->sendUDPData(codec.coder(MessageID::UGVStateMessageID, ugvStateData[robot_id - 1]), ip, udp_ground_port);
        if (sendBack < 0)
            tempVec.push_back(ip);
    }

    for (const auto &ip : tempVec)
        GSIPHash.erase(ip);
    tempVec.clear();
}

std::string communication_bridge::getUserDirectoryPath()
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);
    if (pw != nullptr)
    {
        // std::cout << "用户目录: " << pw->pw_dir << std::endl;
        return pw->pw_dir;
    }

    std::cerr << "无法获取用户目录" << std::endl;
    return "";
}

std::string communication_bridge::getCurrentProgramPath()
{
    char buffer[PATH_MAX];
    ssize_t len = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len != -1)
    {
        buffer[len] = '\0';
        return std::string(buffer);
    }
    return "";
}

std::string communication_bridge::getSunrayPath()
{
    std::string fullPath = getCurrentProgramPath();
    //  size_t pos = fullPath.find("/Sunray");
    // if (pos != std::string::npos) {
    //     return fullPath.substr(0, pos + 7); // 7 是 "/Sunray" 的长度
    // }
    // return "";
    size_t pos = fullPath.find("/devel/lib/sunray_communication_bridge/communication_bridge_node");
    if (pos != std::string::npos)
    {
        return fullPath.substr(0, pos);
    }
    return fullPath;
}

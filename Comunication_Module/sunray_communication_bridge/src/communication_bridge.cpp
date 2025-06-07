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
    }else{
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

    // 【订阅】编队切换
    formation_sub=nh.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd", 1, boost::bind(&communication_bridge::formation_cmd_cb, this,_1));
    // 【发布】编队切换
    formation_pub=nh.advertise<sunray_msgs::Formation>("/sunray/formation_cmd/ground", 1);

    // 【定时器】 定时发送TCP心跳包到地面站
    HeartbeatTimer = nh.createTimer(ros::Duration(0.3), &communication_bridge::sendHeartbeatPacket, this);
    // 【定时器】 定时检测启动的子进程是否存活
    CheckChildProcessTimer = nh.createTimer(ros::Duration(0.3), &communication_bridge::CheckChildProcessCallBack, this);
    // 【定时器】 定时发送ROS节点信息到地面站
    UpdateROSNodeInformationTimer= nh.createTimer(ros::Duration(1), &communication_bridge::UpdateROSNodeInformation, this);


    // 【TCP服务器】 绑定TCP服务器端口
    int back = tcpServer.Bind(static_cast<unsigned short>(std::stoi(tcp_port)));
    // 【TCP服务器】 传入解码器
    tcpServer.setDecoderInterfacePtr(new Codec);
    // 【TCP服务器】 TCP服务器设置监听模式和最大连接数
    tcpServer.Listen(30);
    // 【TCP服务器】 TCP通信：接收TCP消息的回调函数 - 包括：地面站传过来的各种指令
    tcpServer.sigTCPServerReadData.connect(boost::bind(&communication_bridge::TCPServerCallBack, this, _1));
    // 【TCP服务器】 TCP服务器状态信号连接回调函数(地面站连接状态变化时触发) - 包括：地面站连接、断开连接
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

    // 变量 - 是否与地面站建立连接
    station_connected = false;
}

void communication_bridge::TCPLinkState(bool state, std::string IP)
{
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    // 地面站连接后，则开启心跳包；地面站断开后，则关闭心跳包
    if (state)
    {
        GSIPHash.insert(IP);
        station_connected = true;
    }
    else
    {
        GSIPHash.erase(IP);
        if (GSIPHash.empty())
            station_connected = false;
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
    //  std::cout << " GroundControl::UDPCallBack: " << (int)readData.dataFrame.seq << std::endl;

    switch (readData.dataFrame.seq)
    {
    case MessageID::SearchMessageID:// 搜索在线智能体 - SearchData（#200）
    {
        // std::cout << "MessageID::SearchMessageID: " << (int)readData.communicationType << " readData.ip: " << readData.ip << " readData.port: " << readData.port << std::endl;
        DataFrame backData;
        backData.seq=MessageID::ACKMessageID;
        backData.data.ack.init();
        // 仿真无人机应答
        for (int i = uav_id; i < uav_id + uav_simulation_num; i++)
        {
            backData.data.ack.init();
            backData.robot_ID = i;
            backData.data.ack.ID = i;
            backData.data.ack.agentType = 0;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            // std::cout << "应答数据: " << int(backData.ack.ID) << std::endl;
            std::lock_guard<std::mutex> lock(_mutexUDP);
            back = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
            //  std::cout << "发送结果: " << back << std::endl;
        }
        // 仿真无人车应答
        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            backData.data.ack.init();
            backData.robot_ID = i+100;
            backData.data.ack.ID = i;
            backData.data.ack.agentType = 1;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            std::lock_guard<std::mutex> lock(_mutexUDP);
            back = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
        }
        // 真机无人机应答
        if (uav_experiment_num > 0)
        {
            backData.robot_ID = uav_id;
            backData.data.ack.ID = uav_id;
            backData.data.ack.agentType = 0;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            back = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
            std::lock_guard<std::mutex> lock(_mutexUDP);
        }
        // 真机无人车应答
        if (ugv_experiment_num > 0)
        {
            backData.robot_ID = ugv_id+100;
            backData.data.ack.ID = ugv_id;
            backData.data.ack.agentType = 1;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            back = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
            std::lock_guard<std::mutex> lock(_mutexUDP);
        }

        break;
    }
    case MessageID::UAVStateMessageID:    // 无人机状态 - UAVState#2
        // std::cout << " case MessageID::StateMessageID: " << (int)readData.data.state.uavID << std::endl;
        if (uav_id == readData.dataFrame.robot_ID || is_simulation)
            return;
        SynchronizationUAVState(readData.dataFrame.data.uavState);
        break;
    case MessageID::UGVStateMessageID:// 无人车状态 - UGVState（#20）
        if (ugv_id == readData.dataFrame.robot_ID-100 || is_simulation)
            return;
        SynchronizationUGVState(readData.dataFrame.data.ugvState);
        break;
    case MessageID::FormationMessageID:// 编队切换 - Formation（#40）
    {    
        sunray_msgs::Formation sendMSG;
        sendMSG.cmd=readData.dataFrame.data.formation.cmd;
        sendMSG.formation_type=readData.dataFrame.data.formation.formation_type;
        sendMSG.name=readData.dataFrame.data.formation.name;
        formation_pub.publish(sendMSG);
        break;   
    }     
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
    }else if (pid == 0){
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
    }else{
        // 父进程
        int status;
        demoPID = pid;
        printf("This is the parent process. Child PID: %d\n", pid);
    }
}

void communication_bridge::TCPServerCallBack(ReceivedParameter readData)
{
    // 需要上锁，这里是TCP服务端的线程，这些数据基本只有这个函数调用，所以目前不上锁
    uint32_t time_stamp;
    uint8_t robot_id;

    time_stamp = readData.dataFrame.timestamp;
    if( readData.dataFrame.robot_ID<=100)
        robot_id = readData.dataFrame.robot_ID;
    else
        robot_id = readData.dataFrame.robot_ID-100;
    // std::cout << "communication_bridge::TCPServerCallBack:" << (int)readData.dataFrame.seq<< std::endl;
    switch (readData.dataFrame.seq)
    {
    case MessageID::UAVControlCMDMessageID:// 无人机控制指令 - UAVControlCMD（#102）
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = readData.dataFrame.data.uavControlCMD.cmd;
        uav_cmd.desired_pos[0] = readData.dataFrame.data.uavControlCMD.desired_pos[0];
        uav_cmd.desired_pos[1] = readData.dataFrame.data.uavControlCMD.desired_pos[1];
        uav_cmd.desired_pos[2] = readData.dataFrame.data.uavControlCMD.desired_pos[2];

        uav_cmd.desired_vel[0] = readData.dataFrame.data.uavControlCMD.desired_vel[0];
        uav_cmd.desired_vel[1] = readData.dataFrame.data.uavControlCMD.desired_vel[1];
        uav_cmd.desired_vel[2] = readData.dataFrame.data.uavControlCMD.desired_vel[2];

        uav_cmd.desired_acc[0] = readData.dataFrame.data.uavControlCMD.desired_acc[0];
        uav_cmd.desired_acc[1] = readData.dataFrame.data.uavControlCMD.desired_acc[1];
        uav_cmd.desired_acc[2] = readData.dataFrame.data.uavControlCMD.desired_acc[2];  

        uav_cmd.desired_jerk[0] = readData.dataFrame.data.uavControlCMD.desired_jerk[0];
        uav_cmd.desired_jerk[1] = readData.dataFrame.data.uavControlCMD.desired_jerk[1];
        uav_cmd.desired_jerk[2] = readData.dataFrame.data.uavControlCMD.desired_jerk[2]; 

        uav_cmd.desired_att[0] = readData.dataFrame.data.uavControlCMD.desired_att[0];
        uav_cmd.desired_att[1] = readData.dataFrame.data.uavControlCMD.desired_att[1];
        uav_cmd.desired_att[2] = readData.dataFrame.data.uavControlCMD.desired_att[2];
  
        uav_cmd.desired_thrust = readData.dataFrame.data.uavControlCMD.desired_thrust;
        uav_cmd.desired_yaw = readData.dataFrame.data.uavControlCMD.desired_yaw;
        uav_cmd.desired_yaw_rate = readData.dataFrame.data.uavControlCMD.desired_yaw_rate;
        uav_cmd.latitude = readData.dataFrame.data.uavControlCMD.latitude;
        uav_cmd.longitude = readData.dataFrame.data.uavControlCMD.longitude;
        uav_cmd.altitude = readData.dataFrame.data.uavControlCMD.altitude;


        auto it = control_cmd_pub.find(robot_id);
        if (it == control_cmd_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(uav_cmd);
        break;
    }
    case MessageID::UAVSetupMessageID:// 无人机设置指令 - UAVSetup（#103）
    {
        setup.header.stamp = ros::Time::now();
        if (readData.dataFrame.data.uavSetup.control_mode == UAVSetupType::SetControlMode)
        {
            setup.control_mode = "CMD_CONTROL";
        }
        setup.cmd = readData.dataFrame.data.uavSetup.cmd;
        auto it = uav_setup_pub.find(robot_id);
        if (it == uav_setup_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(setup);
        break;
    }
    case MessageID::DemoMessageID:// 无人机demo - DemoData（#202）
        if(!is_simulation)
        {
            if(readData.dataFrame.robot_ID<=100)
            {
                if(uav_id!=robot_id)
                    break;
            }else{
                if(ugv_id!=robot_id)
                    break;
            }
        }
        if (readData.dataFrame.data.demo.demoState == true)
        {
            if (readData.dataFrame.data.demo.demoSize > 0)
            {
                auto it = nodeMap.find(readData.dataFrame.data.demo.demoStr);
                if (it != nodeMap.end())
                {
                    std::cout << "该节点已启动： " << readData.dataFrame.data.demo.demoSize << std::endl;
                }else{
                    pid_t back = OrderCourse(readData.dataFrame.data.demo.demoStr);
                    if (back > 0)
                        nodeMap.insert(std::make_pair(readData.dataFrame.data.demo.demoStr, back));
                }
            }
        }else{
            if (readData.dataFrame.data.demo.demoSize > 0)
            {
                std::cout << "TCPServer 关闭Demo： " << std::endl;
                auto it = nodeMap.find(readData.dataFrame.data.demo.demoStr);
                if (it != nodeMap.end())
                {
                    pid_t temp = it->second;
                    if (kill(temp, SIGTERM) != 0)
                    {
                        perror("kill failed!");
                    }else{
                        printf("Sent SIGTERM to child process %d\n", temp);
                        nodeMap.erase(readData.dataFrame.data.demo.demoStr);
                    }
                }
                else
                    std::cout << "该节点未启动： " << readData.dataFrame.data.demo.demoStr << std::endl;
            }
        }
        break;
    case MessageID::ScriptMessageID:// 功能脚本 - ScriptData（#203）
        if(!is_simulation)
        {
            if(readData.dataFrame.robot_ID<=100)
            {
                if(uav_id!=robot_id)
                    break;
            }else{
                if(ugv_id!=robot_id)
                    break;
            }
        }
        if (readData.dataFrame.data.agentScrip.scriptState == true)
        {
            if (readData.dataFrame.data.agentScrip.scripType == 0)
                executeScript(readData.dataFrame.data.agentScrip.scriptStr, "/scripts_sim/");
            else if (readData.dataFrame.data.agentScrip.scripType == 1)
                executeScript(+readData.dataFrame.data.agentScrip.scriptStr, "/scripts_exp/");
        }
        break;
    case MessageID::WaypointMessageID:// 无人机航点 - WaypointData（#104）
    {
        sunray_msgs::UAVWayPoint waypoint_msg;
        waypoint_msg.header.stamp = ros::Time::now();
        waypoint_msg.wp_num = readData.dataFrame.data.waypointData.wp_num;
        waypoint_msg.wp_type = readData.dataFrame.data.waypointData.wp_type;
        waypoint_msg.wp_end_type = readData.dataFrame.data.waypointData.wp_end_type;
        waypoint_msg.wp_takeoff = readData.dataFrame.data.waypointData.wp_takeoff;
        waypoint_msg.wp_yaw_type = readData.dataFrame.data.waypointData.wp_yaw_type;
        waypoint_msg.wp_move_vel = readData.dataFrame.data.waypointData.wp_move_vel;
        waypoint_msg.wp_vel_p = readData.dataFrame.data.waypointData.wp_vel_p;
        waypoint_msg.z_height = readData.dataFrame.data.waypointData.z_height;

        // std::cout << "MessageID::WaypointMessageID 2 "<<readData.data.waypointData.Waypoint1.X<< std::endl; wp_point_1[0]

        waypoint_msg.wp_point_1 = {readData.dataFrame.data.waypointData.wp_point_1[0], readData.dataFrame.data.waypointData.wp_point_1[1],
                                   readData.dataFrame.data.waypointData.wp_point_1[2], readData.dataFrame.data.waypointData.wp_point_1[3]};
        waypoint_msg.wp_point_2 = {readData.dataFrame.data.waypointData.wp_point_2[0], readData.dataFrame.data.waypointData.wp_point_2[1],
                                   readData.dataFrame.data.waypointData.wp_point_2[2], readData.dataFrame.data.waypointData.wp_point_2[3]};
        waypoint_msg.wp_point_3 = {readData.dataFrame.data.waypointData.wp_point_3[0], readData.dataFrame.data.waypointData.wp_point_3[1],
                                   readData.dataFrame.data.waypointData.wp_point_3[2], readData.dataFrame.data.waypointData.wp_point_3[3]};
        waypoint_msg.wp_point_4 = {readData.dataFrame.data.waypointData.wp_point_4[0], readData.dataFrame.data.waypointData.wp_point_4[1],
                                   readData.dataFrame.data.waypointData.wp_point_4[2], readData.dataFrame.data.waypointData.wp_point_4[3]};
        waypoint_msg.wp_point_5 = {readData.dataFrame.data.waypointData.wp_point_5[0], readData.dataFrame.data.waypointData.wp_point_5[1],
                                   readData.dataFrame.data.waypointData.wp_point_5[2], readData.dataFrame.data.waypointData.wp_point_5[3]};
        waypoint_msg.wp_point_6 = {readData.dataFrame.data.waypointData.wp_point_6[0], readData.dataFrame.data.waypointData.wp_point_6[1],
                                   readData.dataFrame.data.waypointData.wp_point_6[2], readData.dataFrame.data.waypointData.wp_point_6[3]};
        waypoint_msg.wp_point_7 = {readData.dataFrame.data.waypointData.wp_point_7[0], readData.dataFrame.data.waypointData.wp_point_7[1],
                                   readData.dataFrame.data.waypointData.wp_point_7[2], readData.dataFrame.data.waypointData.wp_point_7[3]};
        waypoint_msg.wp_point_8 = {readData.dataFrame.data.waypointData.wp_point_8[0], readData.dataFrame.data.waypointData.wp_point_8[1],
                                   readData.dataFrame.data.waypointData.wp_point_8[2], readData.dataFrame.data.waypointData.wp_point_8[3]};
        waypoint_msg.wp_point_9 = {readData.dataFrame.data.waypointData.wp_point_9[0], readData.dataFrame.data.waypointData.wp_point_9[1],
                                   readData.dataFrame.data.waypointData.wp_point_9[2], readData.dataFrame.data.waypointData.wp_point_9[3]};
        waypoint_msg.wp_point_10 = {readData.dataFrame.data.waypointData.wp_point_10[0], readData.dataFrame.data.waypointData.wp_point_10[1],
                                    readData.dataFrame.data.waypointData.wp_point_10[2], readData.dataFrame.data.waypointData.wp_point_10[3]};
        waypoint_msg.wp_circle_point = {readData.dataFrame.data.waypointData.wp_circle_point[0], readData.dataFrame.data.waypointData.wp_circle_point[1]};

        auto it = uav_waypoint_pub.find(robot_id);
        if (it == uav_waypoint_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        it->second.publish(waypoint_msg);
        break;
    }
    case MessageID::UGVControlCMDMessageID:// 无人车控制指令 - UGVControlCMD （#120）
    {

        auto it = ugv_controlCMD_pub.find(robot_id);
        if (it == ugv_controlCMD_pub.end())
        {
            std::cout << "controlCMD UGV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        sunray_msgs::UGVControlCMD msg;
        msg.cmd = readData.dataFrame.data.ugvControlCMD.cmd;
        msg.yaw_type = readData.dataFrame.data.ugvControlCMD.yaw_type;
        msg.desired_pos[0] = readData.dataFrame.data.ugvControlCMD.desired_pos[0];
        msg.desired_pos[1] = readData.dataFrame.data.ugvControlCMD.desired_pos[1];
        msg.desired_vel[0] = readData.dataFrame.data.ugvControlCMD.desired_vel[0];
        msg.desired_vel[1] = readData.dataFrame.data.ugvControlCMD.desired_vel[1];
        msg.desired_yaw = readData.dataFrame.data.ugvControlCMD.desired_yaw;
        msg.angular_vel = readData.dataFrame.data.ugvControlCMD.angular_vel;
        it->second.publish(msg);
        break;
    }
    case MessageID::FormationMessageID:// 编队切换 - Formation（#40）
    {    
        sunray_msgs::Formation sendMSG;
        sendMSG.cmd=readData.dataFrame.data.formation.cmd;
        sendMSG.formation_type=readData.dataFrame.data.formation.formation_type;
        sendMSG.name=readData.dataFrame.data.formation.name;
        formation_pub.publish(sendMSG);
        break;   
    }  
    default:
        break;
    }
    // std::cout << "communication_bridge::TCPServerCallBack end" << std::endl;
}

bool communication_bridge::SynchronizationUGVState(UGVState Data)
{
    auto it = ugv_state_pub.find(Data.ugv_id);

    if (it == ugv_state_pub.end())
    {
        std::cout << "UGV" + std::to_string(Data.ugv_id) + " topic Publisher not found!" << std::endl;
        return false;
    }

    sunray_msgs::UGVState msg;
    msg.ugv_id = Data.ugv_id;
    msg.connected = Data.connected;
    msg.battery_state = Data.battery_state;
    msg.battery_percentage = Data.battery_percentage;
    msg.location_source = Data.location_source;
    msg.odom_valid = Data.odom_valid;
    msg.position[0] = Data.position[0];
    msg.position[1] = Data.position[1];
    msg.velocity[0] = Data.velocity[0];
    msg.velocity[1] = Data.velocity[1];
    msg.yaw = Data.yaw;

    msg.attitude[0] = Data.attitude[0];
    msg.attitude[1] = Data.attitude[1];
    msg.attitude[2] = Data.attitude[2];

    msg.attitude_q.x = Data.attitude_q[0];
    msg.attitude_q.y = Data.attitude_q[1];
    msg.attitude_q.z = Data.attitude_q[2];
    msg.attitude_q.w = Data.attitude_q[3];

    msg.control_mode = Data.control_mode;

    msg.pos_setpoint[0] = Data.pos_setpoint[0];
    msg.pos_setpoint[1] = Data.pos_setpoint[1];
    msg.vel_setpoint[0] = Data.vel_setpoint[0];
    msg.vel_setpoint[1] = Data.vel_setpoint[1];
    msg.yaw_setpoint = Data.yaw_setpoint;

    msg.home_pos[0] = Data.home_pos[0];
    msg.home_pos[1] = Data.home_pos[1];
    msg.home_yaw = Data.home_yaw;

    msg.hover_pos[0] = Data.hover_pos[0];
    msg.hover_pos[1] = Data.hover_pos[1];
    msg.hover_yaw = Data.hover_yaw;

    it->second.publish(msg);
    return true;
}

bool communication_bridge::SynchronizationUAVState(UAVState Data)
{

    auto it = uav_state_pub.find(Data.uav_id);

    if (it == uav_state_pub.end())
    {
        std::cout << "UAV" + std::to_string(Data.uav_id) + " topic Publisher not found!" << std::endl;
        return false;
    }

    sunray_msgs::UAVState msg;
    msg.uav_id = Data.uav_id;
    msg.connected = Data.connected;
    msg.armed = Data.armed;
    msg.mode = Data.mode;
    msg.landed_state = Data.landed_state;
    msg.battery_state = Data.battery_state;
    msg.battery_percentage = Data.battery_percentage;
    msg.location_source = Data.location_source;
    msg.odom_valid = Data.odom_valid;
    msg.position[0] = Data.position[0];
    msg.position[1] = Data.position[1];
    msg.position[2] = Data.position[2];
    msg.velocity[0] = Data.velocity[0];
    msg.velocity[1] = Data.velocity[1];
    msg.velocity[2] = Data.velocity[2];
    msg.attitude[0] = Data.attitude[0];
    msg.attitude[1] = Data.attitude[1];
    msg.attitude[2] = Data.attitude[2];
    msg.attitude_q.w = Data.attitude_q[0];
    msg.attitude_q.x = Data.attitude_q[1];
    msg.attitude_q.y = Data.attitude_q[2];
    msg.attitude_q.z = Data.attitude_q[3];
    msg.attitude_rate[0] = Data.attitude_rate[0];
    msg.attitude_rate[1] = Data.attitude_rate[1];
    msg.attitude_rate[2] = Data.attitude_rate[2];
    msg.pos_setpoint[0] = Data.pos_setpoint[0];
    msg.pos_setpoint[1] = Data.pos_setpoint[1];
    msg.pos_setpoint[2] = Data.pos_setpoint[2];
    msg.vel_setpoint[0] = Data.vel_setpoint[0];
    msg.vel_setpoint[1] = Data.vel_setpoint[1];
    msg.vel_setpoint[2] = Data.vel_setpoint[2];
    msg.att_setpoint[0] = Data.att_setpoint[0];
    msg.att_setpoint[1] = Data.att_setpoint[1];
    msg.att_setpoint[2] = Data.att_setpoint[2];

    msg.thrust_setpoint = Data.thrust_setpoint;
    msg.control_mode = Data.control_mode;
    msg.move_mode = Data.move_mode;
    msg.takeoff_height = Data.takeoff_height;

    msg.home_pos[0] = Data.home_pos[0];
    msg.home_pos[1] = Data.home_pos[1];
    msg.home_pos[2] = Data.home_pos[2];
    msg.home_yaw = Data.home_yaw;

    msg.hover_pos[0] = Data.hover_pos[0];
    msg.hover_pos[1] = Data.hover_pos[1];
    msg.hover_pos[2] = Data.hover_pos[2];
    msg.hover_yaw = Data.hover_yaw;

    msg.land_pos[0] = Data.land_pos[0];
    msg.land_pos[1] = Data.land_pos[1];
    msg.land_pos[2] = Data.land_pos[2];
    msg.land_yaw = Data.land_yaw;

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

void communication_bridge::UpdateROSNodeInformation(const ros::TimerEvent &e)
{
    // 用于存储节点名称的向量
    std::vector<std::string> node_list;
    // 获取节点列表
    if (ros::master::getNodes(node_list))
    {
        if (is_simulation)
        {
            //
            for (int i = uav_id; i < uav_id + uav_simulation_num; i++)
            {
                for (size_t NodeNumber = 0; NodeNumber < node_list.size(); ++NodeNumber)
                {
                    // std::cout << "Index " << NodeNumber << ": " << node_list[NodeNumber] << std::endl;
                    uavOnlineNodeData[i].seq=MessageID::NodeMessageID;
                    uavOnlineNodeData[i].robot_ID=i;
                    uavOnlineNodeData[i].data.nodeInformation.init();

                    uavOnlineNodeData[i].data.nodeInformation.nodeCount = static_cast<uint16_t>(node_list.size());
                    uavOnlineNodeData[i].data.nodeInformation.nodeID = NodeNumber + 1;
                    uavOnlineNodeData[i].data.nodeInformation.nodeSize = node_list[NodeNumber].size();
                    node_list[NodeNumber].copy(uavOnlineNodeData[i].data.nodeInformation.nodeStr, node_list[NodeNumber].size());

                    // 机载电脑ROS节点 -NodeData（#30）
                    SendUdpDataToAllOnlineGroundStations(uavOnlineNodeData[i]);
                    
                }
            }

            //
            for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
            {
                for (size_t NodeNumber = 0; NodeNumber < node_list.size(); ++NodeNumber)
                {
                    ugvOnlineNodeData[i].seq=MessageID::NodeMessageID;
                    ugvOnlineNodeData[i].robot_ID=i+100;
                    ugvOnlineNodeData[i].data.nodeInformation.init();

                    ugvOnlineNodeData[i].data.nodeInformation.nodeCount = static_cast<uint16_t>(node_list.size());
                    ugvOnlineNodeData[i].data.nodeInformation.nodeID = NodeNumber + 1;
                    ugvOnlineNodeData[i].data.nodeInformation.nodeSize = node_list[NodeNumber].size();
                    node_list[NodeNumber].copy(ugvOnlineNodeData[i].data.nodeInformation.nodeStr, node_list[NodeNumber].size());

                    // 机载电脑ROS节点 -NodeData（#30）
                    SendUdpDataToAllOnlineGroundStations(ugvOnlineNodeData[i]);
                    
                }
            }
        }else{
            if (uav_experiment_num > 0)
            {
                for (size_t NodeNumber = 0; NodeNumber < node_list.size(); ++NodeNumber)
                {
                    // std::cout << "Index " << NodeNumber << ": " << node_list[NodeNumber] << std::endl;
                    uavOnlineNodeData[uav_id].seq=MessageID::NodeMessageID;
                    uavOnlineNodeData[uav_id].robot_ID=uav_id;
                    uavOnlineNodeData[uav_id].data.nodeInformation.init();

                    uavOnlineNodeData[uav_id].data.nodeInformation.nodeCount = static_cast<uint16_t>(node_list.size());
                    uavOnlineNodeData[uav_id].data.nodeInformation.nodeID = NodeNumber + 1;
                    uavOnlineNodeData[uav_id].data.nodeInformation.nodeSize = node_list[NodeNumber].size();
                    node_list[NodeNumber].copy(uavOnlineNodeData[uav_id].data.nodeInformation.nodeStr, node_list[NodeNumber].size());

                    // 机载电脑ROS节点 -NodeData（#30）
                    SendUdpDataToAllOnlineGroundStations(uavOnlineNodeData[uav_id]);
                    
                }
            }

            if (ugv_experiment_num > 0)
            {
                for (size_t NodeNumber = 0; NodeNumber < node_list.size(); ++NodeNumber)
                {
                    ugvOnlineNodeData[ugv_id].seq=MessageID::NodeMessageID;
                    ugvOnlineNodeData[ugv_id].robot_ID=ugv_id+100;
                    ugvOnlineNodeData[ugv_id].data.nodeInformation.init();

                    ugvOnlineNodeData[ugv_id].data.nodeInformation.nodeCount = static_cast<uint16_t>(node_list.size());
                    ugvOnlineNodeData[ugv_id].data.nodeInformation.nodeID = NodeNumber + 1;
                    ugvOnlineNodeData[ugv_id].data.nodeInformation.nodeSize = node_list[NodeNumber].size();
                    node_list[NodeNumber].copy(ugvOnlineNodeData[ugv_id].data.nodeInformation.nodeStr, node_list[NodeNumber].size());

                    // 机载电脑ROS节点 -NodeData（#30）
                    SendUdpDataToAllOnlineGroundStations(ugvOnlineNodeData[ugv_id]);
                }
            }
        }
    }else{
         std::cout <<"无法获取ROS节点列表。请确保ROS Master正在运行!"<< std::endl;
    }
}

void communication_bridge::SendUdpDataToAllOnlineGroundStations(DataFrame data)
{
    std::vector<std::string> tempVec;

    // 发送数据到地面站 本节点 --UDP--> 地面站
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    for (const auto &ip : GSIPHash)
    {
        // 机载电脑ROS节点 -NodeData（#30）
        int sendBack = udpSocket->sendUDPData(codec.coder(data), ip, udp_ground_port);
        if (sendBack < 0)
            tempVec.push_back(ip);
    }

    for (const auto &ip : tempVec)
        GSIPHash.erase(ip);
    tempVec.clear();
}

// 定时发送心跳包（TCP）
void communication_bridge::sendHeartbeatPacket(const ros::TimerEvent &e)
{
    // 如果没有与地面站连接，则不发送心跳包
    if(!station_connected)
        return;
    

    // 心跳包数据结构
    DataFrame Heartbeatdata;
    Heartbeatdata.seq=MessageID::HeartbeatMessageID;
    Heartbeatdata.data.heartbeat.init();
    
    if (is_simulation)
    {
        // 无人机心跳包 - 仿真发送
        for (int i = uav_id; i < uav_simulation_num + uav_id; i++)
        {
            Heartbeatdata.robot_ID = i;
            Heartbeatdata.data.heartbeat.agentType = UAVType;
            tcpServer.allSendData(codec.coder( Heartbeatdata));
        }

        // 无人车心跳包 - 仿真发送
        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            Heartbeatdata.robot_ID = i+100;
            Heartbeatdata.data.heartbeat.agentType = UGVType;
            tcpServer.allSendData(codec.coder(Heartbeatdata));
        }
    }else{
        // 无人机心跳包 - 真机发送
        if (uav_experiment_num > 0)
        {
            Heartbeatdata.robot_ID = uav_id;
            Heartbeatdata.data.heartbeat.agentType = UAVType;
            tcpServer.allSendData(codec.coder( Heartbeatdata));
        }
        // 无人车心跳包 - 真机发送
        if (ugv_experiment_num > 0)
        {
            Heartbeatdata.robot_ID = ugv_id+100;
            Heartbeatdata.data.heartbeat.agentType = UGVType;
            tcpServer.allSendData(codec.coder(Heartbeatdata));
        }
    }
}

    
    

// UAVState回调函数 - 将读取到的数据按照id顺序存储在uavStateData数组中
void communication_bridge::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id)
{
    int index = robot_id - 1;
    uavStateData[index].data.uavState.init();
    uavStateData[index].robot_ID = robot_id;
    uavStateData[index].data.uavState.uav_id =msg->uav_id;
    uavStateData[index].data.uavState.connected = msg->connected;
    uavStateData[index].data.uavState.armed = msg->armed;
    uavStateData[index].data.uavState.mode = getPX4ModeEnum(msg->mode);
    uavStateData[index].data.uavState.landed_state= msg->landed_state;
    uavStateData[index].data.uavState.battery_state = msg->battery_state;
    uavStateData[index].data.uavState.battery_percentage = msg->battery_percentage;

    uavStateData[index].data.uavState.location_source = msg->location_source;
    uavStateData[index].data.uavState.odom_valid = msg->odom_valid;

    uavStateData[index].data.uavState.position[0] = msg->position[0];
    uavStateData[index].data.uavState.position[1] = msg->position[1];
    uavStateData[index].data.uavState.position[2] = msg->position[2];
    uavStateData[index].data.uavState.velocity[0] = msg->velocity[0];
    uavStateData[index].data.uavState.velocity[1] = msg->velocity[1];
    uavStateData[index].data.uavState.velocity[2] = msg->velocity[2];
    uavStateData[index].data.uavState.attitude[0] = msg->attitude[0];
    uavStateData[index].data.uavState.attitude[1] = msg->attitude[1];
    uavStateData[index].data.uavState.attitude[2] = msg->attitude[2];

    uavStateData[index].data.uavState.attitude_q[0] = msg->attitude_q.w;
    uavStateData[index].data.uavState.attitude_q[1] = msg->attitude_q.x;
    uavStateData[index].data.uavState.attitude_q[2] = msg->attitude_q.y;
    uavStateData[index].data.uavState.attitude_q[3] = msg->attitude_q.z;

    uavStateData[index].data.uavState.attitude_rate[0] = msg->attitude_rate[0];
    uavStateData[index].data.uavState.attitude_rate[1] = msg->attitude_rate[1];
    uavStateData[index].data.uavState.attitude_rate[2] = msg->attitude_rate[2];

    uavStateData[index].data.uavState.pos_setpoint[0] = msg->pos_setpoint[0];
    uavStateData[index].data.uavState.pos_setpoint[1] = msg->pos_setpoint[1];
    uavStateData[index].data.uavState.pos_setpoint[2] = msg->pos_setpoint[2];

    uavStateData[index].data.uavState.vel_setpoint[0] = msg->vel_setpoint[0];
    uavStateData[index].data.uavState.vel_setpoint[1] = msg->vel_setpoint[1];
    uavStateData[index].data.uavState.vel_setpoint[2] = msg->vel_setpoint[2];

    uavStateData[index].data.uavState.att_setpoint[0] = msg->att_setpoint[0];
    uavStateData[index].data.uavState.att_setpoint[1] = msg->att_setpoint[1];
    uavStateData[index].data.uavState.att_setpoint[2] = msg->att_setpoint[2];

    uavStateData[index].data.uavState.thrust_setpoint = msg->thrust_setpoint;

    uavStateData[index].data.uavState.control_mode = msg->control_mode;
    uavStateData[index].data.uavState.move_mode = msg->move_mode;

    uavStateData[index].data.uavState.takeoff_height= msg->takeoff_height;

    uavStateData[index].data.uavState.home_pos[0] = msg->home_pos[0];
    uavStateData[index].data.uavState.home_pos[1] = msg->home_pos[1];
    uavStateData[index].data.uavState.home_pos[2] = msg->home_pos[2];
    uavStateData[index].data.uavState.home_yaw= msg->home_yaw;

    uavStateData[index].data.uavState.hover_pos[0] = msg->hover_pos[0];
    uavStateData[index].data.uavState.hover_pos[1] = msg->hover_pos[1];
    uavStateData[index].data.uavState.hover_pos[2] = msg->hover_pos[2];
    uavStateData[index].data.uavState.hover_yaw= msg->hover_yaw;

    uavStateData[index].data.uavState.land_pos[0] = msg->land_pos[0];
    uavStateData[index].data.uavState.land_pos[1] = msg->land_pos[1];
    uavStateData[index].data.uavState.land_pos[2] = msg->land_pos[2];
    uavStateData[index].data.uavState.land_yaw= msg->land_yaw;

    std::string mode = msg->mode;
    if (mode.length() > 15)
        mode = mode.substr(0, 15);
    else if (mode.length() < 15)
        mode.append(15 - mode.length(), ' ');

    // 本节点 --UDP--> 其他无人机 无人机状态信息组播链路发送
    if (uav_id > 0 && !is_simulation && uav_id == robot_id)
    {
        uavStateData[index].seq=MessageID::UAVStateMessageID;
        int back = udpSocket->sendUDPMulticastData(codec.coder(uavStateData[index]), udp_port);
    }
    std::vector<std::string> tempVec;

    // 发送数据到地面站 本节点 --UDP--> 地面站
    // 仿真情况下，所有无人机状态都通过本节点回传（仿真时只有一个通信节点，真机时有N个机载通信节点）
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    for (const auto &ip : GSIPHash)
    {
        uavStateData[index].seq=MessageID::UAVStateMessageID;
        // 无人机状态 - UAVState（#2）
        int sendBack = udpSocket->sendUDPData(codec.coder(uavStateData[index]), ip, udp_ground_port);
        if (sendBack < 0)
            tempVec.push_back(ip);
    }

    for (const auto &ip : tempVec)
        GSIPHash.erase(ip);
    tempVec.clear();
}

void communication_bridge::formation_cmd_cb(const sunray_msgs::Formation::ConstPtr &msg)
{
    // std::cout << "formation_cmd_cb:" << std::endl;
    if(is_simulation)
        return;
    DataFrame formationData;

    if (uav_experiment_num > 0)
        formationData.robot_ID = uav_id;
    else if (ugv_experiment_num > 0)
        formationData.robot_ID = ugv_id+100;

    formationData.seq=MessageID::FormationMessageID;
    formationData.data.formation.init();
    
    formationData.data.formation.cmd=msg->cmd;
    formationData.data.formation.formation_type=msg->formation_type;
    formationData.data.formation.nameSize=msg->name.size();
    strcpy(formationData.data.formation.name, msg->name.c_str());
    // 本节点 --UDP--> 其他智能体 编队切换组播链路发送
    int back = udpSocket->sendUDPMulticastData(codec.coder(formationData), udp_port);
    
}

void communication_bridge::ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int robot_id)
{
    // std::cout << "ugv_state_cb:" << robot_id<< std::endl;

    int index = robot_id - 1;
    ugvStateData[index].data.ugvState.init();
    ugvStateData[index].robot_ID = robot_id+100;
    ugvStateData[index].data.ugvState.ugv_id = robot_id;
    ugvStateData[index].data.ugvState.connected = msg->connected;
    ugvStateData[index].data.ugvState.battery_state = msg->battery_state;
    ugvStateData[index].data.ugvState.battery_percentage = msg->battery_percentage;
    ugvStateData[index].data.ugvState.location_source = msg->location_source;
    ugvStateData[index].data.ugvState.odom_valid = msg->odom_valid;
    ugvStateData[index].data.ugvState.position[0] = msg->position[0];
    ugvStateData[index].data.ugvState.position[1] = msg->position[1];
    ugvStateData[index].data.ugvState.velocity[0] = msg->velocity[0];
    ugvStateData[index].data.ugvState.velocity[1] = msg->velocity[1];
    ugvStateData[index].data.ugvState.yaw = msg->yaw;

    ugvStateData[index].data.ugvState.attitude[0] = msg->attitude[0];
    ugvStateData[index].data.ugvState.attitude[1] = msg->attitude[1];
    ugvStateData[index].data.ugvState.attitude[2] = msg->attitude[2];

    ugvStateData[index].data.ugvState.attitude_q[0] = msg->attitude_q.x;
    ugvStateData[index].data.ugvState.attitude_q[1] = msg->attitude_q.y;
    ugvStateData[index].data.ugvState.attitude_q[2] = msg->attitude_q.z;
    ugvStateData[index].data.ugvState.attitude_q[3] = msg->attitude_q.w;

    ugvStateData[index].data.ugvState.control_mode = msg->control_mode;

    ugvStateData[index].data.ugvState.pos_setpoint[0] = msg->pos_setpoint[0];
    ugvStateData[index].data.ugvState.pos_setpoint[1] = msg->pos_setpoint[1];
    ugvStateData[index].data.ugvState.vel_setpoint[0] = msg->vel_setpoint[0];
    ugvStateData[index].data.ugvState.vel_setpoint[1] = msg->vel_setpoint[1];
    ugvStateData[index].data.ugvState.yaw_setpoint = msg->yaw_setpoint;

    ugvStateData[index].data.ugvState.home_pos[0] = msg->home_pos[0];
    ugvStateData[index].data.ugvState.home_pos[1] = msg->home_pos[1];
    ugvStateData[index].data.ugvState.home_yaw = msg->home_yaw;

    ugvStateData[index].data.ugvState.hover_pos[0] = msg->hover_pos[0];
    ugvStateData[index].data.ugvState.hover_pos[1] = msg->hover_pos[1];
    ugvStateData[index].data.ugvState.hover_yaw = msg->hover_yaw;

    // 本节点 --UDP--> 其他无人车 无人车状态信息组播链路发送
    if (ugv_id > 0 && !is_simulation && ugv_id == robot_id)
    {
        ugvStateData[index].seq=MessageID::UGVStateMessageID;
        int back = udpSocket->sendUDPMulticastData(codec.coder(ugvStateData[index]), udp_port);
    }

    std::vector<std::string> tempVec;
    // 发送数据到地面站 本节点 --UDP--> 地面站
    // 仿真情况下，所有无人机状态都通过本节点回传（仿真时只有一个通信节点，真机时有N个机载通信节点）
    std::lock_guard<std::mutex> lock(_mutexTCPLinkState);

    for (const auto &ip : GSIPHash)
    {
        // 无人车状态 - UGVState（#20）
        ugvStateData[index].seq=MessageID::UGVStateMessageID;
        int sendBack = udpSocket->sendUDPData(codec.coder(ugvStateData[robot_id - 1]), ip, udp_ground_port);
        std::cout << "sendBack: " << sendBack<< std::endl;

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

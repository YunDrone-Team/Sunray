#include "communication_bridge.h"

void communication_bridge::init(ros::NodeHandle &nh)
{
    nh.param<bool>("is_simulation", is_simulation, false);      // 【参数】仿真模式or真机模式：仿真模式下，多个飞机只启动一个公用的通信节点，真机模式下，每个飞机在机载电脑中分别启动一个通信节点

    nh.param<int>("uav_experiment_num", uav_experiment_num, 0); // 【参数】无人机真机数量（用于智能体之间的通信）
    nh.param<int>("uav_simulation_num", uav_simulation_num, 0); // 【参数】仿真无人机数量（用于仿真时，一次性订阅多个无人机消息）
    nh.param<int>("uav_id", uav_id, -1);                        // 【参数】无人机编号(真机为本机，仿真为1)
    nh.param<string>("uav_name", uav_name, "uav");              // 【参数】无人机名字前缀

    nh.param<int>("ugv_experiment_num", ugv_experiment_num, 0); // 【参数】无人车真车数量
    nh.param<int>("ugv_simulation_num", ugv_simulation_num, 0); // 【参数】仿真无人车数量
    nh.param<int>("ugv_id", ugv_id, -1);                        // 【参数】无人车编号(真机为本机，仿真为1)
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

    // 仿真模式
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
            // 【发布】无人机规划点数据 地面站 --TCP--> 本节点 --ROS topic--> 
            uav_goal_pub.insert(std::make_pair(i, (nh.advertise<geometry_msgs::PoseStamped>("/goal_"+std::to_string(i), 1))));
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
            // 【发布】无人车规划点数据 地面站 --TCP--> 本节点 --ROS topic--> 
            ugv_goal_pub.insert(std::make_pair(i, (nh.advertise<geometry_msgs::PoseStamped>("/goal_"+std::to_string(i), 1))));
        }
    }else{
        // 无人机Sunray和地面站之间的通信（此部分仅针对真机）
        if (uav_experiment_num > 0 && uav_id>=0 )
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
            // 【发布】无人机规划点数据 地面站 --TCP--> 本节点 --ROS topic--> 
            uav_goal_pub.insert(std::make_pair(uav_id, (nh.advertise<geometry_msgs::PoseStamped>("/goal_"+std::to_string(uav_id), 1))));
        }

        // 无人车Sunray和地面站之间的通信（此部分仅针对真机）
        if (ugv_experiment_num > 0 && ugv_id>=0)
        {
            //  无人车名字 = 无人车名字前缀 + 无人车ID
            std::string topic_prefix = "/" + ugv_name + std::to_string(ugv_id);
            // 【订阅】无人车状态 ugv_control_node --ROS topic--> 本节点 --UDP--> 地面站/其他Sunray智能体
            ugv_state_sub.push_back(nh.subscribe<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 1, boost::bind(&communication_bridge::ugv_state_cb, this, _1, ugv_id)));
            // 【发布】无人车控制指令 地面站 --TCP--> 本节点 --ROS topic--> ugv_control_node
            ugv_controlCMD_pub.insert(std::make_pair(ugv_id, (nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 1))));
            // 【发布】无人车规划点数据 地面站 --TCP--> 本节点 --ROS topic--> 
            ugv_goal_pub.insert(std::make_pair(ugv_id, (nh.advertise<geometry_msgs::PoseStamped>("/goal_"+std::to_string(ugv_id), 1))));
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
    prevData = readCpuData();
    // 【定时器】 定时发送智能体电脑状态到地面站
    UpdateCPUUsageRateTimer= nh.createTimer(ros::Duration(1), &communication_bridge::UpdateComputerStatus, this);

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
    // 【UDP通信】 UDP通信：接收UDP消息的回调函数 - 包括：地面站搜索在线智能体、机间通信相关的消息
    udpSocket->sigUDPUnicastReadData.connect(boost::bind(&communication_bridge::UDPCallBack, this, _1));
    // 【定时器】 定时更新UDP组播
    UpdateUDPMulticastTimer= nh.createTimer(ros::Duration(30), &communication_bridge::UpdateUDPMulticast, this);
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

uint64_t communication_bridge::calculateMessageDelay(uint64_t msgTimestamp) 
{
    uint64_t currentTimestamp = getCurrentTimestampMs();
    return (currentTimestamp >= msgTimestamp) ? (currentTimestamp - msgTimestamp) : 0;
}

uint64_t communication_bridge::getCurrentTimestampMs() 
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
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
    int send_result;
    //  std::cout << " GroundControl::UDPCallBack: " << (int)readData.dataFrame.seq << std::endl;
    // std::cout << "UDP Message Delay:" << calculateMessageDelay(readData.dataFrame.timestamp)<< std::endl;

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
            send_result = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
            //  std::cout << "发送结果: " << send_result << std::endl;
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
            send_result = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
        }
        // 真机无人机应答
        if (uav_experiment_num > 0 && uav_id>=0)
        {
            backData.robot_ID = uav_id;
            backData.data.ack.ID = uav_id;
            backData.data.ack.agentType = 0;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            std::lock_guard<std::mutex> lock(_mutexUDP);
            send_result = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
        }
        // 真机无人车应答
        if (ugv_experiment_num > 0 && ugv_id>=0)
        {
            backData.robot_ID = ugv_id+100;
            backData.data.ack.ID = ugv_id;
            backData.data.ack.agentType = 1;
            backData.data.ack.port = static_cast<unsigned short>(std::stoi(tcp_port));
            std::lock_guard<std::mutex> lock(_mutexUDP);
            send_result = udpSocket->sendUDPData(codec.coder(backData), readData.ip, (uint16_t)readData.dataFrame.data.search.port);
        }

        break;
    }
    case MessageID::UAVStateMessageID: // 无人机状态 - UAVState (#2)
    {
        // std::cout << " case MessageID::StateMessageID: " << (int)readData.data.state.uavID << std::endl;
        // 如果是仿真模式，直接返回；如果是真机模式，则判断ID是否匹配
        if (uav_id == readData.dataFrame.robot_ID || is_simulation)
            return;
        SynchronizationUAVState(readData.dataFrame.data.uavState);
        break;
    }
    case MessageID::UGVStateMessageID:// 无人车状态 - UGVState（#20）
    {
        if (ugv_id == readData.dataFrame.robot_ID - 100 || is_simulation)
            return;
        SynchronizationUGVState(readData.dataFrame.data.ugvState);
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
    //  std::cout << "communication_bridge::TCPServerCallBack:" << (int)readData.dataFrame.seq<< std::endl;
    // std::cout << "TCP Message Delay:" << calculateMessageDelay(readData.dataFrame.timestamp)<< std::endl;
    switch (readData.dataFrame.seq)
    {
    case MessageID::UAVControlCMDMessageID:// 无人机控制指令 - UAVControlCMD（#102）
    {
        auto it = control_cmd_pub.find(robot_id);
        if (it == control_cmd_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        // ROS话题赋值
        sunray_msgs::UAVControlCMD uav_cmd;
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

        // 根据robot_id查找对应的话题进行发布
        it->second.publish(uav_cmd);
        break;
    }
    case MessageID::UAVSetupMessageID:// 无人机设置指令 - UAVSetup（#103）
    {
        auto it = uav_setup_pub.find(robot_id);
        if (it == uav_setup_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
        sunray_msgs::UAVSetup setup;
        setup.header.stamp = ros::Time::now();
        if (readData.dataFrame.data.uavSetup.cmd == UAVSetupType::SetControlMode)
        {
            if (readData.dataFrame.data.uavSetup.control_mode == UAVSetupType::SetControlMode)
                setup.control_mode = "CMD_CONTROL";
            else if (readData.dataFrame.data.uavSetup.control_mode == UAVSetupType::LandSetupType)
                setup.control_mode = "LAND_CONTROL";
        }
        setup.cmd = readData.dataFrame.data.uavSetup.cmd;

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
        auto it = uav_waypoint_pub.find(robot_id);
        if (it == uav_waypoint_pub.end())
        {
            std::cout << "controlCMD UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
            break;
        }
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
        msg.header.stamp = ros::Time::now();
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
    case MessageID::GroundFormationMessageID:// 编队切换 - Formation（#40）
    {    
        sunray_msgs::Formation sendMSG;
        sendMSG.cmd=readData.dataFrame.data.formation.cmd;
        sendMSG.formation_type=readData.dataFrame.data.formation.formation_type;
        sendMSG.leader_id= readData.dataFrame.data.formation.leader_id;
        sendMSG.name=readData.dataFrame.data.formation.name;
        formation_pub.publish(sendMSG);
        break;   
    }  
    case MessageID::GoalMessageID:// 规划点- Goal（#204）
    {    
        if(readData.dataFrame.robot_ID<=100)
        {
            auto it = uav_goal_pub.find(robot_id);
            if (it == uav_goal_pub.end())
            {
                std::cout << "Goal UAV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
                break;
            }      
            geometry_msgs::PoseStamped sendMSG;
            sendMSG.header.stamp = ros::Time::now();
            sendMSG.header.frame_id = "world";
            sendMSG.pose.orientation.x = 0;
            sendMSG.pose.orientation.y = 0;
            sendMSG.pose.orientation.z = 0;
            sendMSG.pose.orientation.w = 1;
            sendMSG.pose.position.x = readData.dataFrame.data.goal.positionX;
            sendMSG.pose.position.y = readData.dataFrame.data.goal.positionY;
            sendMSG.pose.position.z = readData.dataFrame.data.goal.positionZ;
            it->second.publish(sendMSG);

        }else if(readData.dataFrame.robot_ID>100){
            auto it = ugv_goal_pub.find(robot_id);
            if (it == ugv_goal_pub.end())
            {
                std::cout << "Goal UGV" + std::to_string(robot_id) + " topic Publisher not found!" << std::endl;
                break;
            }      
            geometry_msgs::PoseStamped sendMSG;
            sendMSG.header.stamp = ros::Time::now();
            sendMSG.header.frame_id = "world";
            sendMSG.pose.orientation.x = 0;
            sendMSG.pose.orientation.y = 0;
            sendMSG.pose.orientation.z = 0;
            sendMSG.pose.orientation.w = 1;
            sendMSG.pose.position.x = readData.dataFrame.data.goal.positionX;
            sendMSG.pose.position.y = readData.dataFrame.data.goal.positionY;
            sendMSG.pose.position.z = readData.dataFrame.data.goal.positionZ;
            it->second.publish(sendMSG);
        }
        
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
    msg.header.stamp = ros::Time::now();
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
    msg.header.stamp = ros::Time::now();
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

void communication_bridge::UpdateUDPMulticast(const ros::TimerEvent &e)
{
    std::lock_guard<std::mutex> lock(_mutexUDP);
    if(udpSocket!=nullptr)
        udpSocket->UpdateMulticast();
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
        if (uav_experiment_num > 0 && uav_id>=0)
        {
            Heartbeatdata.robot_ID = uav_id;
            Heartbeatdata.data.heartbeat.agentType = UAVType;
            tcpServer.allSendData(codec.coder( Heartbeatdata));
        }
        // 无人车心跳包 - 真机发送
        if (ugv_experiment_num > 0&& ugv_id>=0)
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
    // std::cout << "sendUDPMulticastData uav_state_cb:" << std::endl;

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

    uavStateData[index].seq=MessageID::UAVStateMessageID;
    SendUdpDataToAllOnlineGroundStations(uavStateData[index]);
}

void communication_bridge::formation_cmd_cb(const sunray_msgs::Formation::ConstPtr &msg)
{
    // std::cout << "formation_cmd_cb:" << std::endl;
    if(is_simulation)
        return;
    DataFrame formationData;

    if (uav_experiment_num > 0 && uav_id>=0 )
        formationData.robot_ID = uav_id;
    else if (ugv_experiment_num > 0 && ugv_id>=0)
        formationData.robot_ID = ugv_id+100;

    formationData.seq=MessageID::FormationMessageID;
    formationData.data.formation.init();
    
    formationData.data.formation.cmd=msg->cmd;
    formationData.data.formation.formation_type=msg->formation_type;
    formationData.data.formation.leader_id=msg->leader_id;
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
        // std::cout << "sendUDPMulticastData back:" << back<< std::endl;

    }
    ugvStateData[index].seq=MessageID::UGVStateMessageID;
    SendUdpDataToAllOnlineGroundStations(ugvStateData[index]);
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

// Read CPU time statistics from /proc/stat
CpuData communication_bridge::readCpuData() {
    std::ifstream statFile("/proc/stat");
    std::string line;
    
    // Read the first line which contains total CPU time
    std::getline(statFile, line);
    
    std::istringstream iss(line);
    std::string cpuLabel;
    CpuData data;
    
    // Parse CPU time data from the line
    iss >> cpuLabel >> data.user >> data.nice >> data.system >> data.idle 
        >> data.iowait >> data.irq >> data.softirq >> data.steal 
        >> data.guest >> data.guest_nice;
    
    return data;
}

// Calculate CPU usage percentage based on previous and current CPU data
double communication_bridge::calculateCpuUsage(const CpuData& prev, const CpuData& curr)
 {
    // Calculate total CPU time difference
    unsigned long long prevTotal = prev.user + prev.nice + prev.system + prev.idle + 
                                  prev.iowait + prev.irq + prev.softirq + prev.steal;
    unsigned long long currTotal = curr.user + curr.nice + curr.system + curr.idle + 
                                  curr.iowait + curr.irq + curr.softirq + curr.steal;
    unsigned long long totalDiff = currTotal - prevTotal;
    
    // Calculate idle time difference
    unsigned long long idleDiff = curr.idle - prev.idle;
    
    // Calculate CPU usage percentage: (1 - idle_time/total_time) * 100%
    if (totalDiff == 0) return 0.0; // Avoid division by zero
    return (1.0 - static_cast<double>(idleDiff) / static_cast<double>(totalDiff)) * 100.0;
}

void communication_bridge::UpdateComputerStatus(const ros::TimerEvent &e)
{
    //计算CPU使用率
    CpuData currData = readCpuData();
    double cpuUsage = calculateCpuUsage(prevData, currData);
    prevData = currData;
    // std::cout << "CPU 占用率: " << cpuUsage <<"%" << std::endl;

    //获取内存占用率
    double memUsage = getMemoryUsage();
    // std::cout << "内存占用率:" << memUsage <<"%" << std::endl;

    //获取CPU温度
    auto cpu_temps = getCpuTemperatures();
    // std::cout << "CPU温度: ";
    // 计算平均值
    double avgTemp = 0.0;
    for (size_t i = 0; i < cpu_temps.size(); ++i) 
    {
        // std::cout << "核心" << i << ": " << cpu_temps[i] << "°C ";
        avgTemp += cpu_temps[i];
    }
    avgTemp /= cpu_temps.size();
    // std::cout << "(平均: " << avgTemp << "°C)" << std::endl;

    //发送UDP单播数据到在线地面站
    DataFrame sendDataFrame;
    sendDataFrame.data.computerStatus.init();
    sendDataFrame.seq=MessageID::AgentComputerStatusMessageID;
    sendDataFrame.data.computerStatus.cpuLoad =cpuUsage;
    sendDataFrame.data.computerStatus.memoryUsage=memUsage;
    sendDataFrame.data.computerStatus.cpuTemperature =avgTemp;

    if (is_simulation)
    {
        for (int i = uav_id; i < uav_id + uav_simulation_num; i++)
        {
            sendDataFrame.robot_ID=i;
            SendUdpDataToAllOnlineGroundStations(sendDataFrame);
        }

        for (int i = ugv_id; i < ugv_id + ugv_simulation_num; i++)
        {
            sendDataFrame.robot_ID=i+100;
            SendUdpDataToAllOnlineGroundStations(sendDataFrame);
        }
    }else{
        if (uav_experiment_num > 0 && uav_id>=0 )
        {
            sendDataFrame.robot_ID=uav_id;
            SendUdpDataToAllOnlineGroundStations(sendDataFrame);
        }

        if (ugv_experiment_num > 0 && ugv_id>=0)
        {
            sendDataFrame.robot_ID=ugv_id+100;
            SendUdpDataToAllOnlineGroundStations(sendDataFrame);
        }
    }

        
}

// 获取系统内存占用率（百分比）
double communication_bridge::getMemoryUsage() 
{
    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open()) 
    {
        std::cout <<"无法打开/proc/meminfo文件"<< std::endl;
        return -1.0;
    }

    long total_mem = 0, available_mem = 0;
    std::string line;

    while (std::getline(meminfo, line)) 
    {
        if (line.find("MemTotal:") != std::string::npos) 
        {
            std::istringstream iss(line);
            std::string key, unit;
            iss >> key >> total_mem >> unit;
            if (unit != "kB") 
                std::cout <<"内存单位不是kB，可能导致计算错误"<< std::endl;
            
        } else if (line.find("MemAvailable:") != std::string::npos) {
            std::istringstream iss(line);
            std::string key, unit;
            iss >> key >> available_mem >> unit;
            if (unit != "kB") 
                std::cout <<"内存单位不是kB，可能导致计算错误"<< std::endl;
            
            break;
        }
    }

    meminfo.close();

    // 确保数据有效性
    if (total_mem <= 0 || available_mem < 0) 
    {
        std::cout << "无效的内存数据: MemTotal="<<total_mem<<", MemAvailable="<<available_mem<< std::endl;
        return -1.0;
    }

    // 计算实际已用内存百分比（使用MemAvailable更准确）
    double used_percent = (1.0 - (double)available_mem / total_mem) * 100.0;
    
    // std::cout << "内存统计: 总内存="<<total_mem / 1024.0 / 1024.0<<"GB, 可用内存="<<available_mem / 1024.0 / 1024.0
    // <<" GB, 使用率="<<used_percent<< std::endl;
    
    return used_percent;
}

// 获取CPU温度（摄氏度），过滤非CPU传感器数据
std::vector<double> communication_bridge::getCpuTemperatures()
{
    std::vector<double> cpu_temps;
    
    // 打印调试信息
    // std::cout << "[DEBUG] 开始获取CPU温度..." << std::endl;
    
    // 尝试方法1: /sys/class/hwmon路径（优先）
    // std::cout << "[DEBUG] 尝试方法1: /sys/class/hwmon路径..." << std::endl;
    std::vector<std::string> cpu_sensors = {"coretemp", "k10temp", "amdgpu"};
    std::vector<std::string> cpu_labels = {"core", "cpu", "package"};
    
    for (const auto& sensor : cpu_sensors) 
    {
        for (int i = 0; i < 10; ++i)
         {
            std::string hwmon_path = "/sys/class/hwmon/hwmon" + std::to_string(i);
            std::ifstream name_file(hwmon_path + "/name");
            
            std::string device_name;
            if (name_file >> device_name && device_name == sensor) 
            {
                // std::cout << "[DEBUG] 找到传感器: " << device_name << " (hwmon" << i << ")" << std::endl;
                
                for (int j = 1; j < 10; ++j) 
                {
                    std::string temp_input = hwmon_path + "/temp" + std::to_string(j) + "_input";
                    std::string temp_label = hwmon_path + "/temp" + std::to_string(j) + "_label";
                    
                    std::ifstream temp_file(temp_input);
                    if (temp_file.good()) 
                    {
                        long temp_millis;
                        if (temp_file >> temp_millis) 
                        {
                            std::string label = "temp" + std::to_string(j);
                            std::ifstream label_file(temp_label);
                            
                            bool has_label = false;
                            if (label_file >> label) 
                            {
                                std::transform(label.begin(), label.end(), label.begin(), ::tolower);
                                has_label = true;
                            }
                            
                            // 判断是否为CPU温度
                            bool is_cpu = false;
                            if (has_label) 
                            {
                                for (const auto& cpu_label : cpu_labels) 
                                {
                                    if (label.find(cpu_label) != std::string::npos) 
                                    {
                                        is_cpu = true;
                                        break;
                                    }
                                }
                            } else {
                                // 无标签时，根据传感器类型判断
                                is_cpu = true;
                            }
                            
                            if (is_cpu)
                            {
                                cpu_temps.push_back(temp_millis / 1000.0);
                                // std::cout << "[DEBUG] 获取到温度: " 
                                //           << (has_label ? label : "unknown") 
                                //           << " = " << temp_millis / 1000.0 << "°C" << std::endl;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    
    if (!cpu_temps.empty()) 
    {
        // std::cout << "[INFO] 方法1成功获取到" << cpu_temps.size() << "个CPU温度值" << std::endl;
        return cpu_temps;
    }
    
    // 尝试方法2: /sys/class/thermal路径
    // std::cout << "[DEBUG] 尝试方法2: /sys/class/thermal路径..." << std::endl;
    for (int i = 0; i < 10; ++i) 
    {
        std::string type_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/type";
        std::string temp_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
        
        std::ifstream type_file(type_path);
        std::string zone_type;
        if (type_file >> zone_type)
         {
            // 检查是否为CPU相关区域
            std::transform(zone_type.begin(), zone_type.end(), zone_type.begin(), ::tolower);
            bool is_cpu_zone = (zone_type.find("cpu") != std::string::npos || 
                               zone_type.find("x86_pkg_temp") != std::string::npos);
            
            if (is_cpu_zone)
            {
                std::ifstream temp_file(temp_path);
                if (temp_file.good()) 
                {
                    long temp_value;
                    if (temp_file >> temp_value) 
                    {
                        // 通常是毫摄氏度
                        double temperature = temp_value / 1000.0;
                        cpu_temps.push_back(temperature);
                        // std::cout << "[DEBUG] 获取到温度: thermal_zone" << i 
                        //           << " (" << zone_type << ") = " << temperature << "°C" << std::endl;
                    }
                }
            }
        }
    }
    
    if (!cpu_temps.empty())
     {
        // std::cout << "[INFO] 方法2成功获取到" << cpu_temps.size() << "个CPU温度值" << std::endl;
        return cpu_temps;
    }
    
    // 尝试方法3: /proc/acpi/thermal_zone路径（适用于旧系统）
    // std::cout << "[DEBUG] 尝试方法3: /proc/acpi/thermal_zone路径..." << std::endl;
    for (int i = 0; i < 10; ++i) 
    {
        std::string zone_path = "/proc/acpi/thermal_zone/THM" + std::to_string(i) + "/temperature";
        std::ifstream temp_file(zone_path);
        
        if (temp_file.good()) 
        {
            std::string line;
            while (std::getline(temp_file, line)) 
            {
                if (line.find("temperature:") != std::string::npos) 
                {
                    size_t pos = line.find(":");
                    if (pos != std::string::npos)
                     {
                        std::string temp_part = line.substr(pos + 1);
                        // 解析温度值（格式可能为" temperature: 45 C"）
                        size_t value_start = temp_part.find_first_not_of(" \t");
                        if (value_start != std::string::npos) 
                        {
                            std::istringstream iss(temp_part.substr(value_start));
                            double temp;
                            std::string unit;
                            if (iss >> temp >> unit && unit == "C") 
                            {
                                cpu_temps.push_back(temp);
                                // std::cout << "[DEBUG] 获取到温度: THM" << i << " = " << temp << "°C" << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (!cpu_temps.empty()) 
    {
        // std::cout << "[INFO] 方法3成功获取到" << cpu_temps.size() << "个CPU温度值" << std::endl;
        return cpu_temps;
    }
    
    
    // 所有方法均失败
    // std::cerr << "[ERROR] 所有方法均无法获取CPU温度数据！" << std::endl;
    // std::cerr << "[ERROR] 请检查: 1) 传感器驱动是否加载 2) 程序是否有读取权限 3) 硬件是否支持温度检测" << std::endl;
    
    double temp=-300;
    cpu_temps.push_back(temp);
    return cpu_temps;
                             
                 
}

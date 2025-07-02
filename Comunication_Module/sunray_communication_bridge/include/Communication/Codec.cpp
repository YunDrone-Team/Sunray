#include "Codec.h"

Codec::Codec()
{


}

uint64_t Codec::getTimestamp()
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 将当前时间点的时间间隔转换为自纪元以来的毫秒数
    auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // 将毫秒数转换为uint64_t类型
    uint64_t timestamp = static_cast<uint64_t>(milliseconds_since_epoch);
    return timestamp;
}


void Codec::floatArrayCopyToUint8tArray(std::vector<uint8_t>& data,std::vector<float>& value)
{
    // 逐个 float 处理，将其字节存储到 uint8_tS 中
    for (size_t i = 0; i < value.size(); ++i)
    {
        uint8_t floatBytes[sizeof(float)];
        // 使用 std::memcpy 安全地将 float 的字节复制到 uint8_t 数组中
        std::memcpy(floatBytes, &value[i], sizeof(float));
        // 将每个字节添加到 data 中
        for (int j = 0; j < (int)sizeof(float); ++j)
            data.push_back(floatBytes[j]);
    }
}


void Codec::doubleCopyToUint8tArray(std::vector<uint8_t>& data,double& value)
{
    // 将 double 类型数据的地址转换为 uint8_t 指针
    uint8_t* bytePtr = reinterpret_cast<uint8_t*>(&value);
    // 由于 double 通常是 8 个字节，遍历 8 次
    for (size_t i = 0; i < sizeof(double); ++i)
        data.push_back(bytePtr[i]);

}

void Codec::floatCopyToUint8tArray(std::vector<uint8_t>& data,float& value)
{
    uint8_t floatBytes[sizeof(float)];
    // 使用 std::memcpy 安全地将 float 的字节复制到 uint8_t 数组中
    std::memcpy(floatBytes, &value, sizeof(float));
    // 将每个字节添加到 data 中
    for (int j = 0; j < (int)sizeof(float); ++j)
        data.push_back(floatBytes[j]);
}

void Codec::uint8tArrayToDouble(std::vector<uint8_t>& data, double& value)
{
    if (data.size() < sizeof(double))
    {
        std::cerr << "Error: The vector size is less than sizeof(double)" << std::endl;
        return;
    }
    // 将 vector 中的字节复制到 double 的内存空间
    std::memcpy(&value, data.data(), sizeof(double));
    // 移除已经处理过的字节
    data.erase(data.begin(), data.begin() + sizeof(double));
}



void Codec::uint8tArrayToFloat(std::vector<uint8_t>& data, float& value)
{

    // 创建一个临时的 uint8_t 数组来存储从 vector 中提取的字节
    uint8_t floatBytes[sizeof(float)];

    // 从 data 中复制字节到 floatBytes
    std::memcpy(floatBytes, data.data(), sizeof(float));

    // 使用 std::memcpy 将字节数组转换回 float 并赋值给 value
    std::memcpy(&value, floatBytes, sizeof(float));

    // 移除已经处理过的字节
    data.erase(data.begin(), data.begin() + sizeof(float));
}


void Codec::safeConvertToUint32(size_t originalSize, uint32_t& convertedSize)
{
    if (originalSize > std::numeric_limits<uint32_t>::max())
    {
        // 可以设置一个错误值，或者采取其他错误处理措施
        convertedSize = 0; // 示例：设置为0表示错误
    } else
        convertedSize = static_cast<uint32_t>(originalSize+18);

}

uint16_t Codec::getChecksum(std::vector<uint8_t> data)
{
    unsigned int sum = 0; // 累加器，使用unsigned int以防溢出

    // 使用索引for循环遍历vector并累加每个uint8_t值
    for (size_t i = 0; i < data.size(); ++i)
        sum += static_cast<uint8_t>(data[i]); // 转换为unsigned uint8_t以避免符号扩展，然后累加到sum中

    // 提取累加结果中的最后两个字节
    // 注意：这里我们假设sum不会超出16位的表示范围，否则需要更复杂的处理
    uint16_t lastTwoBytes = static_cast<uint16_t>(sum & 0xFFFF); // 实际上，0xFFFF对于uint16_t是多余的，但为清晰起见保留
    return lastTwoBytes;
}

void Codec::decoderGoalPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct)
{
    Goal& data = dataFrameStruct.data.goal;
    data.init();

    uint8tArrayToDouble(dataFrame, data.positionX);
    uint8tArrayToDouble(dataFrame, data.positionY);
    uint8tArrayToDouble(dataFrame, data.positionZ);

}

void Codec::decoderAgentComputerStatusPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct)
{
    AgentComputerStatus& data = dataFrameStruct.data.computerStatus;
    data.init();

    uint8tArrayToDouble(dataFrame, data.cpuLoad);
    uint8tArrayToDouble(dataFrame, data.memoryUsage);
    uint8tArrayToDouble(dataFrame, data.cpuTemperature);
}

 void Codec::decoderFormationPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct)
 {
     Formation& data = dataFrameStruct.data.formation;
     data.init();

     data.cmd= static_cast<uint8_t>(dataFrame[0]);
     data.formation_type= static_cast<uint8_t>(dataFrame[1]);
     data.leader_id= static_cast<uint8_t>(dataFrame[2]);

     dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 3);

     // 读取nameSize（2字节，小端序）
     data.nameSize = static_cast<uint16_t>(dataFrame[0]);
     data.nameSize |= (static_cast<uint16_t>(dataFrame[1]) << 8);
     dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

     // 读取name
     for (uint16_t j = 0; j < data.nameSize; ++j)
         data.name[j] = static_cast<char>(dataFrame[j]);
     data.name[data.nameSize] = '\0';  //添加终止符

     dataFrame.erase(dataFrame.begin(), dataFrame.begin() + data.nameSize);
 }


void Codec::decoderNodePayload(std::vector<uint8_t>& dataFrame,DataFrame& node)
{
    NodeData& data = node.data.nodeInformation;
    data.init();

    // 读取nodeCount（2字节，小端序）
    data.nodeCount = static_cast<uint16_t>(dataFrame[0]);
    data.nodeCount |= (static_cast<uint16_t>(dataFrame[1]) << 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取nodeID（2字节，小端序）
    data.nodeID = static_cast<uint16_t>(dataFrame[0]);
    data.nodeID |= (static_cast<uint16_t>(dataFrame[1]) << 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取nodeSize（2字节，小端序）
    data.nodeSize = static_cast<uint16_t>(dataFrame[0]);
    data.nodeSize |= (static_cast<uint16_t>(dataFrame[1]) << 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取nodeStr
    for (uint16_t j = 0; j < data.nodeSize; ++j)
        data.nodeStr[j] = static_cast<char>(dataFrame[j]);
    data.nodeStr[data.nodeSize] = '\0';  //添加终止符
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + data.nodeSize);
}

void Codec::decoderWaypointPayload(std::vector<uint8_t>& dataFrame,DataFrame& waypointData)
{
    WaypointData& data = waypointData.data.waypointData;
    data.init();

    // 读取基础字段
    data.wp_num = static_cast<uint8_t>(dataFrame[0]);
    data.wp_type = static_cast<uint8_t>(dataFrame[1]);
    data.wp_end_type = static_cast<uint8_t>(dataFrame[2]);
    data.wp_takeoff = static_cast<bool>(dataFrame[3]);
    data.wp_yaw_type = static_cast<uint8_t>(dataFrame[4]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 5);

    // 读取浮点数参数
    uint8tArrayToFloat(dataFrame, data.wp_move_vel);
    uint8tArrayToFloat(dataFrame, data.wp_vel_p);
    uint8tArrayToFloat(dataFrame, data.z_height);

    // 读取10个航点数据，每个航点包含4个double
    double* wpPoints[] = {
        data.wp_point_1, data.wp_point_2, data.wp_point_3, data.wp_point_4,
        data.wp_point_5, data.wp_point_6, data.wp_point_7, data.wp_point_8,
        data.wp_point_9, data.wp_point_10
    };

    for (int wpIdx = 0; wpIdx < 10; ++wpIdx)
    {
        for (int i = 0; i < 4; ++i)
            uint8tArrayToDouble(dataFrame, wpPoints[wpIdx][i]);
    }

    // 读取圆点坐标
    uint8tArrayToDouble(dataFrame, data.wp_circle_point[0]);
    uint8tArrayToDouble(dataFrame, data.wp_circle_point[1]);

}

void Codec::decoderScriptPayload(std::vector<uint8_t>& dataFrame,DataFrame& script)
{
    ScriptData& data = script.data.agentScrip;
    data.init();

    // 读取scripType
    data.scripType = static_cast<uint8_t>(dataFrame[0]);

    // 读取scriptState
    data.scriptState = static_cast<uint8_t>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);


    // 读取scriptSize（2字节，小端序）
    data.scriptSize = static_cast<uint16_t>(dataFrame[0]);
    data.scriptSize |= (static_cast<uint16_t>(dataFrame[1]) << 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取scriptStr
    for (uint16_t j = 0; j < data.scriptSize; ++j)
        data.scriptStr[j] = static_cast<char>(dataFrame[j]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + data.scriptSize);
}

void Codec::decoderDemoPayload(std::vector<uint8_t>& dataFrame,DataFrame& demo)
{
    DemoData& data = demo.data.demo;
    data.init();
    // 读取demoState
    data.demoState = static_cast<uint8_t>(dataFrame[0]);
    dataFrame.erase(dataFrame.begin());

    // 读取demoSize（2字节，小端序）
    data.demoSize = static_cast<uint16_t>(dataFrame[0]);
    data.demoSize |= (static_cast<uint16_t>(dataFrame[1]) << 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取demoStr
    for (uint16_t j = 0; j < data.demoSize; ++j)
        data.demoStr[j] = static_cast<char>(dataFrame[j]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + data.demoSize);

}

void Codec::decoderACKPayload(std::vector<uint8_t>& dataFrame,DataFrame& ack)
{
    ACKData& data = ack.data.ack;
    data.init();
    // 读取agentType
    data.agentType = static_cast<uint8_t>(dataFrame[0]);
    // 读取ID
    data.ID = static_cast<uint8_t>(dataFrame[1]);
    // 读取port（2字节，小端序）
    data.port = static_cast<uint16_t>(dataFrame[2]);
    data.port |= (static_cast<uint16_t>(dataFrame[3]) << 8);

    // 一次性移除已处理的4个字节
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 4);
}


void Codec::decoderUAVSetupPayload(std::vector<uint8_t>& dataFrame,DataFrame& setup)
{
    UAVSetup& data = setup.data.uavSetup;
    data.init();

    // 读取命令类型
    data.cmd = static_cast<uint8_t>(dataFrame[0]);
    // 读取PX4模式
    data.px4_mode = static_cast<uint8_t>(dataFrame[1]);
    // 读取控制模式
    data.control_mode = static_cast<uint8_t>(dataFrame[2]);

    // 一次性移除已处理的3个字节
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 3);
}

void Codec::decoderUGVControlPayload(std::vector<uint8_t>& dataFrame,DataFrame& control)
{
    UGVControlCMD& data = control.data.ugvControlCMD;
    data.init(); // 初始化结构体

    // --------------------- 读取控制命令类型和偏航类型 ---------------------
    data.cmd = static_cast<uint8_t>(dataFrame[0]);
    data.yaw_type = static_cast<uint8_t>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2); // 移除前2字节

    // --------------------- 读取期望位置 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.desired_pos[i]);

    // --------------------- 读取期望速度 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.desired_vel[i]);

    // --------------------- 读取期望偏航角和角速度 ---------------------
    uint8tArrayToFloat(dataFrame, data.desired_yaw);
    uint8tArrayToFloat(dataFrame, data.angular_vel);
}

 void Codec::decoderUAVControlPayload(std::vector<uint8_t>& dataFrame,DataFrame& control)
 {
     UAVControlCMD& data = control.data.uavControlCMD;
     data.init(); // 初始化结构体

     // --------------------- 读取控制命令类型 ---------------------
     data.cmd = static_cast<uint8_t>(dataFrame[0]);
     dataFrame.erase(dataFrame.begin()); // 移除1字节

     // --------------------- 读取期望位置 ---------------------
     for (int i = 0; i < 3; ++i)
         uint8tArrayToFloat(dataFrame, data.desired_pos[i]);

     // --------------------- 读取期望速度 ---------------------
     for (int i = 0; i < 3; ++i)
         uint8tArrayToFloat(dataFrame, data.desired_vel[i]);

     // --------------------- 读取期望加速度 ---------------------
     for (int i = 0; i < 3; ++i)
         uint8tArrayToFloat(dataFrame, data.desired_acc[i]);

     // --------------------- 读取期望 jerk ---------------------
     for (int i = 0; i < 3; ++i)
         uint8tArrayToFloat(dataFrame, data.desired_jerk[i]);

     // --------------------- 读取期望姿态 ---------------------
     for (int i = 0; i < 3; ++i)
         uint8tArrayToFloat(dataFrame, data.desired_att[i]);

     // --------------------- 读取期望推力、偏航角、偏航角速率 ---------------------
     uint8tArrayToFloat(dataFrame, data.desired_thrust);
     uint8tArrayToFloat(dataFrame, data.desired_yaw);
     uint8tArrayToFloat(dataFrame, data.desired_yaw_rate);

     // --------------------- 读取经纬度和海拔 ---------------------
     uint8tArrayToFloat(dataFrame, data.latitude);
     uint8tArrayToFloat(dataFrame, data.longitude);
     uint8tArrayToFloat(dataFrame, data.altitude);
 }

void Codec::decoderUGVStatePayload(std::vector<uint8_t>& dataFrame,DataFrame& state)
{
    UGVState& data = state.data.ugvState;
    data.init();

    // --------------------- 读取基础字段 ---------------------
    data.ugv_id = static_cast<uint8_t>(dataFrame[0]);
    data.connected = static_cast<bool>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // --------------------- 读取电池状态 ---------------------
    uint8tArrayToFloat(dataFrame, data.battery_state);
    uint8tArrayToFloat(dataFrame, data.battery_percentage);

    // --------------------- 读取定位状态 ---------------------
    data.location_source = static_cast<uint8_t>(dataFrame[0]);
    data.odom_valid = static_cast<bool>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);  // 移除2字节

    // --------------------- 读取位置数据 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.position[i]);

    // --------------------- 读取速度数据 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.velocity[i]);

    // --------------------- 读取偏航角 ---------------------
    uint8tArrayToFloat(dataFrame, data.yaw);

    // --------------------- 读取姿态角 ---------------------
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.attitude[i]);

    // --------------------- 读取四元数 ---------------------
    for (int i = 0; i < 4; ++i)
        uint8tArrayToFloat(dataFrame, data.attitude_q[i]);

    // --------------------- 读取控制模式 ---------------------
    data.control_mode = static_cast<uint8_t>(dataFrame[0]);
    dataFrame.erase(dataFrame.begin());  // 移除1字节

    // --------------------- 读取位置设定点 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.pos_setpoint[i]);

    // --------------------- 读取速度设定点 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.vel_setpoint[i]);

    // --------------------- 读取偏航设定点 ---------------------
    uint8tArrayToFloat(dataFrame, data.yaw_setpoint);

    // --------------------- 读取home位置 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.home_pos[i]);

    // --------------------- 读取home偏航角 ---------------------
    uint8tArrayToFloat(dataFrame, data.home_yaw);

    // --------------------- 读取hover位置 ---------------------
    for (int i = 0; i < 2; ++i)
        uint8tArrayToFloat(dataFrame, data.hover_pos[i]);

    // --------------------- 读取hover偏航角 ---------------------
    uint8tArrayToFloat(dataFrame, data.hover_yaw);

}

void Codec::decoderUAVStatePayload(std::vector<uint8_t>& dataFrame,DataFrame& state)
{
    UAVState& data = state.data.uavState;
    data.init();

    // 读取基础状态
    data.uav_id = static_cast<uint8_t>(dataFrame[0]);
    data.connected = static_cast<bool>(dataFrame[1]);
    data.armed = static_cast<bool>(dataFrame[2]);
    data.mode = static_cast<uint8_t>(dataFrame[3]);
    data.landed_state = static_cast<uint8_t>(dataFrame[4]);

    // 一次性移除已处理的5个字节
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 5);

    // 读取电池状态
    uint8tArrayToFloat(dataFrame, data.battery_state);
    uint8tArrayToFloat(dataFrame, data.battery_percentage);

    // 读取位置源和里程计状态
    data.location_source = static_cast<uint8_t>(dataFrame[0]);
    data.odom_valid = static_cast<bool>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取位置数据
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.position[i]);

    // 读取速度数据
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.velocity[i]);

    // 读取姿态角数据
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.attitude[i]);

    // 读取四元数数据
    for (int i = 0; i < 4; ++i)
        uint8tArrayToFloat(dataFrame, data.attitude_q[i]);

    // 读取姿态率数据
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.attitude_rate[i]);

    // 读取位置设定点
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.pos_setpoint[i]);

    // 读取速度设定点
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.vel_setpoint[i]);

    // 读取姿态设定点
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.att_setpoint[i]);

    // 读取推力设定点
    uint8tArrayToFloat(dataFrame, data.thrust_setpoint);

    // 读取控制模式和移动模式
    data.control_mode = static_cast<uint8_t>(dataFrame[0]);
    data.move_mode = static_cast<uint8_t>(dataFrame[1]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + 2);

    // 读取起飞高度
    uint8tArrayToFloat(dataFrame, data.takeoff_height);

    // 读取home位置和偏航角
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.home_pos[i]);
    uint8tArrayToFloat(dataFrame, data.home_yaw);

    // 读取悬停位置和偏航角
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.hover_pos[i]);
    uint8tArrayToFloat(dataFrame, data.hover_yaw);

    // 读取降落位置和偏航角
    for (int i = 0; i < 3; ++i)
        uint8tArrayToFloat(dataFrame, data.land_pos[i]);
    uint8tArrayToFloat(dataFrame, data.land_yaw);
}


bool Codec::decoder(std::vector<uint8_t> undecodedData,DataFrame& decoderData)
{
    if(undecodedData.size()<18)
        return false;

//    decoderData.head = undecodedData.at(0);                  // 低位字节
//    decoderData.head |= (undecodedData.at(1) << 8);          // 高位字节
    decoderData.head=0;
    // 解析帧头（2字节，大端序）
    decoderData.head = undecodedData[1] | (static_cast<uint16_t>(undecodedData[0]) << 8);

    if( decoderData.head!=0xac43 && decoderData.head!=0xad21 && decoderData.head!=0xfd32
        && decoderData.head!=0xab65 && decoderData.head!=0xcc90 )
        return false;

    decoderData.length=0;
    for (int i = 0; i <(int)sizeof(uint32_t); ++i)
        decoderData.length |= static_cast<uint32_t>(static_cast<uint8_t>(undecodedData[i+2])) << (i * 8);

    if(undecodedData.size()<decoderData.length)
        return false;

    decoderData.seq=undecodedData.at(6);
    decoderData.robot_ID=undecodedData.at(7);

    decoderData.timestamp=0;
    for (int i = 0; i <(int)sizeof(uint64_t); ++i)
        decoderData.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(undecodedData[i+8])) << (i * 8);

    decoderData.check=0;
    decoderData.check = static_cast<uint16_t>(undecodedData[undecodedData.size()-2]) |
                            (static_cast<uint16_t>(undecodedData[undecodedData.size()-1]) << 8);

    //去除开头的消息帧头2个字节，消息大小4个字节，消息编号1个字节，机器人编号1个字节，时间戳8个字节
    //2+4+1+1+8=16
    undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 16);

    //去掉尾部的校验和2个字节
    undecodedData.resize(undecodedData.size()-2);

    switch (decoderData.seq)
    {
    case MessageID::HeartbeatMessageID:
        /*Payload心跳数据反序列化*/
        decoderData.data.heartbeat.agentType=static_cast<uint8_t>(undecodedData[0]);
        decoderData.data.heartbeat.count=0;
        for (int i = 0; i <(int)sizeof(uint32_t); ++i)
            decoderData.data.heartbeat.count |= static_cast<uint32_t>(static_cast<uint8_t>(undecodedData[i+1])) << (i * 8);
        break;
    case MessageID::UAVStateMessageID:
        /*Payload无人机状态数据反序列化*/
        decoderUAVStatePayload(undecodedData,decoderData);
        break;
    case MessageID::UGVStateMessageID:
        /*Payload无人车状态数据反序列化*/
        decoderUGVStatePayload(undecodedData,decoderData);
        break;
    case MessageID::UAVControlCMDMessageID:
        /*Payload无人机控制指令数据反序列化*/
        decoderUAVControlPayload(undecodedData,decoderData);
        break;
    case MessageID::UGVControlCMDMessageID:
        /*Payload无人车控制指令数据反序列化*/
        decoderUGVControlPayload(undecodedData,decoderData);
        break;
    case MessageID::UAVSetupMessageID:
        /*Payload无人机设置指令数据反序列化*/
        decoderUAVSetupPayload(undecodedData,decoderData);
        break;
    case MessageID::SearchMessageID:
        /*Payload搜索在线智能体数据反序列化*/
        decoderData.data.search.port=0;
        for (int i = 0; i <(int)sizeof(uint64_t); ++i)
            decoderData.data.search.port |= static_cast<uint64_t>(static_cast<uint8_t>(undecodedData[i])) << (i * 8);
        break;
    case MessageID::ACKMessageID:
        /*Payload智能体应答数据反序列化*/
        decoderACKPayload(undecodedData,decoderData);
        break;
    case MessageID::DemoMessageID:
        /*Payload智能体demo数据反序列化*/
        decoderDemoPayload(undecodedData,decoderData);
        break;
    case MessageID::ScriptMessageID:
        /*Payload功能脚本数据反序列化*/
        decoderScriptPayload(undecodedData,decoderData);
        break;
    case MessageID::WaypointMessageID:
         /*Payload航点数据反序列化*/
        decoderWaypointPayload(undecodedData,decoderData);
        break;
    case MessageID::NodeMessageID:
        /*Payload机载电脑ROS节点数据反序列化*/
        decoderNodePayload(undecodedData,decoderData);
        break;
    case MessageID::FormationMessageID:case MessageID::GroundFormationMessageID:
        /*Payload编队切换数据反序列化*/
        decoderFormationPayload(undecodedData,decoderData);
        break;
    case MessageID::GoalMessageID:
        /*Payload规划点数据反序列化*/
        decoderGoalPayload(undecodedData,decoderData);
        break;
    case MessageID::AgentComputerStatusMessageID:
        /*Payload智能体电脑状态数据反序列化*/
        decoderAgentComputerStatusPayload(undecodedData,decoderData);
        break;
    default:break;
    }
    return true;

}


uint16_t Codec::PackBytesLE( uint8_t high,uint8_t low)
{
     return (high << 8) | low;
}

void Codec::SetDataFrameHead(DataFrame& codelessData)
{
    switch (codelessData.seq)
    {
    case MessageID::HeartbeatMessageID:case MessageID::UAVControlCMDMessageID:
    case MessageID::UGVControlCMDMessageID:case MessageID::UAVSetupMessageID:
    case MessageID::DemoMessageID:case MessageID::ScriptMessageID:
    case MessageID::WaypointMessageID:case MessageID::GoalMessageID:
        //TCP帧头 0xac43
        codelessData.head=PackBytesLE(0xac,0x43);
        break;
    case MessageID::UAVStateMessageID: case MessageID::UGVStateMessageID:
    case MessageID::NodeMessageID:case MessageID::AgentComputerStatusMessageID:
        //UDP不带回复帧头 0xab65
        codelessData.head=PackBytesLE(0xab,0x65);
        break;
    case MessageID::SearchMessageID:
        //UDP请求帧头 0xad21
        codelessData.head=PackBytesLE(0xad,0x21);
        break;
    case MessageID::ACKMessageID:
        //UDP回复帧头 UDP回复：0xfd32
        codelessData.head=PackBytesLE(0xfd,0x32);
        break;
    case MessageID::FormationMessageID: case MessageID::GroundFormationMessageID:
        //TCP和UDP共用帧头 不带回复回复：0xcc90
        codelessData.head=PackBytesLE(0xcc,0x90);
        break;
    default:break;
    }
}

void Codec::coderUAVStatePayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    UAVState data=codelessData.data.uavState;

    payload.push_back(static_cast<uint8_t>(data.uav_id));
    payload.push_back(static_cast<uint8_t>(data.connected));
    payload.push_back(static_cast<uint8_t>(data.armed));
    payload.push_back(static_cast<uint8_t>(data.mode));
    payload.push_back(static_cast<uint8_t>(data.landed_state));

    floatCopyToUint8tArray(payload,data.battery_state);
    floatCopyToUint8tArray(payload,data.battery_percentage);

    payload.push_back(static_cast<uint8_t>(data.location_source));
    payload.push_back(static_cast<uint8_t>(data.odom_valid));


    floatCopyToUint8tArray(payload,data.position[0]);
    floatCopyToUint8tArray(payload,data.position[1]);
    floatCopyToUint8tArray(payload,data.position[2]);

    floatCopyToUint8tArray(payload,data.velocity[0]);
    floatCopyToUint8tArray(payload,data.velocity[1]);
    floatCopyToUint8tArray(payload,data.velocity[2]);

    floatCopyToUint8tArray(payload,data.attitude[0]);
    floatCopyToUint8tArray(payload,data.attitude[1]);
    floatCopyToUint8tArray(payload,data.attitude[2]);

    floatCopyToUint8tArray(payload,data.attitude_q[0]);
    floatCopyToUint8tArray(payload,data.attitude_q[1]);
    floatCopyToUint8tArray(payload,data.attitude_q[2]);
    floatCopyToUint8tArray(payload,data.attitude_q[3]);

    floatCopyToUint8tArray(payload,data.attitude_rate[0]);
    floatCopyToUint8tArray(payload,data.attitude_rate[1]);
    floatCopyToUint8tArray(payload,data.attitude_rate[2]);

    floatCopyToUint8tArray(payload,data.pos_setpoint[0]);
    floatCopyToUint8tArray(payload,data.pos_setpoint[1]);
    floatCopyToUint8tArray(payload,data.pos_setpoint[2]);

    floatCopyToUint8tArray(payload,data.vel_setpoint[0]);
    floatCopyToUint8tArray(payload,data.vel_setpoint[1]);
    floatCopyToUint8tArray(payload,data.vel_setpoint[2]);

    floatCopyToUint8tArray(payload,data.att_setpoint[0]);
    floatCopyToUint8tArray(payload,data.att_setpoint[1]);
    floatCopyToUint8tArray(payload,data.att_setpoint[2]);

    floatCopyToUint8tArray(payload,data.thrust_setpoint);

    payload.push_back(static_cast<uint8_t>(data.control_mode));
    payload.push_back(static_cast<uint8_t>(data.move_mode));

    floatCopyToUint8tArray(payload,data.takeoff_height);

    floatCopyToUint8tArray(payload,data.home_pos[0]);
    floatCopyToUint8tArray(payload,data.home_pos[1]);
    floatCopyToUint8tArray(payload,data.home_pos[2]);
    floatCopyToUint8tArray(payload,data.home_yaw);

    floatCopyToUint8tArray(payload,data.hover_pos[0]);
    floatCopyToUint8tArray(payload,data.hover_pos[1]);
    floatCopyToUint8tArray(payload,data.hover_pos[2]);
    floatCopyToUint8tArray(payload,data.hover_yaw);

    floatCopyToUint8tArray(payload,data.land_pos[0]);
    floatCopyToUint8tArray(payload,data.land_pos[1]);
    floatCopyToUint8tArray(payload,data.land_pos[2]);
    floatCopyToUint8tArray(payload,data.land_yaw);

}

void Codec::coderUGVStatePayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    UGVState data=codelessData.data.ugvState;

    payload.push_back(static_cast<uint8_t>(data.ugv_id));
    payload.push_back(static_cast<uint8_t>(data.connected));

    floatCopyToUint8tArray(payload,data.battery_state);
    floatCopyToUint8tArray(payload,data.battery_percentage);

    payload.push_back(static_cast<uint8_t>(data.location_source));
    payload.push_back(static_cast<uint8_t>(data.odom_valid));

    floatCopyToUint8tArray(payload,data.position[0]);
    floatCopyToUint8tArray(payload,data.position[1]);

    floatCopyToUint8tArray(payload,data.velocity[0]);
    floatCopyToUint8tArray(payload,data.velocity[1]);

    floatCopyToUint8tArray(payload,data.yaw);

    floatCopyToUint8tArray(payload,data.attitude[0]);
    floatCopyToUint8tArray(payload,data.attitude[1]);
    floatCopyToUint8tArray(payload,data.attitude[2]);

    floatCopyToUint8tArray(payload,data.attitude_q[0]);
    floatCopyToUint8tArray(payload,data.attitude_q[1]);
    floatCopyToUint8tArray(payload,data.attitude_q[2]);
    floatCopyToUint8tArray(payload,data.attitude_q[3]);

    payload.push_back(static_cast<uint8_t>(data.control_mode));

    floatCopyToUint8tArray(payload,data.pos_setpoint[0]);
    floatCopyToUint8tArray(payload,data.pos_setpoint[1]);

    floatCopyToUint8tArray(payload,data.vel_setpoint[0]);
    floatCopyToUint8tArray(payload,data.vel_setpoint[1]);

    floatCopyToUint8tArray(payload,data.yaw_setpoint);

    floatCopyToUint8tArray(payload,data.home_pos[0]);
    floatCopyToUint8tArray(payload,data.home_pos[1]);

    floatCopyToUint8tArray(payload,data.home_yaw);

    floatCopyToUint8tArray(payload,data.hover_pos[0]);
    floatCopyToUint8tArray(payload,data.hover_pos[1]);

    floatCopyToUint8tArray(payload,data.hover_yaw);
}

void Codec::coderUAVControlCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    UAVControlCMD data=codelessData.data.uavControlCMD;

    payload.push_back(static_cast<uint8_t>(data.cmd));

    floatCopyToUint8tArray(payload,data.desired_pos[0]);
    floatCopyToUint8tArray(payload,data.desired_pos[1]);
    floatCopyToUint8tArray(payload,data.desired_pos[2]);

    floatCopyToUint8tArray(payload,data.desired_vel[0]);
    floatCopyToUint8tArray(payload,data.desired_vel[1]);
    floatCopyToUint8tArray(payload,data.desired_vel[2]);

    floatCopyToUint8tArray(payload,data.desired_acc[0]);
    floatCopyToUint8tArray(payload,data.desired_acc[1]);
    floatCopyToUint8tArray(payload,data.desired_acc[2]);

    floatCopyToUint8tArray(payload,data.desired_jerk[0]);
    floatCopyToUint8tArray(payload,data.desired_jerk[1]);
    floatCopyToUint8tArray(payload,data.desired_jerk[2]);

    floatCopyToUint8tArray(payload,data.desired_att[0]);
    floatCopyToUint8tArray(payload,data.desired_att[1]);
    floatCopyToUint8tArray(payload,data.desired_att[2]);

    floatCopyToUint8tArray(payload,data.desired_thrust);
    floatCopyToUint8tArray(payload,data.desired_yaw);
    floatCopyToUint8tArray(payload,data.desired_yaw_rate);


    floatCopyToUint8tArray(payload,data.latitude);
    floatCopyToUint8tArray(payload,data.longitude);
    floatCopyToUint8tArray(payload,data.altitude);

}

void Codec::coderUGVControlCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    UGVControlCMD data=codelessData.data.ugvControlCMD;

    payload.push_back(static_cast<uint8_t>(data.cmd));
    payload.push_back(static_cast<uint8_t>(data.yaw_type));

    floatCopyToUint8tArray(payload,data.desired_pos[0]);
    floatCopyToUint8tArray(payload,data.desired_pos[1]);

    floatCopyToUint8tArray(payload,data.desired_vel[0]);
    floatCopyToUint8tArray(payload,data.desired_vel[1]);

    floatCopyToUint8tArray(payload,data.desired_yaw);
    floatCopyToUint8tArray(payload,data.angular_vel);

}

void Codec::coderUAVSetupCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    UAVSetup data=codelessData.data.uavSetup;
    payload.push_back(static_cast<uint8_t>(data.cmd));
    payload.push_back(static_cast<uint8_t>(data.px4_mode));
    payload.push_back(static_cast<uint8_t>(data.control_mode));
}

 void Codec::coderACKPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
 {
     ACKData data=codelessData.data.ack;
     payload.push_back(static_cast<uint8_t>(data.agentType));
     payload.push_back(static_cast<uint8_t>(data.ID));
     for (int i = 0; i < (int)sizeof(uint16_t); i++)
         payload.push_back(static_cast<uint8_t>((data.port >> (i * 8)) & 0xFF));
 }

 void Codec::coderDemoPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
 {
     DemoData data=codelessData.data.demo;
     payload.push_back(static_cast<uint8_t>(data.demoState));
     for (int i = 0; i < (int)sizeof(uint16_t); i++)
         payload.push_back(static_cast<uint8_t>((data.demoSize >> (i * 8)) & 0xFF));

     if(payload.capacity()<data.demoSize+3)
         payload.reserve(data.demoSize+3);

     for(int j=0;j<data.demoSize;++j)
         payload.push_back(static_cast<uint8_t>(data.demoStr[j]));
 }

 void Codec::coderScriptPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
 {
     ScriptData data=codelessData.data.agentScrip;
     payload.push_back(static_cast<uint8_t>(data.scripType));
     payload.push_back(static_cast<uint8_t>(data.scriptState));
     for (int i = 0; i < (int)sizeof(uint16_t); i++)
         payload.push_back(static_cast<uint8_t>((data.scriptSize >> (i * 8)) & 0xFF));

     if(payload.capacity()<data.scriptSize+6)
         payload.reserve(data.scriptSize+6);

     for(int j=0;j<data.scriptSize;++j)
         payload.push_back(static_cast<uint8_t>(data.scriptStr[j]));

 }

 void Codec::coderWaypointPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
 {
     WaypointData data=codelessData.data.waypointData;
     payload.push_back(static_cast<uint8_t>(data.wp_num));
     payload.push_back(static_cast<uint8_t>(data.wp_type));
     payload.push_back(static_cast<uint8_t>(data.wp_end_type));
     payload.push_back(static_cast<uint8_t>(data.wp_takeoff));
     payload.push_back(static_cast<uint8_t>(data.wp_yaw_type));

     floatCopyToUint8tArray(payload,data.wp_move_vel);
     floatCopyToUint8tArray(payload,data.wp_vel_p);
     floatCopyToUint8tArray(payload,data.z_height);

     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_1[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_2[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_3[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_4[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_5[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_6[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_7[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_8[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_9[i]);
     for(int i=0;i<4;i++)
         doubleCopyToUint8tArray(payload,data.wp_point_10[i]);

     doubleCopyToUint8tArray(payload,data.wp_circle_point[0]);
     doubleCopyToUint8tArray(payload,data.wp_circle_point[1]);
 }

void Codec::coderNodePayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    NodeData data=codelessData.data.nodeInformation;

    for (int i = 0; i < (int)sizeof(uint16_t); i++)
        payload.push_back(static_cast<uint8_t>((data.nodeCount >> (i * 8)) & 0xFF));

    for (int i = 0; i < (int)sizeof(uint16_t); i++)
        payload.push_back(static_cast<uint8_t>((data.nodeID >> (i * 8)) & 0xFF));

    for (int i = 0; i < (int)sizeof(uint16_t); i++)
        payload.push_back(static_cast<uint8_t>((data.nodeSize >> (i * 8)) & 0xFF));

    if(payload.capacity()<data.nodeSize+8)
        payload.reserve(data.nodeSize+8);

    for(int j=0;j<data.nodeSize;++j)
        payload.push_back(static_cast<uint8_t>(data.nodeStr[j]));
}

void Codec::coderGoalPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    Goal data=codelessData.data.goal;
    doubleCopyToUint8tArray(payload,data.positionX);
    doubleCopyToUint8tArray(payload,data.positionY);
    doubleCopyToUint8tArray(payload,data.positionZ);

}

void Codec::coderAgentComputerStatusload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    AgentComputerStatus data=codelessData.data.computerStatus;
    doubleCopyToUint8tArray(payload,data.cpuLoad);
    doubleCopyToUint8tArray(payload,data.memoryUsage);
    doubleCopyToUint8tArray(payload,data.cpuTemperature);
}

void Codec::coderFormationPayload(std::vector<uint8_t>& payload,DataFrame& codelessData)
{
    Formation data=codelessData.data.formation;
    payload.push_back(static_cast<uint8_t>(data.cmd));
    payload.push_back(static_cast<uint8_t>(data.formation_type));
    payload.push_back(static_cast<uint8_t>(data.leader_id));

    for (int i = 0; i < (int)sizeof(uint16_t); i++)
        payload.push_back(static_cast<uint8_t>((data.nameSize >> (i * 8)) & 0xFF));

    if(payload.capacity()<data.nameSize+4)
        payload.reserve(data.nameSize+4);

    for(int j=0;j<data.nameSize;++j)
        payload.push_back(static_cast<uint8_t>(data.name[j]));
}

std::vector<uint8_t> Codec::coder(DataFrame codelessData)
{
    std::vector<uint8_t> coderData,PayloadData;

    if(validMessageID.count(codelessData.seq)==0)
        return coderData;

    /*对应的Payload数据序列化*/
    switch (codelessData.seq)
    {
    case MessageID::HeartbeatMessageID:
        /*Payload心跳数据序列化*/
        PayloadData.push_back(static_cast<uint8_t>(codelessData.data.heartbeat.agentType));
        //插入心跳包计数（未赋值,未使用）
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            PayloadData.push_back(static_cast<uint8_t>((codelessData.data.heartbeat.count>> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
        break;
    case MessageID::UAVStateMessageID:
        /*Payload无人机状态数据序列化*/
        coderUAVStatePayload(PayloadData,codelessData);
        break;
    case MessageID::UGVStateMessageID:
        /*Payload无人车状态数据序列化*/
        coderUGVStatePayload(PayloadData,codelessData);
        break;
    case MessageID::UAVControlCMDMessageID:
        /*Payload无人机控制指令数据序列化*/
        coderUAVControlCMDPayload(PayloadData,codelessData);
        break;
    case MessageID::UGVControlCMDMessageID:
        /*Payload无人车控制指令数据序列化*/
        coderUGVControlCMDPayload(PayloadData,codelessData);
        break;
    case MessageID::UAVSetupMessageID:
        /*Payload无人机设置指令数据序列化*/
        coderUAVSetupCMDPayload(PayloadData,codelessData);
        break;
    case MessageID::SearchMessageID:
        /*Payload搜索在线智能体数据序列化*/
        for (int i = 0; i < (int)sizeof(uint64_t); ++i)
            PayloadData.push_back(static_cast<uint8_t>((codelessData.data.search.port >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
        break;
    case MessageID::ACKMessageID:
        /*Payload智能体应答数据序列化*/
        coderACKPayload(PayloadData,codelessData);
        break;
    case MessageID::DemoMessageID:
        /*Payload智能体demo数据序列化*/
        coderDemoPayload(PayloadData,codelessData);
        break;
    case MessageID::ScriptMessageID:
        /*Payload功能脚本数据序列化*/
        coderScriptPayload(PayloadData,codelessData);
        break;
    case MessageID::WaypointMessageID:
         /*Payload航点数据序列化*/
        coderWaypointPayload(PayloadData,codelessData);
        break;
    case MessageID::NodeMessageID:
        /*Payload机载电脑ROS节点数据序列化*/
        coderNodePayload(PayloadData,codelessData);
        break;
    case MessageID::FormationMessageID:case MessageID::GroundFormationMessageID:
        /*Payload编队切换数据序列化*/
        coderFormationPayload(PayloadData,codelessData);
        break;
    case MessageID::GoalMessageID:
        /*Payload规划点数据序列化*/
        coderGoalPayload(PayloadData,codelessData);
        break;
    case MessageID::AgentComputerStatusMessageID:
        /*Payload智能体电脑状态数据序列化*/
        coderAgentComputerStatusload(PayloadData,codelessData);
        break;
    default:break;
    }

    SetDataFrameHead(codelessData);//设置消息帧头
    safeConvertToUint32(PayloadData.size(),codelessData.length);//获得数据帧长度
    codelessData.timestamp=getTimestamp();//获取时间戳

    /*插入消息帧头（大端序）*/
    coderData.push_back((codelessData.head >> 8) & 0xFF);  // 高位字节（高8位）先存储
    coderData.push_back(codelessData.head & 0xFF);         // 低位字节（低8位）后存储

    /*插入消息长度*/
    for (int i = 0; i < (int)sizeof(uint32_t); ++i)
        coderData.push_back(static_cast<uint8_t>((codelessData.length >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

    /*插入消息编号*/
    coderData.push_back(codelessData.seq);

    /*插入机器人编号*/
    coderData.push_back(codelessData.robot_ID);

    /*插入时间戳*/
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        coderData.push_back(static_cast<uint8_t>((codelessData.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

    /*将整个Payload数据从尾部插入数据里面*/
    coderData.insert(coderData.end(), PayloadData.begin(), PayloadData.end());

    codelessData.check=getChecksum(coderData);//获取校验值

    /*插入校验值*/
    coderData.push_back(static_cast<uint8_t>((codelessData.check >> 8) & 0xFF)); // 存储高位字节
    coderData.push_back(static_cast<uint8_t>(codelessData.check & 0xFF)); // 存储低位

    return coderData;

}

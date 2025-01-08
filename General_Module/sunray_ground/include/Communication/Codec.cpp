#include "Codec.h"

Codec::Codec()
{
    UDPMessageNum=0;
    TCPMessageNum=0;
    UDPBroadcastMessageNum=0;
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

void Codec::floatCopyToUint8tArray(std::vector<uint8_t>& data,float& value)
{
    uint8_t floatBytes[sizeof(float)];
    // 使用 std::memcpy 安全地将 float 的字节复制到 uint8_t 数组中
    std::memcpy(floatBytes, &value, sizeof(float));
    // 将每个字节添加到 data 中
    for (int j = 0; j < (int)sizeof(float); ++j)
        data.push_back(floatBytes[j]);
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
        convertedSize = static_cast<uint32_t>(originalSize);

}

uint8_t Codec::getUDPBroadcastMessageNum()
{
    if(UDPBroadcastMessageNum > 200)
        UDPBroadcastMessageNum = 1;
    else
        UDPBroadcastMessageNum++;

    return UDPBroadcastMessageNum;
}


uint8_t Codec::getUDPMessageNum()
{
    if(UDPMessageNum > 200)
        UDPMessageNum = 1;
    else
        UDPMessageNum++;

    return UDPMessageNum;
}


uint8_t Codec::getTCPMessageNum()
{
    if(TCPMessageNum > 200)
        TCPMessageNum = 1;
    else
        TCPMessageNum++;

    return TCPMessageNum;
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

void Codec::coderStateDataFrame(std::vector<uint8_t>& dataFrame,StateData& state)
{
    /*状态数据帧封装*/
    dataFrame.push_back(static_cast<uint8_t>(MessageID::StateMessageID));
    state.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((state.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
    //std::cout << "状态数据发送的时间戳 "<<state.timestamp<<std::endl;

    dataFrame.push_back(static_cast<uint8_t>(state.uavID));
    dataFrame.push_back(static_cast<uint8_t>(state.connected));
    dataFrame.push_back(static_cast<uint8_t>(state.armed));
    dataFrame.push_back(static_cast<uint8_t>(state.mode));
    dataFrame.push_back(static_cast<uint8_t>(state.locationSource));
    dataFrame.push_back(static_cast<uint8_t>(state.odom_valid));

    floatCopyToUint8tArray(dataFrame,state.position.x);
    floatCopyToUint8tArray(dataFrame,state.position.y);
    floatCopyToUint8tArray(dataFrame,state.position.z);

    floatCopyToUint8tArray(dataFrame,state.velocity.x);
    floatCopyToUint8tArray(dataFrame,state.velocity.y);
    floatCopyToUint8tArray(dataFrame,state.velocity.z);


    floatCopyToUint8tArray(dataFrame,state.attitudeQuaternion.w);
    floatCopyToUint8tArray(dataFrame,state.attitudeQuaternion.x);
    floatCopyToUint8tArray(dataFrame,state.attitudeQuaternion.y);
    floatCopyToUint8tArray(dataFrame,state.attitudeQuaternion.z);


    floatCopyToUint8tArray(dataFrame,state.attitude.x);
    floatCopyToUint8tArray(dataFrame,state.attitude.y);
    floatCopyToUint8tArray(dataFrame,state.attitude.z);

    floatCopyToUint8tArray(dataFrame,state.attitudeRate.x);
    floatCopyToUint8tArray(dataFrame,state.attitudeRate.y);
    floatCopyToUint8tArray(dataFrame,state.attitudeRate.z);

    floatCopyToUint8tArray(dataFrame,state.posSetpoint.x);
    floatCopyToUint8tArray(dataFrame,state.posSetpoint.y);
    floatCopyToUint8tArray(dataFrame,state.posSetpoint.z);

    floatCopyToUint8tArray(dataFrame,state.velSetpoint.x);
    floatCopyToUint8tArray(dataFrame,state.velSetpoint.y);
    floatCopyToUint8tArray(dataFrame,state.velSetpoint.z);

    floatCopyToUint8tArray(dataFrame,state.attSetpoint.x);
    floatCopyToUint8tArray(dataFrame,state.attSetpoint.y);
    floatCopyToUint8tArray(dataFrame,state.attSetpoint.z);

    floatCopyToUint8tArray(dataFrame,state.batteryState);
    floatCopyToUint8tArray(dataFrame,state.batteryPercentage);

    dataFrame.push_back(static_cast<uint8_t>(state.controlMode));
    dataFrame.push_back(static_cast<uint8_t>(state.moveMode));

}

void Codec::decoderStateDataFrame(std::vector<uint8_t>& dataFrame,StateData& state) //编码状态数据帧
{
    //时间戳赋值
    state.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    int i;
    for (i = 0; i <(int)sizeof(uint64_t); ++i)
        state.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);

    state.uavID=static_cast<uint8_t>(dataFrame[i++]);
    state.connected=static_cast<uint8_t>(dataFrame[i++]);
    state.armed=static_cast<uint8_t>(dataFrame[i++]);
    state.mode=static_cast<uint8_t>(dataFrame[i++]);

    state.locationSource=static_cast<uint8_t>(dataFrame[i++]);
    state.odom_valid=static_cast<uint8_t>(dataFrame[i++]);

    //去除开头的消息帧头2个字节，消息大小4个字节，消息序号1个字节，2+4+1=7
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + i);

//    for (const auto& elem : dataFrame) {
//        // 创建一个临时的 ostringstream 来格式化数字
//                std::ostringstream oss;
//                // 设置输出为16进制，并设置每个数字的宽度为2，用0填充
//                oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(elem);
//                // 将格式化的字符串输出到标准输出流
//                std::cout << oss.str() << " ";
//       }
//    std::cout <<std::endl;
    uint8tArrayToFloat(dataFrame,state.position.x);
//    std::cout << "位置x "<<(int)dataFrame.size()<<std::endl;


    uint8tArrayToFloat(dataFrame,state.position.y);
    uint8tArrayToFloat(dataFrame,state.position.z);

    uint8tArrayToFloat(dataFrame,state.velocity.x);
    uint8tArrayToFloat(dataFrame,state.velocity.y);
    uint8tArrayToFloat(dataFrame,state.velocity.z);

    uint8tArrayToFloat(dataFrame,state.attitudeQuaternion.w);
    uint8tArrayToFloat(dataFrame,state.attitudeQuaternion.x);
    uint8tArrayToFloat(dataFrame,state.attitudeQuaternion.y);
    uint8tArrayToFloat(dataFrame,state.attitudeQuaternion.z);

    uint8tArrayToFloat(dataFrame,state.attitude.x);
    uint8tArrayToFloat(dataFrame,state.attitude.y);
    uint8tArrayToFloat(dataFrame,state.attitude.z);

    uint8tArrayToFloat(dataFrame,state.attitudeRate.x);
    uint8tArrayToFloat(dataFrame,state.attitudeRate.y);
    uint8tArrayToFloat(dataFrame,state.attitudeRate.z);

    uint8tArrayToFloat(dataFrame,state.posSetpoint.x);
    uint8tArrayToFloat(dataFrame,state.posSetpoint.y);
    uint8tArrayToFloat(dataFrame,state.posSetpoint.z);

    uint8tArrayToFloat(dataFrame,state.velSetpoint.x);
    uint8tArrayToFloat(dataFrame,state.velSetpoint.y);
    uint8tArrayToFloat(dataFrame,state.velSetpoint.z);

    uint8tArrayToFloat(dataFrame,state.attSetpoint.x);
    uint8tArrayToFloat(dataFrame,state.attSetpoint.y);
    uint8tArrayToFloat(dataFrame,state.attSetpoint.z);

    uint8tArrayToFloat(dataFrame,state.batteryState);
    uint8tArrayToFloat(dataFrame,state.batteryPercentage);

    state.controlMode=static_cast<uint8_t>(dataFrame[0]);
    state.moveMode=static_cast<uint8_t>(dataFrame[1]);
}


void Codec::decoderControlDataFrame(std::vector<uint8_t>& dataFrame,ControlData& control) //编码控制数据帧
{
    //时间戳赋值
    control.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    int i;
    for (i = 0; i <(int)sizeof(uint64_t); ++i)
        control.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);

    control.controlMode=static_cast<uint8_t>(dataFrame[i++]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + i);

    uint8tArrayToFloat(dataFrame,control.position.x);
    uint8tArrayToFloat(dataFrame,control.position.y);
    uint8tArrayToFloat(dataFrame,control.position.z);
    uint8tArrayToFloat(dataFrame,control.velocity.x);
    uint8tArrayToFloat(dataFrame,control.velocity.y);
    uint8tArrayToFloat(dataFrame,control.velocity.z);
    uint8tArrayToFloat(dataFrame,control.accelerometer.x);
    uint8tArrayToFloat(dataFrame,control.accelerometer.y);
    uint8tArrayToFloat(dataFrame,control.accelerometer.z);

    uint8tArrayToFloat(dataFrame,control.yaw);
    uint8tArrayToFloat(dataFrame,control.roll);
    uint8tArrayToFloat(dataFrame,control.pitch);

//    control.yawRate=static_cast<uint8_t>(dataFrame[0]);
////    control.frame=static_cast<uint8_t>(dataFrame[1]);
//    dataFrame.erase(dataFrame.begin(), dataFrame.begin() +1);

    uint8tArrayToFloat(dataFrame,control.latitude);
    uint8tArrayToFloat(dataFrame,control.longitude);
    uint8tArrayToFloat(dataFrame,control.altitude);

    uint8tArrayToFloat(dataFrame,control.yawRate);

}

void Codec::coderDemoDataFrame(std::vector<uint8_t>& dataFrame,DemoData& demo) //编码无人机demo数据帧
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::DemoMessageID));
    demo.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((demo.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

    dataFrame.push_back(static_cast<uint8_t>(demo.uavID));
    dataFrame.push_back(static_cast<uint8_t>(demo.demoState));

    for (int i = 0; i < (int)sizeof(uint16_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((demo.demoSize >> (i * 8)) & 0xFF));

    if(dataFrame.capacity()<demo.demoSize)
        dataFrame.reserve(demo.demoSize);

    for(int j=0;j<demo.demoSize;++j)
    dataFrame.push_back(static_cast<uint8_t>(demo.demoStr[j]));

}

void Codec::coderControlDataFrame(std::vector<uint8_t>& dataFrame,ControlData& control)
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::ControlMessageID));
    control.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((control.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

    //std::cout << "参数控制数据发送的时间戳 "<<control.timestamp<<std::endl;

    dataFrame.push_back(static_cast<uint8_t>(control.controlMode));

    floatCopyToUint8tArray(dataFrame,control.position.x);
    floatCopyToUint8tArray(dataFrame,control.position.y);
    floatCopyToUint8tArray(dataFrame,control.position.z);
    floatCopyToUint8tArray(dataFrame,control.velocity.x);
    floatCopyToUint8tArray(dataFrame,control.velocity.y);
    floatCopyToUint8tArray(dataFrame,control.velocity.z);
    floatCopyToUint8tArray(dataFrame,control.accelerometer.x);
    floatCopyToUint8tArray(dataFrame,control.accelerometer.y);
    floatCopyToUint8tArray(dataFrame,control.accelerometer.z);

    floatCopyToUint8tArray(dataFrame,control.yaw);
    floatCopyToUint8tArray(dataFrame,control.roll);
    floatCopyToUint8tArray(dataFrame,control.pitch);


    floatCopyToUint8tArray(dataFrame,control.latitude);
    floatCopyToUint8tArray(dataFrame,control.longitude);
    floatCopyToUint8tArray(dataFrame,control.altitude);

    floatCopyToUint8tArray(dataFrame,control.yawRate);

//    dataFrame.push_back(static_cast<uint8_t>(control.frame));

}

void Codec::coderVehicleDataFrame(std::vector<uint8_t>& dataFrame,VehicleData& vehicle)
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::VehicleMessageID));
    vehicle.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((vehicle.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
    dataFrame.push_back(static_cast<uint8_t>(vehicle.sunray_mode));
    dataFrame.push_back(static_cast<uint8_t>(vehicle.px4_mode));

    //std::cout << "模式切换数据发送的时间戳 "<<vehicle.timestamp<<std::endl;

}

void Codec::decoderVehicleDataFrame(std::vector<uint8_t>& dataFrame,VehicleData& vehicle) //编码模式切换数据帧
{
    //时间戳赋值
    vehicle.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    int i;
    for (i = 0; i <(int)sizeof(uint64_t); ++i)
        vehicle.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);

    vehicle.sunray_mode=static_cast<uint8_t>(dataFrame[i++]);
    vehicle.px4_mode=static_cast<uint8_t>(dataFrame[i++]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + i);
}

void Codec::coderACKDataFrame(std::vector<uint8_t>& dataFrame,ACKData& ack)
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::ACKMessageID));
    ack.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((ack.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
    dataFrame.push_back(static_cast<uint8_t>(ack.uavID));
    for (int i = 0; i < (int)sizeof(uint16_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((ack.port >> (i * 8)) & 0xFF));
}



void Codec::coderSearchDataFrame(std::vector<uint8_t>& dataFrame,SearchData& search)
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::SearchMessageID));
    search.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((search.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((search.port >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
}

void Codec::coderHeartDataFrame(std::vector<uint8_t>& dataFrame,HeartbeatData& heartbeat)
{
    dataFrame.push_back(static_cast<uint8_t>(MessageID::HeartbeatMessageID));
    heartbeat.timestamp=getTimestamp();
    for (int i = 0; i < (int)sizeof(uint64_t); ++i)
        dataFrame.push_back(static_cast<uint8_t>((heartbeat.timestamp >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
    //心跳包缺心跳包计数，int32_t count 4个字节
    //std::cout << "心跳数据发送的时间戳 "<<heartbeat.timestamp<<std::endl;

}

void Codec::decoderACKDataFrame(std::vector<uint8_t>& dataFrame,ACKData& ack)
{
    //时间戳赋值
    ack.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    int i;
    for (i = 0; i <(int)sizeof(uint64_t); ++i)
        ack.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);
    ack.uavID=static_cast<uint8_t>(dataFrame[i++]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + i);
    ack.port = 0;
    for (int j = 0; j < (int)sizeof(uint16_t); ++j)
        ack.port |= (static_cast<uint16_t>(dataFrame[j]) << (j * 8));

}

void Codec::decoderDemoDataFrame(std::vector<uint8_t>& dataFrame,DemoData& demo)
{    
    //时间戳赋值
    demo.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    int i;
    for (i = 0; i <(int)sizeof(uint64_t); ++i)
        demo.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);
    demo.uavID=static_cast<uint8_t>(dataFrame[i++]);
    demo.demoState=static_cast<uint8_t>(dataFrame[i++]);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + i);

     demo.demoSize=0;
    for (int j = 0; j < (int)sizeof(uint16_t); ++j)
        demo.demoSize |= (static_cast<uint16_t>(dataFrame[j]) << (j * 8));


    for (int j = 2; j < demo.demoSize+2; ++j)
        demo.demoStr[j-2]=static_cast<uint8_t>(dataFrame[j]);

    dataFrame.clear();
}



void Codec::decoderSearchDataFrame(std::vector<uint8_t>& dataFrame,SearchData& search)
{
    //时间戳赋值
    search.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    for (int i = 0; i <(int)sizeof(uint64_t); ++i)
        search.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);
    search.port = 0;
    for (int i = 0; i <(int)sizeof(uint64_t); ++i)
        search.port |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[(int)sizeof(uint64_t)+i])) << (i * 8);
    dataFrame.erase(dataFrame.begin(), dataFrame.begin() + (int)sizeof(uint64_t)*2);


}


void Codec::decoderHeartDataFrame(std::vector<uint8_t>& dataFrame,HeartbeatData& heartbeat) //解码无人机心跳包数据帧
{
    //时间戳赋值
    heartbeat.timestamp=0;
    // 由于数据是按大端顺序存储的，我们直接左移i*8位并添加对应的字节
    for (int i = 0; i <(int)sizeof(uint64_t); ++i)
        heartbeat.timestamp |= static_cast<uint64_t>(static_cast<uint8_t>(dataFrame[i])) << (i * 8);

    //心跳包缺心跳包计数，int32_t count 4个字节
}

bool Codec::decoder(std::vector<uint8_t> undecodedData,int& messageID,unionData& decoderData)
{
    //std::cout << "数据帧解码准备数据大小 "<<undecodedData.size()<<std::endl;
    uint8_t first= static_cast<uint8_t>(undecodedData.at(0));
    if(first!=0xac && first!=0xad && first!=0xfd &&first!=0xab)
        return false;

    uint8_t second= static_cast<uint8_t>(undecodedData.at(1));
    if(second!=0x43 && second!=0x21 && second!=0x32 &&second!=0x65)
        return false;

    //去除开头的消息帧头2个字节，消息大小4个字节，消息序号1个字节，2+4+1=7
    undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 7);
    //去掉尾部的校验和2个字节
    undecodedData.resize(undecodedData.size()-2);

    //第2个元素是数据帧类型
    messageID=static_cast<int>(static_cast<uint8_t>(undecodedData[1]));
    //std::cout << "消息类型ID "<<(int)undecodedData[1]<<std::endl;
    switch (messageID)
    {
    case MessageID::HeartbeatMessageID://心跳包解码
        //第一个元素是机器人编号
        decoderData.heartbeat.robotID=static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderHeartDataFrame(undecodedData,decoderData.heartbeat);
        break;
    case MessageID::StateMessageID:
        //第一个元素是机器人编号
        decoderData.state.robotID=static_cast<uint8_t>(undecodedData.at(0));
        //std::cout << "机器人编号 "<<(int)undecodedData[0]<<std::endl;
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderStateDataFrame(undecodedData,decoderData.state);
        break;
    case MessageID::ControlMessageID:
        //第一个元素是机器人编号
        decoderData.contro.robotID=static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);

        decoderControlDataFrame(undecodedData,decoderData.contro);
        break;
    case MessageID::VehicleMessageID:
        //第一个元素是机器人编号
        decoderData.vehicle.robotID=static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderVehicleDataFrame(undecodedData,decoderData.vehicle);
        break;
    case MessageID::SearchMessageID:
        //第一个元素是机器人编号
        decoderData.search.robotID =static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderSearchDataFrame(undecodedData,decoderData.search);
        break;
    case MessageID::ACKMessageID:
        //第一个元素是机器人编号
        decoderData.ack.robotID =static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderACKDataFrame(undecodedData,decoderData.ack);
        break;
    case MessageID::DemoMessageID:
        decoderData.demo.robotID=static_cast<uint8_t>(undecodedData.at(0));
        //去掉之前已经读出的两个元素
        undecodedData.erase(undecodedData.begin(), undecodedData.begin() + 2);
        decoderDemoDataFrame(undecodedData,decoderData.demo);
        break;
    default:break;
    }


    return true;
}



std::vector<uint8_t> Codec::coder(int messageID,unionData codelessData)
{
    std::vector<uint8_t> coderData,tempData;
    coderData.clear();
    uint16_t checksum=0;
    uint32_t dataFrameSize=0;

    tempData.clear();
    coderData.clear();


    switch (messageID)
    {
    case MessageID::HeartbeatMessageID://心跳包编码感觉不应该用这个编码
        //TCP帧头 0xac43
        coderData.push_back(0xac);
        coderData.push_back(0x43);

        /*编码心跳包数据帧*/
        coderHeartDataFrame(tempData,codelessData.heartbeat);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度

        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节
        //std::cout << "心跳包数据里的消息长度 "<<dataFrameSize<<std::endl;

        // 打印 sizeVector 中的全部数据
//        std::cout << "心跳包数据字节: ";
//        for (uint8_t byte : coderData)
//        {
//            // 使用 std::hex 设置输出为十六进制格式
//            // 使用 std::setw(2) 和 std::setfill('0') 来确保每个字节都占用两个字符宽度，并用 0 填充
//            std::cout <<std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
//        }
//        std::cout << std::endl; // 输出换行符以结束输出行


        coderData.push_back(getTCPMessageNum());//获取TCP消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.heartbeat.robotID));//获取机器人ID
        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值

        // 将uint16_t的高位字节直接转换为uint8_t并存储到vector中
        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        // 将uint16_t的低位字节直接转换为uint8_t并存储到vector中
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        // 打印 sizeVector 中的全部数据
//        std::cout << "心跳包全部数据字节: ";
//        for (uint8_t byte : coderData)
//        {
//            // 使用 std::hex 设置输出为十六进制格式
//            // 使用 std::setw(2) 和 std::setfill('0') 来确保每个字节都占用两个字符宽度，并用 0 填充
//            std::cout <<std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
//        }
//        std::cout << std::endl; // 输出换行符以结束输出行
        break;
    case MessageID::StateMessageID:
        //UDP不带回复帧头 0xab65
        coderData.push_back(0xab);
        coderData.push_back(0x65);

        /*编码状态数据帧*/
        coderStateDataFrame(tempData,codelessData.state);

        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度
        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getUDPMessageNum());//获取UDP消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.state.robotID));//获取机器人ID

        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值

        // 将uint16_t的高位字节直接转换为uint8_t并存储到vector中
        // 这里使用static_cast<uint8_t>是安全的，因为我们知道结果是一个在uint8_t范围内的值（0-255）
        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        // 将uint16_t的低位字节直接转换为uint8_t并存储到vector中
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        break;
    case MessageID::ControlMessageID:
        //TCP帧头 0xac43
        coderData.push_back(0xac);
        coderData.push_back(0x43);

        /*编码无人机参数控制数据*/
        coderControlDataFrame(tempData,codelessData.contro);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度
        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getTCPMessageNum());//获取TCP消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.contro.robotID));//获取机器人ID
        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值

        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        break;
    case MessageID::VehicleMessageID:
        //TCP帧头 0xac43
        coderData.push_back(0xac);
        coderData.push_back(0x43);
        //UDP不带回复帧头 0xab65
//        coderData.push_back(0xab);
//        coderData.push_back(0x65);

        /*编码控制模式数据帧*/
        coderVehicleDataFrame(tempData,codelessData.vehicle);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度
        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getTCPMessageNum());//获取TCP消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.vehicle.robotID));//获取机器人ID
        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值

        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        break;
    case MessageID::SearchMessageID:
        //UDP请求帧头 0xad21
        coderData.push_back(0xad);
        coderData.push_back(0x21);

        /*编码搜索在线无人机数据帧*/
        coderSearchDataFrame(tempData,codelessData.search);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度

        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getUDPBroadcastMessageNum());//获取UDP广播消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.search.robotID));//获取机器人ID

        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值
        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        break;
    case MessageID::ACKMessageID:
        //UDP回复帧头 UDP回复：0xfd32
        coderData.push_back(0xfd);
        coderData.push_back(0x32);
        coderACKDataFrame(tempData,codelessData.ack);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度

        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getUDPMessageNum());//获取UDP广播消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.ack.robotID));//获取机器人ID

        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值
        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位
        break;
    case MessageID::DemoMessageID:
        //TCP帧头 0xac43
        coderData.push_back(0xac);
        coderData.push_back(0x43);

        coderDemoDataFrame(tempData,codelessData.demo);
        safeConvertToUint32(tempData.size(),dataFrameSize);//获得数据帧长度
        dataFrameSize=dataFrameSize+10;//计算整帧数据的长度
        for (int i = 0; i < (int)sizeof(uint32_t); ++i)
            coderData.push_back(static_cast<uint8_t>((dataFrameSize >> (i * 8)) & 0xFF));// 使用位移和掩码提取每个字节

        coderData.push_back(getTCPMessageNum());//获取TCP消息序号
        coderData.push_back(static_cast<uint8_t>(codelessData.demo.robotID));//获取机器人ID
        //将整个数据帧从尾部加入数据里面
        coderData.insert(coderData.end(), tempData.begin(), tempData.end());

        checksum=getChecksum(coderData);//获取校验值

        coderData.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF)); // 存储高位字节
        coderData.push_back(static_cast<uint8_t>(checksum & 0xFF)); // 存储低位

        break;
    default:break;


    }
    return coderData;
}

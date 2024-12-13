#ifndef CODEC_H
#define CODEC_H

#include "Communication/MSG.h"
#include <cstring>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>

class Codec
{
public:
    Codec();
    std::vector<uint8_t> coder(int messageID,unionData codelessData);//数据编码，返回编码完成的数据，传入消息类型和消息联合体
    bool decoder(std::vector<uint8_t> undecodedData,int& messageID,unionData& decoderData);//要求传入的是一条完整的数据


    void coderStateDataFrame(std::vector<uint8_t>& dataFrame,StateData& state);//编码状态数据帧
    void coderControlDataFrame(std::vector<uint8_t>& dataFrame,ControlData& control);//编码无人机参数控制数据
    void coderVehicleDataFrame(std::vector<uint8_t>& dataFrame,VehicleData& vehicle); //编码无人机模式切换数据帧
    void coderHeartDataFrame(std::vector<uint8_t>& dataFrame,HeartbeatData& heartbeat); //编码无人机心跳包数据帧
    void coderSearchDataFrame(std::vector<uint8_t>& dataFrame,SearchData& search); //编码搜索在线无人机数据帧
    void coderACKDataFrame(std::vector<uint8_t>& dataFrame,ACKData& ack); //编码无人机应答数据帧


    void decoderHeartDataFrame(std::vector<uint8_t>& dataFrame,HeartbeatData& heartbeat); //解码无人机心跳包数据帧
    void decoderStateDataFrame(std::vector<uint8_t>& dataFrame,StateData& state);//编码状态数据帧
    void decoderControlDataFrame(std::vector<uint8_t>& dataFrame,ControlData& control);//编码控制数据帧
    void decoderVehicleDataFrame(std::vector<uint8_t>& dataFrame,VehicleData& vehicle);//编码模式切换数据帧
    void decoderSearchDataFrame(std::vector<uint8_t>& dataFrame,SearchData& search); //解码搜索在线无人机数据帧
    void decoderACKDataFrame(std::vector<uint8_t>& dataFrame,ACKData& ack); //解码无人机应答数据帧


    uint64_t getTimestamp();//获得uint64_t类型的时间戳

    void floatArrayCopyToUint8tArray(std::vector<uint8_t>& data,std::vector<float>& value);//std::vector<float>数据加入std::vector<uint8_t>
    void floatCopyToUint8tArray(std::vector<uint8_t>& data,float& value);//float数据加入std::vector<uint8_t>
    void uint8tArrayToFloat(std::vector<uint8_t>& data, float& value);

    void safeConvertToUint32(size_t originalSize, uint32_t& convertedSize);//将size_t值安全转换为uint32_t的值

    uint16_t getChecksum(std::vector<uint8_t> data);//获取校验值
    uint8_t getUDPMessageNum();//获取UDP消息序号
    uint8_t getUDPBroadcastMessageNum();//获取UDP广播消息序号
    uint8_t getTCPMessageNum();//获得TCP消息序号

private:
    uint8_t UDPMessageNum;//UDP消息序号
    uint8_t UDPBroadcastMessageNum;//UDP广播消息序号
    uint8_t TCPMessageNum;//TCP消息序号

};

#endif // CODEC_H

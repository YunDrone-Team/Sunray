#ifndef CODEC_H
#define CODEC_H

#include "Communication/MSG.h"
#include "Communication/DecoderInterfaceBase.h"
#include <cstring>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unordered_set>

class Codec: public DecoderInterfaceBase
{
public:
    Codec();

    /**
     * 根据通信协议对DataFrame进行编码，生成可传输的序列化数据帧。
     * @param codelessData 待编码的数据帧，包含：
     *   - seq: 消息编号（对应MSG.h中MessageID的枚举值）
     *   - data: 消息的有效载荷（Payload），需与消息编号匹配
     *   - robot_ID: 无人机ID不变，无人车ID+100
     * @return 序列化后的完整数据帧，格式为：
     *   [帧头(2B)][消息长度(4B)][消息编号(1B)][机器人编号(1B)][时间戳(8B)][payload...][校验和(2B)]
     */
    std::vector<uint8_t> coder(DataFrame codelessData);

    /**
     * 根据通信协议反序列化数据帧，解析数据
     * @param undecodedData 待解析的完整数据帧字节流（需包含帧头、消息编号、数据长度等协议字段）
     * @param decoderData 输出参数，反序列化后的消息内容：
     *   - seq: 解析得到的消息编号（对应MSG.h中MessageID的枚举值）
     *   - data: 解析得到的有效载荷（Payload）数据
     *   - robot_ID: 解析得到的机器人编号数据
     * @return 反序列化成功返回true，失败返回false（如帧格式错误、校验失败）
     * @note 输入数据必须是包含完整协议头的一帧且通过校验的数据
     * @see coder() 配套的序列化函数
     */
    bool decoder(std::vector<uint8_t> undecodedData,DataFrame& decoderData);

    void SetDataFrameHead(DataFrame& codelessData);//给数据帧帧头赋值

    void coderUAVStatePayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderUGVStatePayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderUAVControlCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderUGVControlCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderUAVSetupCMDPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderACKPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderDemoPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderScriptPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderWaypointPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderNodePayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderFormationPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderGoalPayload(std::vector<uint8_t>& payload,DataFrame& codelessData);
    void coderAgentComputerStatusload(std::vector<uint8_t>& payload,DataFrame& codelessData);


    void decoderUAVStatePayload(std::vector<uint8_t>& dataFrame,DataFrame& state);//解码无人机状态Payload帧
    void decoderUGVStatePayload(std::vector<uint8_t>& dataFrame,DataFrame& state);//解码无人车状态Payload帧
    void decoderUAVControlPayload(std::vector<uint8_t>& dataFrame,DataFrame& control);//解码无人机控制Payload帧
    void decoderUGVControlPayload(std::vector<uint8_t>& dataFrame,DataFrame& control);//解码无人车控制Payload帧
    void decoderUAVSetupPayload(std::vector<uint8_t>& dataFrame,DataFrame& setup);//解码无人机设置指令Payload帧
    void decoderACKPayload(std::vector<uint8_t>& dataFrame,DataFrame& ack); //解码智能体应答Payload帧
    void decoderDemoPayload(std::vector<uint8_t>& dataFrame,DataFrame& demo); //解码智能体demo的Payload帧
    void decoderScriptPayload(std::vector<uint8_t>& dataFrame,DataFrame& script); //解码无人机script的Payload帧
    void decoderWaypointPayload(std::vector<uint8_t>& dataFrame,DataFrame& waypointData);//解码无人机航点Payload帧
    void decoderNodePayload(std::vector<uint8_t>& dataFrame,DataFrame& node); //解码智能体在线ROS节点Payload帧
    void decoderFormationPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct); //解码编队切换Payload帧
    void decoderGoalPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct); //解码规划点Payload帧
    void decoderAgentComputerStatusPayload(std::vector<uint8_t>& dataFrame,DataFrame& dataFrameStruct); //解码智能体电脑状态Payload帧


    uint64_t getTimestamp();//获得uint64_t类型的时间戳

    void floatArrayCopyToUint8tArray(std::vector<uint8_t>& data,std::vector<float>& value);//std::vector<float>数据加入std::vector<uint8_t>
    void floatCopyToUint8tArray(std::vector<uint8_t>& data,float& value);//float数据加入std::vector<uint8_t>
    void doubleCopyToUint8tArray(std::vector<uint8_t>& data,double& value);//double数据加入std::vector<uint8_t>
    void uint8tArrayToFloat(std::vector<uint8_t>& data, float& value);
    void uint8tArrayToDouble(std::vector<uint8_t>& data, double& value);

    uint16_t PackBytesLE( uint8_t high,uint8_t low);// 将两个8位无符号整数按小端字节序组合成一个16位无符号整数

    void safeConvertToUint32(size_t originalSize, uint32_t& convertedSize);//将size_t值安全转换为uint32_t的值

    uint16_t getChecksum(std::vector<uint8_t> data);//获取校验值

private:
    std::unordered_set<int> validMessageID={
        MessageID::HeartbeatMessageID,
        MessageID::UAVStateMessageID,
        MessageID::UGVStateMessageID,
        MessageID::NodeMessageID,
        MessageID::UAVControlCMDMessageID,
        MessageID::UAVSetupMessageID,
        MessageID::WaypointMessageID,
        MessageID::UGVControlCMDMessageID,
        MessageID::SearchMessageID,
        MessageID::ACKMessageID,
        MessageID::DemoMessageID,
        MessageID::ScriptMessageID,
        MessageID::FormationMessageID,
        MessageID::GoalMessageID,
        MessageID::AgentComputerStatusMessageID,
        MessageID::GroundFormationMessageID,
    };

};

#endif // CODEC_H

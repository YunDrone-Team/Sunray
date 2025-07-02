#ifndef DECODERINTERFACEBASE_H
#define DECODERINTERFACEBASE_H
#include "Communication/MSG.h"

class DecoderInterfaceBase {
public:
    virtual bool decoder(std::vector<uint8_t> undecodedData,DataFrame& decoderData)=0;//要求传入的是一条完整的数据
    virtual uint16_t getChecksum(std::vector<uint8_t> data)= 0;//获取校验值
    virtual ~DecoderInterfaceBase() = default;
};

#endif // DECODERINTERFACEBASE_H

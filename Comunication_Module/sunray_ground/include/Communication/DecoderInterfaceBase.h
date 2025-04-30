#ifndef DECODERINTERFACEBASE_H
#define DECODERINTERFACEBASE_H
#include "Communication/MSG.h"

class DecoderInterfaceBase {
public:
    virtual bool decoder(std::vector<uint8_t> undecodedData,int& messageID,unionData& decoderData) = 0;
    virtual uint16_t getChecksum(std::vector<uint8_t> data)= 0;//获取校验值

    virtual ~DecoderInterfaceBase() = default;
};

#endif // DECODERINTERFACEBASE_H

#include <iostream>
#include "ground_msg.h"

int main() {
    // 创建一个Mode_Message实例
    Mode_Message mode_msg;
    mode_msg.head = 23;
    mode_msg.length = 15;
    mode_msg.msg_id = 103;
    mode_msg.robot_id = 1;
    mode_msg.check = 0;
    mode_msg.payload.time_stamp = 123456789;
    mode_msg.payload.uav_mode = 2;

    // 计算checksum
    mode_msg.check = calculate_checksum_Mode_Message(&mode_msg);

    // 打包Mode_Message
    char* packed_msg = pack_Mode_Message(mode_msg.head, mode_msg.length, mode_msg.msg_id, mode_msg.robot_id, mode_msg.check, mode_msg.payload);

    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;

    extract_message_header(packed_msg, &head, &length, &msg_id, &robot_id, &check);
    std::cout << "head: " << head << " length: " << length << " msg_id: " << static_cast<int>(msg_id) << " robot_id: " << static_cast<int>(robot_id) << " check: " << check << std::endl;

    // 释放内存
    delete[] packed_msg;

    return 0;
}

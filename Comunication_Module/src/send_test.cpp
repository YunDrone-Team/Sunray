#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "ground_msg.h"
#include <unistd.h>

int main() {
    // Create a Mode_Message
    Mode_Message mode_msg;
    mode_msg.head = 0xADFF;
    mode_msg.length = 15;
    mode_msg.msg_id = 103;
    mode_msg.robot_id = 1;
    mode_msg.payload.time_stamp = 1234567;
    mode_msg.payload.uav_mode = 1;

    // 计算checksum
    mode_msg.check = calculate_checksum_Mode_Message(&mode_msg);

    // 打包Mode_Message
    char* packed_msg = pack_Mode_Message(mode_msg.head, mode_msg.length, mode_msg.msg_id, mode_msg.robot_id, mode_msg.check, mode_msg.payload);
    std::cout<<strlen(packed_msg)<<std::endl;
    // Create a socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // Set server address
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8969); // Replace with actual port
    if (inet_pton(AF_INET, "192.168.0.15", &(server_addr.sin_addr)) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return 1;
    }

    // Connect to server
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        return 1;
    }

    // Send the Mode_Message
    // char* encoded_msg = encode_Mode_Message(&mode_msg);
    uint16_t head;
    uint32_t length;
    uint8_t msg_id;
    uint8_t robot_id;
    uint16_t check;

    extract_message_header(packed_msg, &head, &length, &msg_id, &robot_id, &check);
    std::cout << "head: " << head << " length: " << length << " msg_id: " << static_cast<int>(msg_id) << " robot_id: " << static_cast<int>(robot_id) << " check: " << check << std::endl;

    std::cout<<"len:"<<sizeof(packed_msg)<<std::endl;
    if (send(sock, packed_msg, 1024, 0) < 0) {
        std::cerr << "Send failed" << std::endl;
        return 1;
    }

    // Close the socket
    close(sock);

    return 0;
}
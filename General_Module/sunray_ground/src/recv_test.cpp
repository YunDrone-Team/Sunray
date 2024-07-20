#include <boost/asio.hpp>
#include <iostream>
#include <array>
#include "ground_message.h"

using namespace boost::asio;
using namespace boost::asio::ip;


Message parseMessage_(char* buffer) {
    Message msg;
    memcpy(&msg, buffer, sizeof(Message));
    return msg;
}

int main() {
    // 创建IO服务对象
    boost::asio::io_service io_service;
    boost::asio::io_context io_context;
    // 创建socket对象
    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 9696));

    // 准备接收数据的缓冲区
    // std::array<char, 1024> recv_buf;
    // uint8_t recv_buf[1024];
    char buffer_[1024];
    boost::system::error_code error;
 
    // 接收数据
    boost::asio::ip::udp::endpoint remote_endpoint;

    while (true)
    {
        std::size_t recv_len = socket.receive_from(boost::asio::buffer(buffer_), remote_endpoint, 0, error);
        // 检查是否有错误发生
        if (!error) {
            char* buffer = new char[recv_len];
            std::copy(buffer_, buffer_ + recv_len, buffer);
            Message msg = parseMessage_(buffer);
            uint16_t check_sum = calculateMessageCheck(msg);
            std::cout << "head: " <<msg.head<< std::endl;
            std::cout << "length: " << msg.length << std::endl;
            std::cout << "msg_id: " << static_cast<int>(msg.msg_id) << std::endl;
            std::cout << "robot_id: " << static_cast<int>(msg.robot_id)<< std::endl;
            std::cout << "msg_mode: " << static_cast<int>(msg.msg_mode) << std::endl;
            std::cout << "check: " << check_sum << std::endl;
            switch (static_cast<int>(msg.msg_mode)) {
                case static_cast<int>(MessageType::TAKEOFFMSG):
                    std::cout << "  Takeoff message: " << std::endl;
                    std::cout << "    Time stamp: " << msg.payload.takeoff.time_stamp << std::endl;
                    std::cout << "    Takeoff: " << static_cast<int>(msg.payload.takeoff.takeoff) << std::endl;
                    break;
                case static_cast<int>(MessageType::MODEMSG):
                    std::cout << "  Mode message: " << std::endl;
                    std::cout << "    Time stamp: " << msg.payload.mode.time_stamp << std::endl;
                    std::cout << "    UAV mode: " << static_cast<int>(msg.payload.mode.uav_mode) << std::endl;
                    break;
                case static_cast<int>(MessageType::CONTROLMSG):
                    std::cout << "  Control message: " << std::endl;
                    std::cout << "    Time stamp: " << msg.payload.control.time_stamp << std::endl;
                    std::cout << "    Type: " << msg.payload.control.type << std::endl;
                    std::cout << "    X: " << msg.payload.control.x << std::endl;
                    std::cout << "    Y: " << msg.payload.control.y << std::endl;
                    std::cout << "    Z: " << msg.payload.control.z << std::endl;
                    // ...
                    break;
                default:
                    break;
            }
        replyMessage reply_msg;
        reply_msg.head = 64818;
        reply_msg.length = sizeof(reply_msg);
        reply_msg.msg_id = msg.msg_id;
        reply_msg.robot_id = msg.robot_id;
        reply_msg.msg_mode = msg.msg_mode;
        reply_msg.check = calculateReplyMessageCheck(reply_msg); 
        char send_buf[sizeof(reply_msg)];
        std::cout<<"head: "<< reply_msg.head<<std::endl;
        std::cout<<"length: "<<reply_msg.length<<std::endl;
        std::cout<<"msg_id: "<<static_cast<int>(reply_msg.msg_id)<<std::endl;
        std::cout<<"robot_id: "<<static_cast<int>(reply_msg.robot_id)<<std::endl;
        std::cout<<"msg_mode: "<<static_cast<int>(reply_msg.msg_mode)<<std::endl;
        std::cout<<"check: "<<reply_msg.check<<std::endl;
        memcpy(send_buf, &reply_msg, reply_msg.length);
        std::cout << "Received datagram from " << remote_endpoint.address() << ":" << remote_endpoint.port() << std::endl;
        udp::endpoint target(remote_endpoint.address(), remote_endpoint.port());
        socket.send_to(boost::asio::buffer(send_buf), target);
        // replyMessage reply_msg_1;
        // memcpy(&reply_msg_1, send_buf, sizeof(replyMessage));
        // std::cout<<"####: "<< reply_msg.head<<std::endl;
        // std::cout<<"head: "<< reply_msg.head<<std::endl;
        // std::cout<<"length: "<<reply_msg.length<<std::endl;
        // std::cout<<"msg_id: "<<static_cast<int>(reply_msg.msg_id)<<std::endl;
        // std::cout<<"robot_id: "<<static_cast<int>(reply_msg.robot_id)<<std::endl;
        // std::cout<<"msg_mode: "<<static_cast<int>(reply_msg.msg_mode)<<std::endl;
        // std::cout<<"check: "<<reply_msg.check<<std::endl;
        }else {
            std::cerr << "Error receiving message: " << error.message() << std::endl;
        }
    }
 
    return 0;
}
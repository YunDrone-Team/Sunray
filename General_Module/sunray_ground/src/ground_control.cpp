#include <boost/asio.hpp>
#include <iostream>
#include <cstdint>

#include "ground_message.h"
#include "ros_msg_utils.h"

using namespace std;
using namespace boost::asio;
using namespace boost::asio::ip;

struct Command_Type {
    static const uint8_t Hover = 0;
    static const uint8_t XYZ_POS = 1;
    static const uint8_t XYZ_VEL = 2;
    static const uint8_t XY_VEL_Z_POS = 3;
    static const uint8_t XYZ_ATT = 4;
    static const uint8_t TRAJECTORY = 5;
    static const uint8_t LAT_LON_ALT = 6;
};

class GroundControl {
public:

    ros::Publisher cmd_pub;
    ros::Publisher setup_pub;
    GroundControl(int argc, char **argv) : io_service_(), socket_(io_service_, udp::endpoint(udp::v4(), 9696)) {
        ros::init(argc, argv, "ground_control_node");
        ros::NodeHandle nh("~");
    }

    void start() {
        receiveLoop();
    }

private:
    void receiveLoop() {
        char buffer_[1024];
        boost::system::error_code error;
        udp::endpoint remote_endpoint;

        while (true) {
            std::size_t recv_len = socket_.receive_from(boost::asio::buffer(buffer_), remote_endpoint, 0, error);
            if (!error) {
                char* buffer = new char[recv_len];
                std::copy(buffer_, buffer_ + recv_len, buffer);
                Message msg = parseMessage_(buffer);
                uint16_t check_sum = calculateMessageCheck(msg);
                processMessage(msg);
                delete[] buffer;
            } else {
                std::cerr << "Error receiving message: " << error.message() << std::endl;
            }
        }
    }

    void processMessage(const Message& msg) {
        switch (static_cast<int>(msg.msg_mode)) {
            case static_cast<int>(MessageType::TAKEOFFMSG):
                //...
                std::cout << "  Takeoff message: " << std::endl;
                std::cout << "    Time stamp: " << msg.payload.takeoff.time_stamp << std::endl;
                std::cout << "    Takeoff: " << msg.payload.takeoff.takeoff << std::endl;
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
                //...
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
        memcpy(send_buf, &reply_msg, reply_msg.length);

        std::cout << "Received datagram from " << remote_endpoint_.address() << ":" << remote_endpoint_.port() << std::endl;
        udp::endpoint target(remote_endpoint_.address(), remote_endpoint_.port());
        socket_.send_to(boost::asio::buffer(send_buf), target);
    }

    Message parseMessage_(char* buffer) {
        Message msg;
        memcpy(&msg, buffer, sizeof(Message));
        return msg;
    }

    io_service io_service_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
};

int main(int argc, char **argv) {
    GroundControl node(argc, argv);
    node.start();
    return 0;
}
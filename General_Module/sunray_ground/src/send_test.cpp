#include <boost/asio.hpp>
#include <string>
#include <iostream>
// #include "ground_message.h"
#include "sunray_msg_ground.h"
#include "sunray_connect.hpp"

int main(int argc, char **argv) {
    sunray_socket socket;

    // Initialize TCP socket
    socket.initTCPSocket("192.168.0.29", 8080);
    socket.bindTCPSocket();
    socket.listenTCP();

    // Initialize UDP socket
    socket.initUDPSocket("192.168.0.29", 8081);
    socket.bindUDPSocket();
    socket.listenUDP();

    // Send a TCP message
    socket.sendTCPMessage("Hello, TCP!");

    // Send a UDP message
    socket.sendUDPMesssage("Hello, UDP!");

    // Receive and print TCP messages
    std::thread receiveTCPThread([&socket]() {
        while (true) {
            char *message;
            {
                std::lock_guard<std::mutex> lock(socket.tcpMessageQueueMutex);
                if (!socket.tcpMessageQueue.empty()) {
                    message = socket.tcpMessageQueue.front();
                    socket.tcpMessageQueue.pop();
                } else {
                    continue;
                }
            }
            std::cout << "Received TCP message: " << message << std::endl;
            // socket.sendTCPMessage("Hello, TCP!");
            delete[] message;
        }
    });

    // Receive and print UDP messages
    std::thread receiveUDPThread([&socket]() {
        while (true) {
            char *message;
            {
                std::lock_guard<std::mutex> lock(socket.udpMessageQueueMutex);
                if (!socket.udpMessageQueue.empty()) {
                    message = socket.udpMessageQueue.front();
                    socket.udpMessageQueue.pop();
                } else {
                    continue;
                }
            }
            std::cout << "Received UDP message: " << message << std::endl;
            // socket.sendUDPMesssage("Hello, UDP!");
            delete[] message;
        }
    });

    // Wait for 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Join threads
    receiveTCPThread.join();
    receiveUDPThread.join();

    return 0;
}
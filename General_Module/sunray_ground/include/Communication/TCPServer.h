#ifndef TCPSERVER_H
#define TCPSERVER_H

#include "CommunicationTCPSocket.h"


class TCPServer : public CommunicationTCPSocket
{
public:
    TCPServer();
    ~TCPServer();
    boost::signals2::signal<void(ReceivedParameter)> sigTCPServerReadData;

    bool TCPServerOnRun();
    void setRunState(bool state);
    void readResultDisposal(int result,char* readData,SOCKET sock,fd_set& temp, std::string ip);//读取结果处理
    void allSendData(std::vector<char> data);

    void TCPServerManagingData(std::vector<uint8_t>& data,std::string IP);
    bool resetMaxSock();

    std::vector<std::string> waitClearIP;


    fd_set fdRead,fdTemp;//描述符（socket） 集合 file descriptor set
    SocketIP backSock;
    SOCKET maxSock;
    std::atomic<bool> threadState;
    std::atomic<bool> runState;

};

#endif // TCPSERVER_H

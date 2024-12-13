#ifndef TCPCLIENT_H
#define TCPCLIENT_H
#include "CommunicationTCPSocket.h"

#include <mutex>

#define ClientNumber 20

struct ConnectStr
{
    unsigned short ListeningPort;
    std::string TargetIP;
    unsigned short TargetPort;
    bool State;
};


class TCPClient
{
public:
    boost::signals2::signal<void(ReceivedParameter)> sigAllTCPClientReadData;

    boost::signals2::signal<void(ConnectStr)> sigCreateTCPClientResult;


    TCPClient();
    ~TCPClient();
    void createClient(unsigned short listeningPort,std::string ip=nullptr,unsigned short targetPort=9696);
    void setRunState(int state);

    int  clientSendTCPData(std::vector<uint8_t> sendData,std::string targetIp);      //服务端发送数据接口,目的ip

    CommunicationTCPSocket* client[ClientNumber]={nullptr};
    void createThread();
    void onRun();
    std::atomic<bool> threadState;
    std::atomic<bool> runState;
    std::atomic<bool> createThreadState;
    std::atomic<bool> createThreadRun;

    std::vector<CommunicationTCPSocket*> clientVector;
    std::vector<ConnectStr> WaitConnectVector;

    std::mutex mutexConnect;
    std::mutex mutexRun;


};

#endif // TCPCLIENT_H

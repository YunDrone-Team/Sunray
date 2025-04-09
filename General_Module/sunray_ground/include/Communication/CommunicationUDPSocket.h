#ifndef COMMUNICATIONUDPSOCKET_H
#define COMMUNICATIONUDPSOCKET_H

#include<thread>
#include<mutex>
#include<string.h>
#include <boost/signals2.hpp>
#include <vector>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <fstream>
#include <fcntl.h>
#include "Communication/Cell.h"
#include "Communication/MSG.h"
#include "Communication/Codec.h"

class CommunicationUDPSocket
{
public:
    static CommunicationUDPSocket * getInstance();              //获取一个实例
     boost::signals2::signal<void(std::vector<uint8_t>, std::string)> sigReadData;
     boost::signals2::signal<void(ReceivedParameter)> sigUDPUnicastReadData;

     boost::signals2::signal<void(int)> sigUDPError;


    bool InitSocket();                                        //初始化Socket
    int Bind(unsigned short port=9898);                         //绑定监听端口号
    int BindSingleSocketToNetworkCardAndPort(SOCKET tempSock,std::string networkCardIp,unsigned short port);
    bool bindSocketToInterface(int sockfd, const char* interfaceName);

    //接收数据，要读取的Socket,buffer缓冲区，bufferSize缓冲区大小，返回值是成功读取到的数据大小，小于0读取错误
    //souIp是源IP地址，linuxPort是在Linux下源端口，winPort是在Windows下的源端口
    int ReadData(SOCKET Sock,char* buffer,int bufferSize,std::string& ip,uint16_t* linuxPort=nullptr,unsigned short* winPort=nullptr);

    //循环接收数据
    void OnRun();

    //发送数据接口,souIp是源IP地址，linuxPort是在Linux下源端口，winPort是在Windows下的源端口
    int sendUDPData(std::vector<uint8_t> sendData,std::string targetIp=nullptr,uint16_t targetPort=9898);              //发送数据接口
    int sendUDPBroadcastData(std::vector<uint8_t> sendData,uint16_t targetPort=9898);
    int sendUDPMulticastData(std::vector<uint8_t> sendData,uint16_t targetPort=9898);

    void setUDPReadState(bool state);   //设置UDP是否循环读取

//    void sendUDPData(std::vector<char> sendData,std::string targetIp,unsigned short targetPort);

    void setRunState(bool state);
    int findStdVectorComponent(uint8_t a,uint8_t b,std::vector<uint8_t> Data);

    void Close(SOCKET tempSock);
    SOCKET SocketConfiguration(SOCKET tempSock);
    ~CommunicationUDPSocket();
private:
    CommunicationUDPSocket();

    std::vector<NetworkInterface> getNetworkInterfaces();// 遍历网卡函数
    bool isLinuxInterfaceActive(const std::string& interfaceName); // 检查 Linux 网卡是否活动

    std::string wstringToString(const std::wstring& wstr);// 宽字符转窄字符函数

    void UDPUnicastManagingData(std::vector<uint8_t>& data,std::string IP,uint16_t port);

    void ResetFdRead(fd_set& fdRead );

    std::vector<uint8_t> UDPUnicastCacheData;//UDPUnicast缓存数据，用于缓存数据未收全的情况
    bool HandleUdpSocketReadEvent(SOCKET tempSock,fd_set& fdRead);
    bool resetMaxSock();

    int SendDataToMulticastTarget(SOCKET tempSock, std::vector<uint8_t> sendData,uint16_t targetPort);
    int SendDataToTarget(SOCKET tempSock, const std::vector<uint8_t> sendData, std::string targetIp, uint16_t targetPort);

    static CommunicationUDPSocket* CommunicationPtr;
    std::atomic<bool> UDPReadState;
    std::atomic<bool> runState;
    std::unordered_map<std::string, SOCKET> ipSocketMap;

    std::mutex mutexSocket;

    char* _buffer;
    int _bufferSize;
    char* _souIp;
    uint16_t* _linuxPort;
    unsigned short* _winPort;
    Codec codec;
    std::string multicastIP;
    SOCKET maxSock=INVALID_SOCKET;
    SOCKET defaultSock=INVALID_SOCKET;

};

#endif // COMMUNICATIONUDPSOCKET_H

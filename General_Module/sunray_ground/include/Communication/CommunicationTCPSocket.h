#ifndef COMMUNICATIONTCPSOCKET_H
#define COMMUNICATIONTCPSOCKET_H

#include<thread>
#include<mutex>
#include <boost/signals2.hpp>
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream>
#include <fcntl.h>


#include "Communication/Cell.h"
#include "Communication/MSG.h"
#include "Communication/Codec.h"


//单例设计取消
class CommunicationTCPSocket
{
public:

//    static CommunicationTCPSocket * getInstance();                            //获取一个实例

    std::weak_ptr<CommunicationTCPSocket> next;
    CommunicationTCPSocket();


    boost::signals2::signal<void(std::vector<uint8_t>, std::string)> sigReadData;

    boost::signals2::signal<void(ReceivedParameter)> sigTCPClientReadData;


    SOCKET InitSocket();                                                     //初始化Socket
    int Bind(unsigned short port=9696,const char* ip=nullptr);              //绑定IP和端口号
    int Listen(int n);                                                       //监听socket号，n是套接字排队的最大连接数

    //接受客户端连接,linuxPort是在Linux下连接的源端口，winPort是在Windows下连接的源端口
    SocketIP Accept(uint16_t* linuxPort=nullptr,unsigned short* winPort=nullptr);

    //接收数据，要读取的Socket,buffer缓冲区，bufferSize缓冲区大小，返回值是成功读取到的数据大小，小于0读取错误
    int ReadData(SOCKET Sock,char* buffer,int bufferSize);
    void readAllLink();//停用

    int  Connect(const char* ip,unsigned short port=9696);                   //连接服务器
    int  sendTCPData(std::vector<uint8_t> sendData,std::string targetIp);      //服务端发送数据接口,目的ip
    void Close();                                                            //关闭套接字

    int writeData(SOCKET Sock,std::vector<uint8_t> data);

    bool TCPClientOnRun();
    std::string getTCPClientTargetIP();
    SOCKET  getTCPClientSocket();


    int findStdVectorComponent(uint8_t a,uint8_t b,std::vector<uint8_t> Data);
    int setSocketNonblocking(SOCKET sockfd);
    int setSocketBlocking(SOCKET sockfd);


    ~CommunicationTCPSocket();
//private:
//    CommunicationTCPSocket();
//    static CommunicationTCPSocket* CommunicationPtr;
    std::unordered_map<std::string, SOCKET> ipSocketMap;

    SOCKET _sock;
    std::string connectIP;
    Codec codec;

    std::unordered_map<SOCKET,std::vector<uint8_t>> TCPServerCacheDataMap;


private:
    /*
    数据解析,通信()接收的数据都由这个函数解析,通过函数指针指向这个函数使用;
    四个参数都是引用,
    参数1(data):接收到的数据;

    */
    void TCPClientManagingData(std::vector<uint8_t>& data,std::string IP);


    std::vector<uint8_t> TCPClientCacheData;//TCPClient缓存数据，用于缓存数据未收全的情况


//    std::atomic<bool> threadState;
};

#endif // COMMUNICATIONTCPSOCKET_H

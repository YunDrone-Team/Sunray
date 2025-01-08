#include "CommunicationUDPSocket.h"

CommunicationUDPSocket* CommunicationUDPSocket::CommunicationPtr=nullptr;


CommunicationUDPSocket * CommunicationUDPSocket::getInstance()              //获取一个实例
{
    if(CommunicationUDPSocket::CommunicationPtr == nullptr)
        CommunicationUDPSocket::CommunicationPtr =new CommunicationUDPSocket();
    return CommunicationUDPSocket::CommunicationPtr;
}


CommunicationUDPSocket::CommunicationUDPSocket()
{
     _sock = INVALID_SOCKET;
     UDPReadState=true;
     runState=false;
     multicastIP="224.1.1.1";
     //线程
     std::thread t(std::mem_fn(&CommunicationUDPSocket::OnRun), this);
     t.detach();
}

SOCKET CommunicationUDPSocket::InitSocket()                                        //初始化Socket
{
    std::cout << "CommunicationUDPSocket::InitSocket() "<<std::endl;
#ifdef _WIN32
        //启动Windows socket 2.x环境
        WORD ver = MAKEWORD(2, 2);
        WSADATA dat;
        WSAStartup(ver, &dat);
#endif

#ifndef _WIN32
        //if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
        //	return (1);
        //忽略异常信号，默认情况会导致进程终止
//        signal(SIGPIPE, SIG_IGN);
#endif

        if (INVALID_SOCKET != _sock)
        {
            Close();
        }
//        _sock = socket(AF_INET, SOCK_STREAM, IPPROTO_UDP);
        _sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (INVALID_SOCKET == _sock)
        {
            //套接字错误
            std::cout << "UDP套接字错误 "<<_sock<<std::endl;
            sigUDPError(errno);

        }else {
            //套接字正常
            std::cout << "UDP套接字正常 "<<_sock<<std::endl;
            /*设置SO_REUSEADDR SO_REUSEADDR套接字选项允许在同一本地地址和端口上启动监听套接字，SO_REUSEPORT
             * 即使之前的套接字仍在TIME_WAIT状态。这对于快速重启服务器特别有用，因为它可以避免等待TIME_WAIT状态结束。*/
            int reuse = 1;
            if (setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0)
            {
                perror("setsockopt");
                // 处理错误
            }

            //广播配置
            int broadcastEnable = 1;
            int result = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
            if (result < 0) {
                perror("setsockopt(SO_BROADCAST) failed");
                close(_sock);
                exit(EXIT_FAILURE);
            }

            // 添加组播设置
            struct ip_mreq mreq;
            // 组播组地址
            inet_pton(AF_INET, multicastIP.c_str(), &mreq.imr_multiaddr);
            // 本地接口地址，使用INADDR_ANY表示任意接口
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);

            if (setsockopt(_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq)) < 0) {
                perror("setsockopt for multicast");
                // 处理错误
            }

            // 禁止组播环回
            const int loop = 0;
            if (setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0) {
                perror("setsockopt for disabling multicast loop");
            }
        }
        std::cout << "CommunicationUDPSocket::InitSocket() end"<<std::endl;
        return _sock;
}

int CommunicationUDPSocket::Bind(unsigned short port)                               //绑定监听端口号
{
//    if (INVALID_SOCKET == _sock)
        InitSocket();

    if(_sock==INVALID_SOCKET)
        return SOCKET_ERROR;

    sockaddr_in _sin= {};

    _sin.sin_family = AF_INET;
    _sin.sin_port = htons(port);

#ifdef _WIN32
    _sin.sin_addr.S_un.S_addr = htonl(INADDR_ANY);//INADDR_ANY;
#else
    _sin.sin_addr.s_addr = htonl(INADDR_ANY);
    //        _sin.sin_addr.s_addr = INADDR_ANY;

#endif

    int ret = bind(_sock, (sockaddr*)&_sin, sizeof(_sin));
    if (SOCKET_ERROR == ret)
    {
        //绑定端口号失败，端口号占用之类的原因
        std::cout << "UDP绑定端口号失败 "<<_sock<<" "<<port<<std::endl;
        perror("bind failed");
        sigUDPError(errno);

    }else {
        //绑定端口号成功
        std::cout << "UDP绑定端口号成功 "<<_sock<<std::endl;

    }
    return ret;
}

CommunicationUDPSocket::~CommunicationUDPSocket()
{
    Close();
#ifdef _WIN32
    //清除Windows socket环境
    WSACleanup();
#else

#endif

}

int CommunicationUDPSocket::findStdVectorComponent(uint8_t a,uint8_t b,std::vector<uint8_t> Data)
{
    auto it = std::find(Data.begin(), Data.end(), a);

    if (it != Data.end() && (it + 1) != Data.end() && *(it + 1) == b)
        return std::distance(Data.begin(), it);

    return -1;
}


void CommunicationUDPSocket::UDPUnicastManagingData(std::vector<uint8_t>& data,std::string IP,uint16_t port)
{
    //std::cout << "UDP准备解码数据 "<<std::endl;

    std::vector<uint8_t> copyData=data;
    while(true)
    {
//        std::cout << "UDP数据大小 "<<copyData.size()<<std::endl;
        if(copyData.size()<19)
            break;
        int index=findStdVectorComponent(0Xab,0X65,copyData);
        if(index<0)
            index=findStdVectorComponent(0Xad,0X21,copyData);
        if(index<0)
            index=findStdVectorComponent(0Xfd,0X32,copyData);
         //std::cout << "UDP查找帧头 "<<index<<std::endl;
        if( index>=0 )
        {
//           std::cout << "udp读取到的数据nLen "<<nLen<<" "<<static_cast<unsigned int>(hexValue)<<" "<<static_cast<unsigned int>(two)<<std::endl;

            // 定义要复制的范围（例如，从索引2到索引5，但不包括索引5）
            auto start = copyData.begin() + index+2;
            auto end = copyData.begin() + index+6;

            // 创建新的vector，并复制指定范围的数据
            std::vector<uint8_t> sizeVector(start, end);
            uint32_t size=0;

            // 注意：这里假设 coderData 是以小端字节序存储的（最低有效字节在前）
            for (int i = 0; i < 4; ++i)
                size |= static_cast<uint32_t>(sizeVector[i]) << (i * 8);

            //std::cout << "UDP数据长度 "<<size<<" "<<"实际数据长度 "<<data.size()<<std::endl;

            if(data.size()<size)
                break;

            start = copyData.begin() + index;
            end = copyData.begin() + index+size-2;

            std::vector<uint8_t> waitCheckVector(start, end);
            //std::cout << "UDP准备计算校验和数据的大小： "<<waitCheckVector.size()<<std::endl;

            uint16_t checksum =codec.getChecksum(waitCheckVector);

            uint16_t BackChecksum = static_cast<uint16_t>(copyData[index+size-1]) |
                    (static_cast<uint16_t>(copyData[index +size-2]) << 8);

            if(checksum!=BackChecksum)
            {
                //std::cout << "UDP校验和错误 计算的校验和： "<<checksum<<" "<<" 接收到校验和 "<<BackChecksum<<" 下标 "<<index+size-2<<" 校验和第一个字节 "<<(int)copyData[index+size-2]<<" 校验和第二个字节 "<<(int)copyData[index+size-1]<<std::endl;

                //清除帧头
                copyData.erase(copyData.begin()+index, copyData.begin() + 6);//待测试
                data.erase(data.begin()+index, data.begin() + 6);
                continue;
            }

            ReceivedParameter readData;
            readData.communicationType=CommunicationType::UDPUnicastCommunicationType;

            codec.decoder(std::vector<uint8_t>(data.begin()+index,data.begin()+index+size),readData.messageID,readData.data);
            //清除已处理数据
            data.erase(data.begin()+index, data.begin() +index+size);
            copyData.erase(copyData.begin()+index, copyData.begin()+index+size);//待测试
            if(readData.messageID==MessageID::SearchMessageID)
                readData.communicationType=CommunicationType::UDPBroadcastCommunicationType;
            readData.port=port;
            readData.ip=IP;
            sigUDPUnicastReadData(readData);
//            std::cout <<std::endl;
//            std::cout << "float 测试数据"<<std::endl;
//            std::vector<uint8_t> test;
//            float fl=2;
//            codec.floatCopyToUint8tArray(test,fl);
//            for (const auto& elem : test) {
//                // 创建一个临时的 ostringstream 来格式化数字
//                        std::ostringstream oss;
//                        // 设置输出为16进制，并设置每个数字的宽度为2，用0填充
//                        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(elem);
//                        // 将格式化的字符串输出到标准输出流
//                        std::cout << oss.str() << " ";
//               }
//            std::cout <<std::endl;
//            float two;
//            codec.uint8tArrayToFloat(test,two);
//            std::cout << "解码后float 数据 "<<two<<std::endl;

            //std::cout << "UDP数据处理完成，发送处理好的数据"<<std::endl;

            if(data.size()<19)
                break;

            continue;

        }else{
            data.clear();
            break;
        }
    }

}


void CommunicationUDPSocket::OnRun()
{
    while (UDPReadState)
    {
        if(!runState)
            continue;

        //伯克利套接字 BSD socket
        fd_set fdRead;//描述符（socket） 集合
        //清理集合
        FD_ZERO(&fdRead);
        //将描述符（socket）加入集合
        FD_SET(_sock, &fdRead);
        ///nfds 是一个整数值 是指fd_set集合中所有描述符(socket)的范围，而不是数量
        ///既是所有文件描述符最大值+1 在Windows中这个参数可以写0
        timeval t = { 0, 1};
        int ret = select(_sock + 1, &fdRead, 0, 0, &t); //linux后期改epoll
        if (ret < 0)
        {
            //select出问题了
            break;
        }else if (ret==0) {
            //没有数据进来
            continue;
        }else{
            //判断描述符（socket）是否在集合中
            if (FD_ISSET(_sock, &fdRead))
            {
                FD_CLR(_sock, &fdRead);
                char szRecv[4096] = {};
                std::string ip;
                uint16_t port;
                int len=ReadData(_sock,szRecv,4096,ip,&port);
                if(len<0)
                {
                    //拿到数据后？
                    Close();
                    UDPReadState=false;
                    break;

                }else if(len==0){
#ifdef _WIN32
                    //closesocket(sock);
#else
                    //这一块处理不确定，待测试
                    Close();
                    UDPReadState=false;
                    break;
#endif
                }else{
//                    std::string str(szRecv); // 使用C风格字符串初始化std::string
//                    std::vector<char> data(str.begin(), str.end());
//                    std::string ip(souIp);
//                    sigReadData(data,ip);

                    //同时接收不同ip的udp数据可能导致错误数据
                    std::vector<uint8_t> data(szRecv, szRecv + len);

                    //std::cout << "信号 来源ip "<<ip<<std::endl;
                    UDPUnicastCacheData.insert(UDPUnicastCacheData.end(), data.begin(), data.end());
                    UDPUnicastManagingData(UDPUnicastCacheData,ip,port);

                }

            }
        }
    }
    std::cout << "UDP线程结束"<<std::endl;

}


int CommunicationUDPSocket::ReadData(SOCKET Sock,char* buffer,int bufferSize,std::string& ip,uint16_t* linuxPort,unsigned short* winPort)
{

    struct sockaddr_in client_addr={};
    unsigned int addrlen = sizeof(client_addr);


    //bzero(buffer, bufferSize);
    //memset(buffer, sizeof(abuffer));
    int nLen = recvfrom(Sock, buffer, (size_t)bufferSize, 0,(struct sockaddr *)&client_addr,&addrlen);

#ifdef _WIN32
    if(winPort!=nullptr)
        *winPort=client_addr.sin_port;//这句待测
#else
    if(linuxPort!=nullptr)
        *linuxPort=client_addr.sin_port;//这句待测
#endif

    char* souIp=inet_ntoa(client_addr.sin_addr);
    ip=souIp;

    if(nLen>0)
    {
//        std::string str(buffer); // 使用C风格字符串初始化std::string
//        std::vector<uint8_t> data(str.begin(), str.end());
//        std::string ip(inet_ntoa(client_addr.sin_addr));
//        sigReadData(data,ip);

//        uint8_t hexValue=*buffer;
//        uint8_t  two=*(buffer+1);
        //std::cout << "udp读取到的数据nLen "<<nLen<<" "<<static_cast<unsigned int>(hexValue)<<" "<<static_cast<unsigned int>(two)<<std::endl;
        //std::cout << "udp读取到的数据 来源ip "<<souIp<<std::endl;
    }else {
        std::cout << "udp读取数据失败 "<<_sock<<" "<<nLen<<std::endl;
        sigUDPError(errno);
    }

    return nLen;
}

void CommunicationUDPSocket::setRunState(bool state)
{
    runState=state;
}



void CommunicationUDPSocket::setUDPReadState(bool state)    //设置UDP是否循环读取
{
    UDPReadState=state;
}

//void CommunicationUDPSocket::sendUDPData(std::vector<char> sendData,std::string targetIp,unsigned short targetPort)
//{
//    sockaddr_in _sin = {};
//    _sin.sin_family = AF_INET;
//    _sin.sin_port = htons(targetPort);
//    _sin.sin_addr.s_addr = inet_addr(targetIp.c_str());

//    if(_sock!=INVALID_SOCKET)
//        sendto(_sock,sendData.data(),sendData.size(),MSG_CONFIRM,(sockaddr*)&_sin,sizeof (_sin));
//}


int CommunicationUDPSocket::sendUDPBroadcastData(std::vector<uint8_t> sendData,uint16_t targetPort)
{
//    int broadcastEnable = 1;
//    int result = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
//       if (result < 0) {
//           perror("setsockopt(SO_BROADCAST) failed");
//           close(_sock);
//           exit(EXIT_FAILURE);
//       }
//    std::cout << "udp targetPort  "<<targetPort<<std::endl;

    return sendUDPData(sendData,"255.255.255.255",targetPort);
}

int CommunicationUDPSocket::sendUDPMulticastData(std::vector<uint8_t> sendData,uint16_t targetPort)
{
    int sendResult = 0;
    if ( INVALID_SOCKET != _sock && sendData.data() != nullptr && sendData.size() >0)
    {
        struct sockaddr_in target_addr={};
        target_addr.sin_family= AF_INET;
        target_addr.sin_port= htons(targetPort);
#ifdef _WIN32
    target_addr.sin_addr.S_un.S_addr = inet_addr(targetIp);
#else
    target_addr.sin_addr.s_addr = inet_addr(multicastIP.c_str());
#endif
        unsigned int addrlen = sizeof(target_addr);
        //发送数据
        sendResult = (int)sendto(_sock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
    }
    if(sendResult<0)
    {
        perror("Multicast sendto failed");
        sigUDPError(errno);
    }
    return sendResult;
}


int CommunicationUDPSocket::sendUDPData(std::vector<uint8_t> sendData,std::string targetIp,uint16_t targetPort)              //发送数据接口
{
    int sendResult = 0;
    if ( INVALID_SOCKET != _sock && sendData.data() != nullptr && sendData.size() >0)
    {
        struct sockaddr_in target_addr={};
        target_addr.sin_family= AF_INET;
        target_addr.sin_port= htons(targetPort);
#ifdef _WIN32
    target_addr.sin_addr.S_un.S_addr = inet_addr(targetIp);
#else
    target_addr.sin_addr.s_addr = inet_addr(targetIp.c_str());
#endif

        unsigned int addrlen = sizeof(target_addr);
        //发送数据
        sendResult = (int)sendto(_sock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
    }
    if(sendResult<0)
    {
        perror("sendto failed");
        sigUDPError(errno);
    }
    return sendResult;
}

void CommunicationUDPSocket::Close()
{
    std::cout << "CommunicationUDPSocket::Close() "<<std::endl;

    if (_sock != INVALID_SOCKET)
    {

        //关闭套节字socket
#ifdef _WIN32
        closesocket(_sock);

#else
        close(_sock);
#endif
        _sock = INVALID_SOCKET;
    }
}

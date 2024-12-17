#include "CommunicationTCPSocket.h"


//CommunicationTCPSocket* CommunicationTCPSocket::CommunicationPtr=nullptr;

//CommunicationTCPSocket * CommunicationTCPSocket::CommunicationTCPSocket::getInstance()  //获取一个实例
//{
//    if(CommunicationTCPSocket::CommunicationPtr == nullptr)
//        CommunicationTCPSocket::CommunicationPtr =new CommunicationTCPSocket();
//    return CommunicationTCPSocket::CommunicationPtr;
//}


CommunicationTCPSocket::CommunicationTCPSocket()
{
    _sock = INVALID_SOCKET;
    ipSocketMap.clear();
    connectIP.clear();
//    threadState=true;


}


CommunicationTCPSocket::~CommunicationTCPSocket()
{
    Close();
#ifdef _WIN32
    //清除Windows socket环境
    WSACleanup();
#else

#endif
//    threadState=false;

}


int CommunicationTCPSocket::setSocketNonblocking(SOCKET sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1)
        return -1;

    return fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

int CommunicationTCPSocket::setSocketBlocking(SOCKET sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1) {
        // Error retrieving the current flags
        perror("fcntl F_GETFL failed");
        return -1;
    }

    // Remove the O_NONBLOCK flag by using the bitwise NOT (~) operator
    // and the bitwise AND (&) operator
    flags &= ~O_NONBLOCK;

    // Set the new flags
    if (fcntl(sockfd, F_SETFL, flags) == -1) {
        // Error setting the new flags
        perror("fcntl F_SETFL failed");
        return -1;
    }

    return 0; // Success
}

SOCKET CommunicationTCPSocket::InitSocket() //初始化Socket
{
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
        signal(SIGPIPE, SIG_IGN);
#endif

        if (INVALID_SOCKET != _sock)
        {
            Close();
        }
        _sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (INVALID_SOCKET == _sock)
        {
            //套接字错误
            std::cout << "TCP初始化套接字错误 "<<_sock<<std::endl;
            sigTCPError(errno);
        }
        else {
            //套接字正常
            std::cout << "TCP初始化套接字正常 "<<_sock<<std::endl;
            /*设置SO_REUSEADDR SO_REUSEADDR套接字选项允许在同一本地地址和端口上启动监听套接字，
             * 即使之前的套接字仍在TIME_WAIT状态。这对于快速重启服务器特别有用，因为它可以避免等待TIME_WAIT状态结束。*/
            int reuse = 1;
            if (setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0)
            {
                perror("setsockopt");
                // 处理错误
                sigTCPError(errno);
            }

        }
        return _sock;

}

int CommunicationTCPSocket::Listen(int n)
{
    // listen函数用于将套接字设置为监听模式,n(backlog): 指定了系统应该为相应套接字排队的最大连接数。
    int ret = listen(_sock, n);
    if (SOCKET_ERROR == ret)
    {
        //监听网络端口失败
        std::cout << "TCP监听网络端口失败 "<<_sock<<std::endl;
        sigTCPError(errno);
    }
    else {
        //监听网络端口成功
        std::cout << "TCP监听网络端口成功 "<<_sock<<std::endl;

    }
    return ret;
}

SocketIP CommunicationTCPSocket::Accept(uint16_t* linuxPort,unsigned short* winPort)
{
    // 4 accept 等待接受客户端连接
    sockaddr_in clientAddr = {};
    int nAddrLen = sizeof(sockaddr_in);
    SOCKET cSock = INVALID_SOCKET;
    SocketIP back;
    back.socket=INVALID_SOCKET;
    back.IP.clear();
#ifdef _WIN32
    cSock = accept(_sock, (sockaddr*)&clientAddr, &nAddrLen);
    if(winPort!=nullptr)
        *winPort=clientAddr.sin_port;//这句待测
#else
    cSock = accept(_sock, (sockaddr*)&clientAddr, (socklen_t *)&nAddrLen);
    if(linuxPort!=nullptr)
        *linuxPort=clientAddr.sin_port;//这句待测
#endif
    std::string AccIp(inet_ntoa(clientAddr.sin_addr));
    if (INVALID_SOCKET == cSock)
    {
        std::cout << "TCP接收客户端链接失败 "<<cSock<<std::endl;
        sigTCPError(errno);

        //接收客户端链接失败
        auto it = ipSocketMap.find(AccIp);
        if (it != ipSocketMap.end()) {
            ipSocketMap.erase(AccIp);
        }
    }
    else
    {
        std::cout << "TCP接收客户端连接成功 "<<cSock<<std::endl;

       //接收客户端连接成功
        auto it = ipSocketMap.find(AccIp);
        if (it != ipSocketMap.end())
            ipSocketMap.erase(AccIp);


        ipSocketMap[std::string(AccIp)] = cSock;
        back.socket=cSock;
        back.IP=std::string(AccIp);

        std::vector<uint8_t> temp;
        TCPServerCacheDataMap[cSock]=temp;
    }
    return back;
}


int CommunicationTCPSocket::writeData(SOCKET Sock,std::vector<uint8_t> data)
{
    //这个接口不太合适，慎用，最好用另一个send
    int temp=write(Sock, data.data(), data.size());
    return temp;
}



int CommunicationTCPSocket::ReadData(SOCKET Sock,char* buffer,int bufferSize)
{

    //接收客户端数据
    int nLen = recv(Sock, buffer, bufferSize, 0);
    if(nLen>0)
    {
//        uint8_t hexValue=*buffer;
//        uint8_t  two=*(buffer+1);
        //std::cout << "TCP读取到的数据nLen "<<nLen<<" "<<static_cast<unsigned int>(hexValue)<<" "<<static_cast<unsigned int>(two)<<std::endl;
    }else if(nLen==0)
    {
        std::cout << "TCP读取数据0 "<<nLen<<std::endl;
        sigTCPError(errno);
//        perror("recv");
    }else{
         std::cout << "TCP读取数据失败 "<<nLen<<std::endl;
          perror("recv failed"); // 打印错误消息到stderr，并包含errno的值
          sigTCPError(errno);
    }
    return nLen;
}

std::string CommunicationTCPSocket::getTCPClientTargetIP()
{
    return connectIP;
}

SOCKET  CommunicationTCPSocket::getTCPClientSocket()
{
    return _sock;
}

int CommunicationTCPSocket::findStdVectorComponent(uint8_t a,uint8_t b,std::vector<uint8_t> Data)
{
    auto it = std::find(Data.begin(), Data.end(), a);

    if (it != Data.end() && (it + 1) != Data.end() && *(it + 1) == b)
        return std::distance(Data.begin(), it);

    return -1;
}



void CommunicationTCPSocket::TCPClientManagingData(std::vector<uint8_t>& data,std::string IP)
{
    std::cout << "TCPClient准备校验数据 "<<data.size()<<" ip: "<<IP<<std::endl;

    std::vector<uint8_t> copyData=data;
    while(true)
    {
        if(copyData.size()<19)
            break;
        int index=findStdVectorComponent(0Xac,0X43,copyData);
        std::cout << "查找TCPClient帧头 "<<index<<std::endl;

        if( index>=0 )
        {
            // 定义要复制的范围（例如，从索引2到索引5，但不包括索引5）
            auto start = copyData.begin() + index+2;
            auto end = copyData.begin() + index+6;

            // 创建新的vector，并复制指定范围的数据
            std::vector<uint8_t> sizeVector(start, end);
            uint32_t size=0;

            // 注意：这里假设 coderData 是以小端字节序存储的（最低有效字节在前）
            for (int i = 0; i < 4; ++i)
                size |= static_cast<uint32_t>(sizeVector[i]) << (i * 8);

            std::cout << "TCPClient数据长度 "<<size<<" "<<"实际数据长度 "<<data.size()<<std::endl;


            if(data.size()<size)
                break;

            start = copyData.begin() + index;
            end = copyData.begin() + index+size-2;

            std::vector<uint8_t> waitCheckVector(start, end);

            uint16_t checksum =codec.getChecksum(waitCheckVector);

            uint16_t BackChecksum = static_cast<uint16_t>(data[index+size-1]) |
                    (static_cast<uint16_t>(data[index +size-2]) << 8);

            if(checksum!=BackChecksum)
            {
                //清除帧头
                std::cout << "TCPClient校验和错误 计算的校验和： "<<checksum<<" "<<" 接收到校验和 "<<BackChecksum<<" 下标 "<<index+size-2<<" 校验和第一个字节 "<<(int)copyData[index+size-2]<<" 校验和第二个字节 "<<(int)copyData[index+size-1]<<std::endl;
                copyData.erase(copyData.begin()+index, copyData.begin() + 6);//待测
                data.erase(data.begin()+index, data.begin() + 6);
                continue;
            }

            ReceivedParameter readData;
            readData.communicationType=CommunicationType::TCPClientCommunicationType;

            codec.decoder(std::vector<uint8_t>(data.begin()+index,data.begin()+index+size),readData.messageID,readData.data);
            //清除已处理数据
            data.erase(data.begin()+index, data.begin() +index+size);
            copyData.erase(copyData.begin()+index, copyData.begin()+index+size);//待测


            readData.ip=IP;
            sigTCPClientReadData(readData);
            std::cout << "TCPClient接收数据正确，处理完成。： "<<std::endl;

            if(data.size()<19)
                break;
            continue;

        }else{
            data.clear();
            break;
        }
    }


}



bool CommunicationTCPSocket::TCPClientOnRun()
{
    fd_set fdRead;
    FD_ZERO(&fdRead);

    FD_SET(_sock, &fdRead);

    timeval t = { 0, 1};
    int ret = select(_sock + 1, &fdRead, 0, 0, &t); //linux后期改epoll
    if (ret < 0)
    {
        //select出问题了
        Close();
        sigTCPError(errno);
    }else if (ret ==0) {
        //超时，限定时间内没有IO操作
    }else{
        //判断描述符（socket）是否在集合中
        if (FD_ISSET(_sock, &fdRead))
        {
            char szRecv[4096] = {};
            int len=ReadData(_sock,szRecv,4096);
            if(len<0)
            {
                //错误处理,关闭对应的socket,清除储存的ip和socket的键值对
                FD_CLR(_sock, &fdRead);
#ifdef _WIN32
                closesocket(_sock);
#else
                sigTCPError(errno);
                close(_sock);
#endif
                //重连？
                //Connect(it->first.c_str());
                return false;
            }else if(len==0){
                //没有数据
                //                        continue;

                FD_CLR(_sock, &fdRead);
#ifdef _WIN32
                //closesocket(_sock);
#else
                sigTCPError(errno);
                close(_sock);
#endif
                return false;
            }else{
                //接收到数据了
                std::vector<uint8_t> data(szRecv, szRecv + len);
                std::string ip(connectIP);
                //sigReadData(data,ip);

                TCPClientCacheData.insert(TCPClientCacheData.end(), data.begin(), data.end());
                TCPClientManagingData(TCPClientCacheData,ip);

            }


        }

    }
    return true;
}



int  CommunicationTCPSocket::sendTCPData(std::vector<uint8_t> sendData,std::string targetIp)
{

    if(sendData.size()<=0)
        return -1;

    int sendResult = 0;
    SOCKET targetSocket=INVALID_SOCKET;
    //int totalSent = 0; // 总共已发送的字节数

    auto it = ipSocketMap.find(targetIp);
    if (it != ipSocketMap.end())
        targetSocket=it->second;

    if ( INVALID_SOCKET != targetSocket && sendData.data() != nullptr && sendData.size() >0)
    {
        //发送数据
        sendResult = send(targetSocket, sendData.data(), sendData.size(), 0);
        if(sendResult<0)
            sigTCPError(errno);

        /*数据分包发送，待启用*/
//        const int MAX_PACKET_SIZE = 1000; // 每个数据包的最大大小
//        int bytesLeft = dataSize; // 剩余要发送的字节数

//        // 循环发送数据，直到所有数据都发送完毕
//        while (bytesLeft > 0)
//        {
//            int bytesToSend = std::min(bytesLeft, MAX_PACKET_SIZE); // 本次要发送的字节数
//            int sendResult = send(targetSocket, sendData + totalSent, bytesToSend, 0);

//            if (sendResult == SOCKET_ERROR)
//            {
//                // 发送失败，可以根据errno获取更多信息

//            }

//            totalSent += sendResult; // 更新总共已发送的字节数
//            bytesLeft -= sendResult; // 更新剩余要发送的字节数
//        }

    }
    return sendResult;
}

int  CommunicationTCPSocket::Connect(const char* ip,unsigned short port)           //连接服务器
{
    std::cout << "CommunicationTCPSocket::Connect "<<ip<<std::endl;
    if(ip==nullptr)
        return SOCKET_ERROR;
    std::cout << "CommunicationTCPSocket::Connect 2 : "<<_sock<<std::endl;

    if (INVALID_SOCKET == _sock)
    {
        InitSocket();
    }
    // 2 连接服务器 connect
    sockaddr_in _sin = {};
    _sin.sin_family = AF_INET;
    _sin.sin_port = htons(port);
#ifdef _WIN32
    _sin.sin_addr.S_un.S_addr = inet_addr(ip);
#else
    _sin.sin_addr.s_addr = inet_addr(ip);
#endif
//    printf("<socket=%d>正在连接服务器<%s:%d>...\n", _sock, ip, port);
    std::cout << "正在连接服务器"<<std::endl;
    setSocketNonblocking(_sock);
    CommunicationState back;
    back.sock=_sock;
    back.ip=ip;
    back.port=port;
    int ret = connect(_sock, (sockaddr*)&_sin, sizeof(sockaddr_in));
    setSocketBlocking(_sock);
    if (SOCKET_ERROR == ret)
    {
//        printf("<socket=%d>错误，连接服务器<%s:%d>失败...\n",_sock, ip, port);
//        std::cout << "错误，连接服务器失败..."<<std::endl;
         perror("connect failed");
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(_sock, &write_fds);

        struct timeval timeout;
        timeout.tv_sec = 5; // 设置超时时间为 5 秒
        timeout.tv_usec = 0;

        ret = select(_sock + 1, NULL, &write_fds, NULL, &timeout);
        if(ret>0)
        {
            std::cout << "select连接服务器成功..."<<std::endl;
            back.state=TCPClientState::ConnectionSuccessful;
            sigTCPClientState(back);
            connectIP=ip;
        }else if (SOCKET_ERROR == ret){
            std::cout << "错误，连接服务器失败..."<<std::endl;
            back.state=TCPClientState::ConnectionFail;
            sigTCPClientState(back);

        }else if (ret==0) {
            std::cout << "错误，连接服务器超时..."<<std::endl;
            ret=SOCKET_ERROR;
            back.state=TCPClientState::ConnectionTimeout;
            sigTCPClientState(back);
        }
    }
    else {
//        printf("<socket=%d>连接服务器<%s:%d>成功...\n",_sock, ip, port);
        std::cout << "连接服务器成功..."<<std::endl;
        back.state=TCPClientState::ConnectionSuccessful;
        sigTCPClientState(back);
        connectIP=ip;
    }
    return ret;
}

//这个先不用
void CommunicationTCPSocket::readAllLink()
{
    for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
    {
           char szRecv[4096] = {};
           int len=ReadData(it->second,szRecv,4096);
           if(len<0)
           {
               //错误处理,关闭对应的socket,清除储存的ip和socket的键值对
#ifdef _WIN32
        closesocket(it->second);
#else
        close(it->second);
#endif
            //重连？
            //Connect(it->first.c_str());

            auto itDelete = ipSocketMap.find(it->first);
            if (it != ipSocketMap.end())
                ipSocketMap.erase(itDelete->first);

           }else if(len==0){
               //没有数据
               continue;
           }else{
               //接收到数据了
               std::string str(szRecv); // 使用C风格字符串初始化std::string
               std::vector<uint8_t> data(str.begin(), str.end());
               std::string ip(it->first);
               sigReadData(data,ip);

           }
    }
}



int CommunicationTCPSocket::Bind( unsigned short port,const char* ip) //绑定IP和端口号
{

    if (INVALID_SOCKET == _sock)
    {
        InitSocket();
    }
    // 2 bind 绑定用于接受客户端连接的网络端口
    sockaddr_in _sin = {};
    _sin.sin_family = AF_INET;
    _sin.sin_port = htons(port);//host to net unsigned short

#ifdef _WIN32
    if (ip){
        _sin.sin_addr.S_un.S_addr = inet_addr(ip);
    }
    else {
        _sin.sin_addr.S_un.S_addr = INADDR_ANY;
    }
#else
    if (ip!=nullptr) {
        _sin.sin_addr.s_addr = inet_addr(ip);
    }
    else {
        _sin.sin_addr.s_addr = INADDR_ANY;
    }
#endif
    int ret = bind(_sock, (sockaddr*)&_sin, sizeof(_sin));
    if (SOCKET_ERROR == ret)
    {
        //绑定端口号失败，端口号占用之类的原因
        std::cout << "TCP绑定端口号失败 "<<_sock<<" "<<port<<" "<<ip<<std::endl;
        sigTCPError(errno);
    }
    else {
        //绑定端口号成功
        std::cout << "TCP绑定端口号成功"<<std::endl;
    }
    return ret;
}


void CommunicationTCPSocket::Close()
{
    if (_sock != INVALID_SOCKET)
    {

        //关闭套节字socket
#ifdef _WIN32
        closesocket(_sock);       
#else
        close(_sock);
#endif
        _sock = INVALID_SOCKET;
        ipSocketMap.clear();

        TCPServerCacheDataMap.clear();
    }
}

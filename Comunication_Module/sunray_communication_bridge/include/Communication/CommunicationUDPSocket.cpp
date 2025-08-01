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
     maxSock=INVALID_SOCKET;
     UDPReadState=true;
     runState=false;
     multicastIP="224.1.1.1";
     //线程
     std::thread t(std::mem_fn(&CommunicationUDPSocket::OnRun), this);
     t.detach();
}

std::string CommunicationUDPSocket::wstringToString(const std::wstring& wstr)
{
#ifdef _WIN32
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], static_cast<int>(wstr.size()), NULL, 0, NULL, NULL);
    std::string strTo(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, &wstr[0], static_cast<int>(wstr.size()), &strTo[0], size_needed, NULL, NULL);
    return strTo;
#else
    return " ";
#endif
}

bool CommunicationUDPSocket::isLinuxInterfaceActive(const std::string& interfaceName)
{
    std::ifstream statusFile("/sys/class/net/" + interfaceName + "/operstate");
    std::string status;
    if (statusFile >> status) {
        return status == "up";
    }
    return false;
}

// 遍历网卡函数
std::vector<NetworkInterface> CommunicationUDPSocket::getNetworkInterfaces()
{
    std::vector<NetworkInterface> interfaces;

#ifdef _WIN32
    // Windows 实现
    ULONG outBufLen = 0;
    GetAdaptersAddresses(AF_INET, 0, NULL, NULL, &outBufLen);
    PIP_ADAPTER_ADDRESSES pAddresses = (PIP_ADAPTER_ADDRESSES) new BYTE[outBufLen];
    if (GetAdaptersAddresses(AF_INET, 0, NULL, pAddresses, &outBufLen) == NO_ERROR)
    {
        PIP_ADAPTER_ADDRESSES pCurrAddresses = pAddresses;
        while (pCurrAddresses)
        {
            PIP_ADAPTER_UNICAST_ADDRESS pUnicast = pCurrAddresses->FirstUnicastAddress;
            if (pUnicast != NULL && pCurrAddresses->OperStatus == IfOperStatusUp)
            {
                NetworkInterface iface;
                // 宽字符转窄字符
                iface.name = wstringToString(pCurrAddresses->FriendlyName);
                sockaddr_in* addr = (sockaddr_in*)pUnicast->Address.lpSockaddr;
                iface.ip = inet_ntoa(addr->sin_addr);

                // 检测网卡是否有效（排除回环地址为例）
                if (/*iface.ip != "127.0.0.1" &&*/!iface.name.empty())
                    interfaces.push_back(iface);

            }
            pCurrAddresses = pCurrAddresses->Next;
        }
    }
    delete[] pAddresses;
#else
    // Linux 实现 待测
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        return interfaces;
    }
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;
        if (ifa->ifa_addr->sa_family == AF_INET)
        {
            NetworkInterface iface;
            iface.name = std::string(ifa->ifa_name);
            struct sockaddr_in *sa = (struct sockaddr_in *)ifa->ifa_addr;
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sa->sin_addr), ip, INET_ADDRSTRLEN);
            iface.ip = std::string(ip);

            // 检查网卡是否活动
            if (isLinuxInterfaceActive(iface.name) && iface.ip != "127.0.0.1" &&!iface.name.empty())
            {
                interfaces.push_back(iface);
            }
        }
    }
    freeifaddrs(ifaddr);
#endif
    return interfaces;

}


bool CommunicationUDPSocket::InitSocket()                                        //初始化Socket
{

//    std::cout << "CommunicationUDPSocket::InitSocket() "<<std::endl;
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
//        std::vector<NetworkInterface> interfaces;
        std::vector<NetworkInterface> interfaces = getNetworkInterfaces();
        for (const auto& iface : interfaces)
        {
//            std::cout << "Interface: " << iface.name << ", IP: " << iface.ip << std::endl;
            std::lock_guard<std::mutex> lock(mutexSocket);
            auto it = ipSocketMap.find(iface.ip);
            if (it != ipSocketMap.end())
            {
                Close(it->second);
            }
            SOCKET tempSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (INVALID_SOCKET == tempSock)
            {
                //套接字错误
                std::cout<< "UDP Socket Error! Socket:"<<tempSock<<std::endl;
                sigUDPError(errno);

            }else {
                tempSock=SocketConfiguration(tempSock);
                if(tempSock!= INVALID_SOCKET)
                {
                    ipSocketMap[iface.ip]=tempSock;
#ifdef _WIN32
#else
                    bindSocketToInterface(tempSock,iface.name.c_str());
#endif
                }
            }
        }

        if(interfaces.size()<=0)
        {
            if(defaultSock!=INVALID_SOCKET)
                Close(defaultSock);
            defaultSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (INVALID_SOCKET == defaultSock)
            {
                //套接字错误
                std::cout<< "UDP Socket Error! Socket:"<<defaultSock<<std::endl;
                sigUDPError(errno);

            }else {
                defaultSock=SocketConfiguration(defaultSock);

            }
        }

        if(ipSocketMap.size() >=1 )
            return true;
        return false;
}

// 绑定套接字到特定网络接口
bool CommunicationUDPSocket::bindSocketToInterface(int sockfd, const char* interfaceName) 
{
    std::cerr << "bindSocketToInterface " << interfaceName << std::endl;
#ifdef _WIN32
//    // Windows 下使用 WSAIoctl 进行接口绑定
//    ULONG interfaceIndex = 0;
//    MIB_IPINTERFACE_ROW interfaceRow = { 0 };
//    interfaceRow.Family = AF_INET;
//    GetIpInterfaceEntry(&interfaceRow);

//    // 获取接口索引
//    if (GetIfEntry(&interfaceRow) != NO_ERROR) {
//        std::cerr << "Failed to get interface index on Windows." << std::endl;
//        return false;
//    }

//    // 使用 WSAIoctl 绑定到特定接口
//    DWORD bytesReturned;
//    if (WSAIoctl(sockfd, SIO_ASSOCIATE_HANDLE_WITH_IF, &interfaceRow.InterfaceIndex, sizeof(interfaceRow.InterfaceIndex), NULL, 0, &bytesReturned, NULL, NULL) != 0) {
//        std::cerr << "Failed to bind socket to interface on Windows: " << WSAGetLastError() << std::endl;
//        return false;
//    }
#else
    // Linux 下使用 SO_BINDTODEVICE 选项
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interfaceName, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &ifr, sizeof(ifr)) == -1) {
        std::cerr << "Failed to bind socket to interface on Linux: " << strerror(errno) << std::endl;
        return false;
    }
#endif
    return true;
}

int CommunicationUDPSocket::BindSingleSocketToNetworkCardAndPort(SOCKET tempSock,std::string networkCardIp,unsigned short port)
{
    sockaddr_in _sin= {};

    _sin.sin_family = AF_INET;
    _sin.sin_port = htons(port);

#ifdef _WIN32
//    _sin.sin_addr.S_un.S_addr = htonl(INADDR_ANY);//INADDR_ANY; inet_addr(INADDR_ANY)
     _sin.sin_addr.S_un.S_addr = inet_addr(networkCardIp.c_str());
#else
     _sin.sin_addr.s_addr = htonl(INADDR_ANY);
    //        _sin.sin_addr.s_addr = INADDR_ANY;
    //  _sin.sin_addr.s_addr = inet_addr(networkCardIp.c_str());
#endif

    int ret = bind(tempSock, (sockaddr*)&_sin, sizeof(_sin));
    if (SOCKET_ERROR == ret)
    {
        //绑定端口号失败，端口号占用之类的原因
//        std::cout << "Failed to bind the UDP port number! Socket:"<<tempSock<<" port "<<port<<" networkCardIp "<<networkCardIp<<std::endl;
        std::cout << "Failed to bind UDP port!" << std::endl
                  << "  Socket: " << tempSock << std::endl
                  << "  Port: " << port << std::endl
                  << "  Network card IP: " << networkCardIp << std::endl;
        fprintf(stderr, "Bind failed: %s\n", strerror(errno));
        perror("bind failed");
//        sigUDPError(errno);
#ifdef _WIN32
   sigUDPError(WSAGetLastError());
#else
   sigUDPError(errno);
#endif
    }else {
        //绑定端口号成功
//        std::cout << "The UDP port number is bound successfully! Socket:"<<tempSock<<" port "<<port<<" return: "<<ret<<" networkCardIp "<<networkCardIp<<" "<<bool(inet_addr(networkCardIp.c_str())==INADDR_NONE)<<std::endl;
        std::cout << "UDP port bound successfully!" << std::endl
                  << "  Socket: " << tempSock << std::endl
                  << "  Port: " << port << std::endl
                  << "  Return value: " << ret << std::endl
                  << "  Network card IP: " << networkCardIp << std::endl
                  << "  IP validity: " << (inet_addr(networkCardIp.c_str()) == INADDR_NONE ? "Invalid" : "Valid")
                  << std::endl;
    }


    return ret;
}


int CommunicationUDPSocket::Bind(unsigned short port)                               //绑定监听端口号
{
    InitSocket();
    for (const auto& pair : ipSocketMap)
    {
        if(BindSingleSocketToNetworkCardAndPort(pair.second,pair.first,port)==SOCKET_ERROR)
            return SOCKET_ERROR;
//        if(maxSock<pair.second)
//            maxSock=pair.second;
    }

    if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
    {
        sockaddr_in _sin= {};

        _sin.sin_family = AF_INET;
        _sin.sin_port = htons(port);

#ifdef _WIN32
        _sin.sin_addr.S_un.S_addr = htonl(INADDR_ANY);//INADDR_ANY; inet_addr(INADDR_ANY)
#else
        _sin.sin_addr.s_addr = htonl(INADDR_ANY);
        //        _sin.sin_addr.s_addr = INADDR_ANY;
        //  _sin.sin_addr.s_addr = inet_addr(networkCardIp.c_str());
#endif

        int ret = bind(defaultSock, (sockaddr*)&_sin, sizeof(_sin));
        if (SOCKET_ERROR == ret)
        {
            //绑定端口号失败，端口号占用之类的原因
            std::cout << "defaultSock Failed to bind the UDP port number! Socket:"<<defaultSock<<" port "<<port<<std::endl;
            fprintf(stderr, "Bind failed: %s\n", strerror(errno));
            perror("bind failed");
            //        sigUDPError(errno);
#ifdef _WIN32
            sigUDPError(WSAGetLastError());
#else
            sigUDPError(errno);
#endif
        }else {
            //绑定端口号成功
            std::cout << "defaultSock The UDP port number is bound successfully! Socket:"<<defaultSock<<" port "<<port<<" return: "<<ret<<std::endl;

        }

    }

    return 0;
}

void CommunicationUDPSocket::setDecoderInterfacePtr(DecoderInterfaceBase* ptr)
{
    if(decoderInterfacePtr!=nullptr)
        delete decoderInterfacePtr;

    decoderInterfacePtr=ptr;
}


CommunicationUDPSocket::~CommunicationUDPSocket()
{

    for (const auto& pair : ipSocketMap)
        Close(pair.second);

    if(defaultSock !=INVALID_SOCKET)
        Close(defaultSock);

    if(decoderInterfacePtr!=nullptr)
        delete decoderInterfacePtr;

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
//    std::cout << "UDPUnicastManagingData "<<std::endl;

    std::vector<uint8_t> copyData=data;
    while(true)
    {
//        std::cout << "UDP data size "<<copyData.size()<<std::endl;
        if(copyData.size()<19)
            break;
        int index=findStdVectorComponent(0Xab,0X65,copyData);
        if(index<0)
            index=findStdVectorComponent(0Xad,0X21,copyData);
        if(index<0)
            index=findStdVectorComponent(0Xfd,0X32,copyData);
        if(index<0)
            index=findStdVectorComponent(0Xcc,0X90,copyData);
//        std::cout << "UDP find head "<<index<<std::endl;
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

//            std::cout << "size: "<<size<<" "<<"data.size(): "<<data.size()<<std::endl;

            if(data.size()<size)
                break;

            start = copyData.begin() + index;
            end = copyData.begin() + index+size-2;

            std::vector<uint8_t> waitCheckVector(start, end);
            //std::cout << "UDP准备计算校验和数据的大小： "<<waitCheckVector.size()<<std::endl;

//            uint16_t checksum =codec.getChecksum(waitCheckVector);
            uint16_t checksum=0;
            if(decoderInterfacePtr!=nullptr)
                checksum=decoderInterfacePtr->getChecksum(waitCheckVector);

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

//            codec.decoder(std::vector<uint8_t>(data.begin()+index,data.begin()+index+size),readData.messageID,readData.data);
            if(decoderInterfacePtr!=nullptr)
                decoderInterfacePtr->decoder(std::vector<uint8_t>(data.begin()+index,data.begin()+index+size),readData.dataFrame);

            //清除已处理数据
            data.erase(data.begin()+index, data.begin() +index+size);
            copyData.erase(copyData.begin()+index, copyData.begin()+index+size);//待测试
            if(readData.dataFrame.seq==MessageID::SearchMessageID)
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

void CommunicationUDPSocket::ResetFdRead(fd_set& fdRead)
{
    //清理集合
    // std::cout << "ResetFdRead: "<<ipSocketMap.size()<<std::endl;

    FD_ZERO(&fdRead);
    std::lock_guard<std::mutex> lock(mutexSocket);
    for (const auto& pair : ipSocketMap)
    {
        FD_SET(pair.second, &fdRead);
        if(maxSock<pair.second)
            maxSock=pair.second;
    }

    if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
    {
        FD_SET(defaultSock, &fdRead);
        maxSock=defaultSock;

    }
    //  std::cout << "ResetFdRead end maxSock:"<<maxSock<<std::endl;


}

bool CommunicationUDPSocket::HandleUdpSocketReadEvent(SOCKET tempSock,fd_set& fdRead)
{
    //判断描述符（socket）是否在集合中
    if (FD_ISSET(tempSock, &fdRead))
    {
        FD_CLR(tempSock, &fdRead);
        char szRecv[4096] = {};
        std::string ip;
        uint16_t port;
        int len=ReadData(tempSock,szRecv,4096,ip,&port);
        if(len<0)
        {
            Close(tempSock);
            UDPReadState=false;
            return false;

        }else if(len==0){
#ifdef _WIN32
            //closesocket(sock);
#else
            //这一块处理不确定，待测试
            Close(tempSock);
            UDPReadState=false;
            return false;
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
    return true;
}

bool CommunicationUDPSocket::resetMaxSock()
{
    std::vector<std::string> deleteVector;
    std::lock_guard<std::mutex> lock(mutexSocket);

    for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
    {
#ifdef _WIN32
        u_long mode;
        if (ioctlsocket(it->second, FIONBIO, &mode) == SOCKET_ERROR)
        {
            deleteVector.push_back(it->first);
        }
        else
        {
            if (maxSock < it->second || maxSock==INVALID_SOCKET )
            {
                maxSock = it->second;

            }
        }
#else
        if (fcntl(it->second, F_GETFL, 0) == -1)
        {
            deleteVector.push_back(it->first);
        }
        else
        {
            if (maxSock < it->second)
            {
                maxSock = it->second;
            }
        }
#endif
    }

    if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
    {
#ifdef _WIN32
        u_long mode;
        if (ioctlsocket(defaultSock, FIONBIO, &mode) == SOCKET_ERROR)
            defaultSock=INVALID_SOCKET;
        else
            maxSock = defaultSock;
#else
        if (fcntl(defaultSock, F_GETFL, 0) == -1)
            defaultSock=INVALID_SOCKET;
        else
            maxSock = defaultSock;
#endif
    }

    for (const auto &str : deleteVector)
    {
        auto itDelete = ipSocketMap.find(str);
        if (itDelete != ipSocketMap.end())
        {
            std::string clientIP = itDelete->first;
            ipSocketMap.erase(itDelete);


        }
    }
    deleteVector.clear();

//    std::cout << "maxSock "<<(maxSock!=INVALID_SOCKET)<<"  "<<maxSock<<" ! "<<INVALID_SOCKET<<std::endl;


    return true;
}


void CommunicationUDPSocket::OnRun()
{
    while (UDPReadState)
    {
        if(!runState)
            continue;

        //伯克利套接字 BSD socket
        fd_set fdRead;//描述符（socket） 集合
        ResetFdRead(fdRead);
        resetMaxSock();

        ///nfds 是一个整数值 是指fd_set集合中所有描述符(socket)的范围，而不是数量
        ///既是所有文件描述符最大值+1 在Windows中这个参数可以写0
        timeval t = { 0, 1};
        int ret=-1;
//        std::cout << "maxSock "<<(maxSock!=INVALID_SOCKET)<<"  "<<(int)maxSock<<" ! "<<INVALID_SOCKET<<std::endl;

        if(maxSock!=INVALID_SOCKET)
            ret = select(maxSock + 1, &fdRead, 0, 0, &t); //linux后期改epoll
        else
            continue;


        if (ret < 0)
        {
            //select出问题了
            std::cout << "select error"<<std::endl;

            resetMaxSock();
            ResetFdRead(fdRead);
            continue;

//            break;
        }else if (ret==0) {
            //没有数据进来
            continue;
        }else{
//            std::cout << "select in"<<std::endl;
            std::lock_guard<std::mutex> lock(mutexSocket);
            for (const auto& pair : ipSocketMap)
            {
                if( !HandleUdpSocketReadEvent(pair.second,fdRead) )
                    break;
            }
            if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
            {
                if( !HandleUdpSocketReadEvent(defaultSock,fdRead) )
                    break;
            }

        }
    }
    std::cout << "UDP thread termination!"<<std::endl;

}


int CommunicationUDPSocket::ReadData(SOCKET Sock,char* buffer,int bufferSize,std::string& ip,uint16_t* linuxPort,unsigned short* winPort)
{

    struct sockaddr_in client_addr={};
    //unsigned int addrlen = sizeof(client_addr);


    //bzero(buffer, bufferSize);
    //memset(buffer, sizeof(abuffer));
    //int nLen = recvfrom(Sock, buffer, (size_t)bufferSize, 0,(struct sockaddr *)&client_addr,&addrlen);

#ifdef _WIN32
    int addrlen = sizeof(client_addr);
    int nLen = recvfrom(Sock, buffer, (size_t)bufferSize, 0,(struct sockaddr *)&client_addr,&addrlen);
    if(winPort!=nullptr)
        *winPort=client_addr.sin_port;//这句待测
#else
    unsigned int addrlen = sizeof(client_addr);
    int nLen = recvfrom(Sock, buffer, (size_t)bufferSize, 0,(struct sockaddr *)&client_addr,&addrlen);
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
//        std::cout << "udp读取到的数据 来源ip "<<souIp<<std::endl;
    }else {
        std::cout << "udp读取数据失败 "<<" "<<nLen<<std::endl;
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
//    return sendUDPData(sendData,"192.168.31.168",targetPort);

}


int CommunicationUDPSocket::SendDataToMulticastTarget(SOCKET tempSock, std::vector<uint8_t> sendData,uint16_t targetPort)
{
    int sendResult = 0;

    if ( INVALID_SOCKET != tempSock && sendData.data() != nullptr && sendData.size() >0)
    {
        struct sockaddr_in target_addr={};
        target_addr.sin_family= AF_INET;
        target_addr.sin_port= htons(targetPort);
#ifdef _WIN32
    target_addr.sin_addr.S_un.S_addr = inet_addr(multicastIP.c_str());
    int addrlen = sizeof(target_addr);
           // 发送数据，将 unsigned char * 强制转换为 const char *
    sendResult = sendto(tempSock, reinterpret_cast<const char*>(sendData.data()), static_cast<int>(sendData.size()), 0, (struct sockaddr*)&target_addr, addrlen);
#else
    target_addr.sin_addr.s_addr = inet_addr(multicastIP.c_str());
    unsigned int addrlen = sizeof(target_addr);
    //发送数据
    sendResult = (int)sendto(tempSock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
#endif
//        unsigned int addrlen = sizeof(target_addr);
//        //发送数据
//        sendResult = (int)sendto(_sock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
    }
    return sendResult;

}


int CommunicationUDPSocket::sendUDPMulticastData(std::vector<uint8_t> sendData,uint16_t targetPort)
{
    int sendResult = 0;

    for (const auto& pair : ipSocketMap)
    {
        sendResult=SendDataToMulticastTarget(pair.second,sendData,targetPort);
        if(sendResult<0)
        {
            perror("Multicast sendto failed");
            sigUDPError(errno);
            return sendResult;
        }
    }

    if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
    {
        sendResult=SendDataToMulticastTarget(defaultSock,sendData,targetPort);
        if(sendResult<0)
        {
            perror("Multicast sendto failed");
            sigUDPError(errno);
            return sendResult;
        }
    }


    return sendResult;
}

int CommunicationUDPSocket::SendDataToTarget(SOCKET tempSock, const std::vector<uint8_t> sendData, std::string targetIp, uint16_t targetPort)
{
//    std::cout << "SendDataToTarget socket:  "<<tempSock<<" targetIp "<<targetIp<<" data.size() "<<sendData.size()<<std::endl;
    int sendResult = 0;
    if ( INVALID_SOCKET != tempSock && sendData.data() != nullptr && sendData.size() >0)
    {
        struct sockaddr_in target_addr={};
        target_addr.sin_family= AF_INET;
        target_addr.sin_port= htons(targetPort);
#ifdef _WIN32
    target_addr.sin_addr.S_un.S_addr = inet_addr(targetIp.c_str());
    int addrlen = sizeof(target_addr);
    sendResult = sendto(tempSock, reinterpret_cast<const char*>(sendData.data()), static_cast<int>(sendData.size()), 0,
                        reinterpret_cast<struct sockaddr*>(&target_addr), addrlen);
#else
    target_addr.sin_addr.s_addr = inet_addr(targetIp.c_str());
    unsigned int addrlen = sizeof(target_addr);
    //发送数据
    sendResult = (int)sendto(tempSock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
#endif

//        unsigned int addrlen = sizeof(target_addr);
//        //发送数据
//        sendResult = (int)sendto(_sock, sendData.data(), sendData.size(), 0,(struct sockaddr *)&target_addr,addrlen);
    }

    return sendResult;


}



int CommunicationUDPSocket::sendUDPData(std::vector<uint8_t> sendData,std::string targetIp,uint16_t targetPort)              //发送数据接口
{
    int sendResult = 0;
//    std::cout << "ipSocketMap size:  "<<ipSocketMap.size()<<std::endl;
    int temp;
    for (const auto& pair : ipSocketMap)
    {
        temp=SendDataToTarget(pair.second,sendData,targetIp,targetPort);
        if(temp>0)
            sendResult=temp;
//        if(sendResult<0)
//        {
//            perror("sendUDPData sendto failed");
////            std::cout << "WSAGetLastError:  "<<WSAGetLastError()<<std::endl;
//            sigUDPError(errno);
//        }
    }

    if(ipSocketMap.size()<=0 && defaultSock !=INVALID_SOCKET)
    {
        sendResult=SendDataToTarget(defaultSock,sendData,targetIp,targetPort);
        if(temp>0)
            sendResult=temp;
//        if(sendResult<0)
//        {
//            perror("sendUDPData sendto failed");
//            sigUDPError(errno);
//        }
    }


    return sendResult;
}

SOCKET CommunicationUDPSocket::UpdateMulticastConfiguration(SOCKET tempSock)
{
#ifdef _WIN32
    // 尝试退出组播
    struct ip_mreq mreqLeave;
    SOCKADDR_IN multicastAddrLeave;
    INT addrLenLeave = sizeof(multicastAddrLeave);

    std::string nonConstMulticastIPLeave = multicastIP;
    if (WSAStringToAddressA(&nonConstMulticastIPLeave[0], AF_INET, nullptr,
                           reinterpret_cast<sockaddr*>(&multicastAddrLeave), &addrLenLeave) == 0) {
        mreqLeave.imr_multiaddr.s_addr = multicastAddrLeave.sin_addr.s_addr;
        mreqLeave.imr_interface.s_addr = INADDR_ANY;

        // 尝试退出组播（即使之前未加入也不会出错）
        setsockopt(tempSock, IPPROTO_IP, IP_DROP_MEMBERSHIP,
                  reinterpret_cast<const char *>(&mreqLeave), sizeof(mreqLeave));
    }

    // 添加组播设置
    struct ip_mreq mreqn;
    // 使用 WSAStringToAddressA 来转换组播组地址
    SOCKADDR_IN multicastAddr;

    INT addrLen = sizeof(multicastAddr);
    std::string nonConstMulticastIP = multicastIP;  // 创建非 const 副本，因为 WSAStringToAddressA 第一个参数要求非 const
    if (WSAStringToAddressA(&nonConstMulticastIP[0], AF_INET, nullptr, reinterpret_cast<sockaddr*>(&multicastAddr), &addrLen) != 0)
    {
        std::cerr << "WSAStringToAddressA failed: " << WSAGetLastError() << std::endl;
        return INVALID_SOCKET;
    }
    mreqn.imr_multiaddr.s_addr = multicastAddr.sin_addr.s_addr;
    // 本地接口地址，使用 INADDR_ANY 表示任意接口
    mreqn.imr_interface.s_addr = INADDR_ANY;

    if (setsockopt(tempSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char *>(&mreqn), sizeof(mreqn)) < 0)
    {
        perror("setsockopt for multicast");
        // 处理错误
        return INVALID_SOCKET;
    }
    // 禁止组播环回
    const int loop = 0;
    if (setsockopt(tempSock, IPPROTO_IP, IP_MULTICAST_LOOP, reinterpret_cast<const char *>(&loop), sizeof(loop)) < 0)
    {
        perror("setsockopt for disabling multicast loop");
        return INVALID_SOCKET;
    }
#else
    // 尝试退出组播
    struct ip_mreq mreqLeave;
    inet_pton(AF_INET, multicastIP.c_str(), &mreqLeave.imr_multiaddr);
    mreqLeave.imr_interface.s_addr = htonl(INADDR_ANY);

    // 尝试退出组播（即使之前未加入也不会出错）
    setsockopt(tempSock, IPPROTO_IP, IP_DROP_MEMBERSHIP,
               (const char *)&mreqLeave, sizeof(mreqLeave));

    // 添加组播设置
    struct ip_mreq mreq;
    // 组播组地址
    inet_pton(AF_INET, multicastIP.c_str(), &mreq.imr_multiaddr);
    // 本地接口地址，使用INADDR_ANY表示任意接口
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(tempSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt for multicast");
        // 处理错误
        return INVALID_SOCKET;
    }

    // 禁止组播环回
    const int loop = 0;
    if (setsockopt(tempSock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0)
    {
        perror("setsockopt for disabling multicast loop");
        return INVALID_SOCKET;
    }
#endif
//    std::cout << "CommunicationUDPSocket::UpdateMulticastConfiguration end  "<<std::endl;
    return tempSock;
}


void CommunicationUDPSocket::UpdateMulticast()
{
    for (const auto& pair : ipSocketMap)
    {
        SOCKET socket = pair.second;
        UpdateMulticastConfiguration(socket);
    }
}


SOCKET CommunicationUDPSocket::SocketConfiguration(SOCKET tempSock)
{
    /*设置SO_REUSEADDR SO_REUSEADDR套接字选项允许在同一本地地址和端口上启动监听套接字，SO_REUSEPORT
     * 即使之前的套接字仍在TIME_WAIT状态。这对于快速重启服务器特别有用，因为它可以避免等待TIME_WAIT状态结束。
        UDP没有这个TIME_WAIT状态，该配置会导致端口被占用后还能绑定成功，但是功能不太正常，所以屏蔽这个配置，端口被占用后输出提示信号*/
    int reuse = 1;
    if (setsockopt(tempSock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0)
    {
        perror("setsockopt");
        // 处理错误
        return INVALID_SOCKET;
    }

#ifdef _WIN32
    //广播配置
    int broadcastEnable = 1;
    int result = setsockopt(tempSock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&broadcastEnable), sizeof(broadcastEnable));
    if (result < 0)
    {
        perror("setsockopt(SO_BROADCAST) failed");
        Close(tempSock);
        exit(EXIT_FAILURE);
        return INVALID_SOCKET;
    }


    // 添加组播设置
    struct ip_mreq mreqn;
    // 使用 WSAStringToAddressA 来转换组播组地址
    SOCKADDR_IN multicastAddr;

    INT addrLen = sizeof(multicastAddr);
    std::string nonConstMulticastIP = multicastIP;  // 创建非 const 副本，因为 WSAStringToAddressA 第一个参数要求非 const
    if (WSAStringToAddressA(&nonConstMulticastIP[0], AF_INET, nullptr, reinterpret_cast<sockaddr*>(&multicastAddr), &addrLen) != 0)
    {
        std::cerr << "WSAStringToAddressA failed: " << WSAGetLastError() << std::endl;
        return INVALID_SOCKET;
    }
    mreqn.imr_multiaddr.s_addr = multicastAddr.sin_addr.s_addr;
    // 本地接口地址，使用 INADDR_ANY 表示任意接口
    mreqn.imr_interface.s_addr = INADDR_ANY;

    if (setsockopt(tempSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char *>(&mreqn), sizeof(mreqn)) < 0)
    {
        perror("setsockopt for multicast");
        // 处理错误
        return INVALID_SOCKET;
    }
    // 禁止组播环回
    const int loop = 0;
    if (setsockopt(tempSock, IPPROTO_IP, IP_MULTICAST_LOOP, reinterpret_cast<const char *>(&loop), sizeof(loop)) < 0)
    {
        perror("setsockopt for disabling multicast loop");
        return INVALID_SOCKET;
    }

#else
    //广播配置
    int broadcastEnable = 1;
    int result = setsockopt(tempSock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (result < 0)
    {
        perror("setsockopt(SO_BROADCAST) failed");
        Close(tempSock);
        exit(EXIT_FAILURE);
        return INVALID_SOCKET;
    }

    // 添加组播设置
    struct ip_mreq mreq;
    // 组播组地址
    inet_pton(AF_INET, multicastIP.c_str(), &mreq.imr_multiaddr);
    // 本地接口地址，使用INADDR_ANY表示任意接口
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(tempSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt for multicast");
        // 处理错误
        return INVALID_SOCKET;
    }

    // 禁止组播环回
    const int loop = 0;
    if (setsockopt(tempSock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0)
    {
        perror("setsockopt for disabling multicast loop");
        return INVALID_SOCKET;
    }
#endif
    return tempSock;
}


void CommunicationUDPSocket::Close(SOCKET tempSock)
{
    std::cout << "CommunicationUDPSocket::Close() "<<std::endl;

    if (tempSock != INVALID_SOCKET)
    {

        //关闭套节字socket
#ifdef _WIN32
        closesocket(tempSock);

#else
        close(tempSock);
#endif
    }
}

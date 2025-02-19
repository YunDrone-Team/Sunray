#include "TCPServer.h"

TCPServer::TCPServer()
{
    InitSocket();
    threadState=true;
    runState=false;

    waitClearIP.clear();

    //新开一个线程!!
    std::thread workThreadPtr(std::mem_fn(&TCPServer::TCPServerOnRun), this);
    workThreadPtr.detach();



}

TCPServer::~TCPServer()
{
    Close();
    threadState=false;
}

void TCPServer::setRunState(bool state)
{
    FD_ZERO(&fdRead);
    FD_ZERO(&fdTemp);
    maxSock=_sock;
    FD_SET(maxSock, &fdTemp);

    runState=state;
}

void TCPServer::allSendData(std::vector<uint8_t> data)
{
    for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
    {
//        writeData(it->second,data);
//        send(it->second, data.data(), data.size(), 0);
        //发送结果没处理

#ifdef _WIN32
        send(it->second, reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size()), 0);
#else
        send(it->second, data.data(), data.size(), 0);
#endif
    }
}

void TCPServer::TCPServerManagingData(std::vector<uint8_t>& data,std::string IP)
{
    //std::cout << "准备校验数据 "<<data.size()<<" ip: "<<IP<<std::endl;
    std::vector<uint8_t> copyData=data;
    while(true)
    {
        if(copyData.size()<19)
            break;
        int index=findStdVectorComponent(0Xac,0X43,copyData);
        //std::cout << "查找TCPServer帧头 "<<index<<std::endl;

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

            //std::cout << "TCPServer数据长度 "<<size<<" "<<"实际数据长度 "<<data.size()<<std::endl;

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
                std::cout << "TCPServer校验和错误 计算的校验和： "<<checksum<<" "<<" 接收到校验和 "<<BackChecksum<<" 下标 "<<index+size-2<<" 校验和第一个字节 "<<(int)copyData[index+size-2]<<" 校验和第二个字节 "<<(int)copyData[index+size-1]<<std::endl;
                //清除帧头
                copyData.erase(copyData.begin()+index, copyData.begin() + 6);//待测试
                data.erase(data.begin()+index, data.begin() + 6);
                continue;
            }

            ReceivedParameter readData;
            readData.communicationType=CommunicationType::TCPServerCommunicationType;

            codec.decoder(std::vector<uint8_t>(data.begin()+index,data.begin()+index+size),readData.messageID,readData.data);
            //清除已处理数据
            data.erase(data.begin()+index, data.begin() +index+size);
            copyData.erase(copyData.begin() + index, copyData.begin() + + index + size); // 待测试


            readData.ip=IP;
            sigTCPServerReadData(readData);

            if(data.size()<19)
                break;
            continue;

        }else{
            data.clear();
            break;
        }
    }
}


void TCPServer::readResultDisposal(int result,char* readData,SOCKET sock,fd_set& temp,std::string ip)
{
//    std::cout << "TCPServer readResultDisposal 准备读取数据结果处理 "<<result<<std::endl;

    if(result<0)
    {
        //错误处理,关闭对应的socket,清除储存的ip和socket的键值对
        sigTCPServerError(errno);
        FD_CLR(sock,&temp);
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        //重连？
        //Connect(it->first.c_str());



        auto itDeleteCache = TCPServerCacheDataMap.find(sock);
        if (itDeleteCache  != TCPServerCacheDataMap.end())
            TCPServerCacheDataMap.erase(itDeleteCache->first);

//        FD_ZERO(&fdRead);
//        FD_ZERO(&fdTemp);
//        maxSock=_sock;
//        FD_SET(maxSock, &fdTemp);

//        FD_CLR(sock, &fdTemp);
//        for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
//        {
//            if(it->second>maxSock)
//                maxSock=it->second;
//            FD_SET(maxSock, &fdTemp);

//        }


    }else if(result==0){
//        std::cout << "读取数据结果为0 "<<result<<std::endl;

#ifdef _WIN32
        //closesocket(sock);
#else
        sigTCPServerError(errno);
        close(sock);
//        auto itDelete = ipSocketMap.find(ip);
//        if (itDelete  != ipSocketMap.end())
//            ipSocketMap.erase(itDelete->first);


        auto itDeleteCache = TCPServerCacheDataMap.find(sock);
        if (itDeleteCache  != TCPServerCacheDataMap.end())
            TCPServerCacheDataMap.erase(itDeleteCache->first);



#endif
//        std::cout << "读取数据结果为0 处理结束 "<<result<<std::endl;

        //没有数据
    }else{
        //接收到数据了
//        std::string str(readData); // 使用C风格字符串初始化std::string
//        std::vector<uint8_t> data(str.begin(), str.end());
//        sigReadData(data,ip);

        std::vector<uint8_t> data(readData, readData + result);
        //sigReadData(data,ip);

        std::vector<uint8_t> temp;
        auto itCache = TCPServerCacheDataMap.find(sock);
        if (itCache  != TCPServerCacheDataMap.end())
              temp=itCache->second;
        temp.insert(temp.end(), data.begin(), data.end());

        TCPServerManagingData(temp,ip);
        if (TCPServerCacheDataMap.find(sock)  != TCPServerCacheDataMap.end())
            TCPServerCacheDataMap[sock]=temp;

    }
//    std::cout << "TCPServer readResultDisposal 读取数据结果处理结束 "<<result<<std::endl;

}


bool TCPServer::resetMaxSock()
{
//    if(fcntl(_sock, F_GETFL, 0)==-1)
//        return false;
//    maxSock=_sock;
//    std::vector<std::string> deleteVector;
//    for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
//    {
//        if(fcntl(it->second, F_GETFL, 0)==-1)
//            deleteVector.push_back(it->first);
//        else
//        {
//            if(maxSock<it->second)
//                maxSock=it->second;
//        }
//    }

//    for (const auto& str : deleteVector)
//    {
//        auto itDelete = ipSocketMap.find(str);
//        if (itDelete  != ipSocketMap.end())
//            ipSocketMap.erase(itDelete->first);

//    }
//    deleteVector.clear();
//    FD_ZERO(&fdRead);
//    FD_ZERO(&fdTemp);
//    FD_SET(maxSock, &fdTemp);

//    return true;

    #ifdef _WIN32
            u_long mode;
            if (ioctlsocket(_sock, FIONBIO, &mode) == SOCKET_ERROR) {
                return false;
            }
    #else
            if (fcntl(_sock, F_GETFL, 0) == -1) {
                return false;
            }
    #endif
            maxSock = _sock;
            std::vector<std::string> deleteVector;
            for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it) {
    #ifdef _WIN32
                if (ioctlsocket(it->second, FIONBIO, &mode) == SOCKET_ERROR) {
                    deleteVector.push_back(it->first);
                } else {
                    if (maxSock < it->second) {
                        maxSock = it->second;
                    }
                }
    #else
                if (fcntl(it->second, F_GETFL, 0) == -1) {
                    deleteVector.push_back(it->first);
                } else {
                    if (maxSock < it->second) {
                        maxSock = it->second;
                    }
                }
    #endif
            }

            for (const auto& str : deleteVector) {
                auto itDelete = ipSocketMap.find(str);
                if (itDelete != ipSocketMap.end()) {
                    ipSocketMap.erase(itDelete);
                }
            }
            deleteVector.clear();
            FD_ZERO(&fdRead);
            FD_ZERO(&fdTemp);
            FD_SET(maxSock, &fdTemp);

            return true;

}



bool TCPServer::TCPServerOnRun()
{

    while(threadState)
    {
        if(!runState)
            continue;
//        std::cout << "TCPServerOnRun "<<_sock<<std::endl;
        timeval t = { 0, 1};
        fdRead=fdTemp;
        int ret = select(maxSock + 1, &fdRead, 0, 0, &t); //linux后期改epoll
        if (ret < 0)
        {
            //select出问题了
            sigTCPServerError(errno);
            std::cout << "select出问题了 "<<ret<<" "<<maxSock + 1<<std::endl;
            bool fl=false;
            if (errno == EBADF)
                 fl=resetMaxSock();

            if(fl)
                continue;

            Close();
            threadState=false;
            break;
        }else if (ret ==0) {
            //超时，限定时间内没有IO操作
           //std::cout << "超时，限定时间内没有IO操作 "<<_sock<<std::endl;
            continue;
        }else{
            //判断描述符（socket）是否在集合中
            //std::cout << "判断描述符（socket）是否在集合中 "<<FD_ISSET(_sock, &fdRead)<<" "<<_sock<<std::endl;

            if (FD_ISSET(_sock, &fdRead))
            {
                backSock=Accept();
                if(backSock.socket==INVALID_SOCKET)
                {
                    std::cout << "客户端链接返回值INVALID_SOCKET "<<std::endl;
                }else{
                    FD_SET(backSock.socket, &fdTemp);
                    if(backSock.socket>maxSock)
                        maxSock=backSock.socket;

                    //这里可能会导致重复收到两条数据，待测
                    char szRecv[4096] = {};
                    int len=ReadData(backSock.socket,szRecv,4096);
                    readResultDisposal(len,szRecv,backSock.socket,fdTemp,backSock.IP);

                }

            }

//            std::cout << "准备遍历socket连接，有无数据可读 "<<std::endl;
            //遍历socket连接，有无数据可读
            for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
            {
                //std::cout << "遍历socket连接，有无数据可读 "<<FD_ISSET(it->second, &fdRead)<<" "<<std::endl;

                if (FD_ISSET(it->second, &fdRead))
                {
//                    std::cout << "遍历socket连接，有数据可读 "<<FD_ISSET(it->second, &fdRead)<<" "<<it->second<<std::endl;

                    char szRecv[4096] = {};
                    int len=ReadData(it->second,szRecv,4096);

                    readResultDisposal(len,szRecv,it->second,fdTemp,it->first);

                    if(len<0)
                    {
                        waitClearIP.push_back(it->first);
                    }else if (len==0) {
#ifdef _WIN32

#else
                        waitClearIP.push_back(it->first);
#endif
                    }
                }
            }
            for (const auto& str : waitClearIP)
            {
//                std::cout << str << std::endl;
                auto itDelete = ipSocketMap.find(str);
                if (itDelete  != ipSocketMap.end())
                    ipSocketMap.erase(itDelete->first);
                FD_ZERO(&fdRead);
                FD_ZERO(&fdTemp);
                maxSock=_sock;
                FD_SET(maxSock, &fdTemp);

                for (auto it = ipSocketMap.begin(); it != ipSocketMap.end(); ++it)
                {
                    if(it->second>maxSock)
                        maxSock=it->second;
                    FD_SET(maxSock, &fdTemp);

                }
            }
            waitClearIP.clear();

        }
    }

    std::cout << "TCP线程结束"<<std::endl;

    return false;
}

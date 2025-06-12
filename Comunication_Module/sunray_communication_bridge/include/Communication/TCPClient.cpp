#include "TCPClient.h"

TCPClient::TCPClient()
{
    threadState=true;
    createThreadRun=true;
    //新开一个线程!!
    std::thread workThreadPtr(std::mem_fn(&TCPClient::onRun), this);
    workThreadPtr.detach();

    std::thread wcreateThreadPtr(std::mem_fn(&TCPClient::createThread), this);
    wcreateThreadPtr.detach();
}

TCPClient::~TCPClient()
{
//    for(int i=0;i<ClientNumber;++i)
//    {
//        if(client[i]!=nullptr)
//        {
//            client[i]->Close();
//            delete client[i];
//            client[i]=nullptr;
//        }
//    }
    std::cout << "TCPClient::~TCPClient()"<<std::endl;

    threadState=false;
    createThreadRun=false;
    std::cout << "TCPClient::~TCPClient() end"<<std::endl;

}



void TCPClient::onRun()
{
    int back;
    while (threadState)
    {
        if (!runState)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 当runState为false时休眠
            continue;
        }

        std::vector<CommunicationTCPSocket*> temp;

        std::unique_lock<std::mutex> lockRun(mutexRun);
        if (clientVector.empty())
        {
            lockRun.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 休眠100毫秒
            continue;
        }

        // 记录需要删除的元素
        for (const auto& item : clientVector)
        {
            back = item->TCPClientOnRun();
            if (!back)
                temp.push_back(item);
        }


        if (!temp.empty())
        {
            for (const auto& item : temp)
            {
                for(auto toDelete=clientVector.begin();toDelete != clientVector.end();)
                {
                    if(item==*toDelete)
                    {
                        if(*toDelete!=nullptr)
                        {
                            (*toDelete)->Close();
                            delete *toDelete;
                            *toDelete=nullptr;
                        }

                        toDelete=clientVector.erase(toDelete);
                    }else
                        ++toDelete;
                }
            }

            temp.clear();
        }
    }
    std::unique_lock<std::mutex> lockRun(mutexRun);

    for (auto item : clientVector)
    {
        if(item!=nullptr)
        {
            item->Close();
            delete item;
            item=nullptr;
        }

    }
    clientVector.clear();
    std::cout << "TCP Client thread termination!"<<std::endl;
}



void TCPClient::closeLine(std::string ip)
{

    std::vector<CommunicationTCPSocket*> temp;

    std::unique_lock<std::mutex> lockRun(mutexRun);

    if(clientVector.size()>0)
    {
        for (const auto& item : clientVector)
        {
            if(item->getTCPClientTargetIP()==ip)
            {
                item->Close();
                temp.push_back(item);
            }
        }
    }
//    lockRun.unlock();


    if(temp.size()>0)
    {

        for (const auto& itDelete : temp)
        {

//            std::unique_lock<std::mutex> lockDelete(mutexRun);

            auto it =std::find(clientVector.begin(), clientVector.end(), itDelete);
            if (it != clientVector.end())
            {
                // 删除元素之前，先释放内存（如果使用原始指针的话）
                delete *it; // 注意是delete *it，因为it是指向指针的指针
                clientVector.erase(it);
            }
//            lockDelete.unlock();

        }
        temp.clear();
    }
    lockRun.unlock();

}



void TCPClient::setRunState(int state)
{
    runState=state;
}

int  TCPClient::clientSendTCPData(std::vector<uint8_t> sendData,std::string targetIp)
{
    int sendResult = 0;
std::cout << "TCPClient::clientSendTCPData clientVector "<<clientVector.size()<<" "<<sendData.size()<<std::endl;
    if(sendData.size()<=0)
        return -1;

    for (const auto& item : clientVector)
    {
        std::cout << "targetIp "<<targetIp<<" item->getTCPClientTargetIP() "<<item->getTCPClientTargetIP()<<std::endl;

        if(targetIp!=item->getTCPClientTargetIP())
                   continue;

        if ( INVALID_SOCKET != item->getTCPClientSocket() && sendData.data() != nullptr && sendData.size() >0)
        {
//            sendResult = send(item->getTCPClientSocket(), sendData.data(), sendData.size(), 0);

#ifdef _WIN32
         sendResult = send(item->getTCPClientSocket(), reinterpret_cast<const char*>(sendData.data()), static_cast<int>(sendData.size()), 0);
#else
        sendResult = send(item->getTCPClientSocket(), sendData.data(), sendData.size(), 0);
#endif
            if(sendResult<=0)
            {
                perror("send failed"); // 使用perror打印错误消息，它会自动使用errno
                std::cout << "send failed "<<sendData.size()<<std::endl;
            }
            std::cout << "Socket: "<<item->getTCPClientSocket()<<" sendResult "<<sendResult<<std::endl;
        }

    }


    //int totalSent = 0; // 总共已发送的字节数
//    for(int i=0;i<ClientNumber;++i)
//    {
//        if(client[i]==nullptr)
//            continue;

//        if(targetIp!=client[i]->getTCPClientTargetIP())
//            continue;

//        if ( INVALID_SOCKET != client[i]->getTCPClientSocket() && sendData.data() != nullptr && sendData.size() >0)
//        {
//            //发送数据
//            sendResult = send(client[i]->getTCPClientSocket(), sendData.data(), sendData.size(), 0);
//            if(sendResult<=0)
//            {
//                perror("send failed"); // 使用perror打印错误消息，它会自动使用errno
//                std::cout << "send failed "<<sendData.size()<<std::endl;


//            }
//        }
//    }


    std::cout << "return sendResult "<<std::endl;
    return sendResult;
}

void TCPClient::createThread()
{
    while (createThreadRun)
    {

        std::unique_lock<std::mutex> lock(mutexConnect);
        if(WaitConnectVector.size()>0)
        {
            for (const auto& item : WaitConnectVector)
            {
                ConnectStr temp=item;
                CommunicationTCPSocket* TCPSocket=new CommunicationTCPSocket;
                TCPSocket->InitSocket();
                TCPSocket->setDecoderInterfacePtr(temp.decoderInterfacePtr);
                int back;
//                 back=TCPSocket->Bind(temp.ListeningPort);
//                std::cout << "绑定端口号back "<<back<< std::endl;
//                if(back==SOCKET_ERROR)
//                {
//                    std::cout << "绑定端口号失败 back==SOCKET_ERROR "<<std::endl;
//                    TCPSocket->Close();
//                    delete TCPSocket;
//                    TCPSocket=nullptr;
//                    temp.State=false;
//                    sigCreateTCPClientResult(temp);
//                    continue;
//                }
                back=TCPSocket->Connect(temp.TargetIP.data(),temp.TargetPort);
                if(back==SOCKET_ERROR)
                {
                    std::cout << "连接服务器失败 "<<std::endl;
                    TCPSocket->Close();
                    delete TCPSocket;
                    TCPSocket=nullptr;
                    temp.State=false;
                    sigCreateTCPClientResult(temp);
                    continue;
                }
                TCPSocket->sigTCPClientReadData.connect([this](const ReceivedParameter& param)
                {
                    // 将接收到的参数传递给 sigAllTCPClientReadData
                    sigAllTCPClientReadData(param);
                });
                std::string ip=temp.TargetIP;
                TCPSocket->sigTCPError.connect([this,ip](const int& error)
                {
                    // 将接收到的参数传递给 sigTCPClientError
                    TCPClientErrorStr temp;
                    temp.error=error;
                    temp.targetIP=ip;
                    sigTCPClientError(temp);
                });

                 std::unique_lock<std::mutex> lockRun(mutexRun);
                 if(TCPSocket!=nullptr)
                    clientVector.push_back(TCPSocket);
                 lockRun.unlock();

                 temp.State=true;
                 sigCreateTCPClientResult(temp);

                //std::cout << "创建客户端成功 "<<back<<std::endl;


            }
            WaitConnectVector.clear();
            lock.unlock();


        }else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 休眠5200毫秒
            continue;
        }

    }

    std::cout << "TCP Client createThread termination!"<<std::endl;

}



void TCPClient::createClient(unsigned short listeningPort,DecoderInterfaceBase* ptr,std::string ip,unsigned short targetPort)
{   //重新连接服务器没处理

        //std::cout << "TCPClient::createClient "<<std::endl;
        ConnectStr connectData;
        connectData.ListeningPort=listeningPort;
        connectData.TargetIP=ip;
        connectData.TargetPort=targetPort;
        connectData.decoderInterfacePtr=ptr;
        connectData.State=false;

        std::unique_lock<std::mutex> lock(mutexConnect);
        WaitConnectVector.push_back(connectData);
        lock.unlock();
        //std::cout << "TCPClient::createClient end"<<std::endl;


//       for(int i=0;i<ClientNumber;++i)
//       {

//           if(client[i]==nullptr)
//           {
//               client[i]=new CommunicationTCPSocket;
//               client[i]->InitSocket();
//               int back=client[i]->Bind(listeningPort);
//               std::cout << "绑定端口号back "<<back<< std::endl;
//               if(back==SOCKET_ERROR)
//               {
//                   std::cout << "绑定端口号失败 back==SOCKET_ERROR "<<std::endl;
//                   client[i]->Close();
//                   delete client[i];
//                   client[i]=nullptr;
//                   return false;
//               }
//               back=client[i]->Connect(ip.data(),targetPort);
//               if(back==SOCKET_ERROR)
//               {
//                   client[i]->Close();
//                   delete client[i];
//                   client[i]=nullptr;
//                   return false;
//               }
//               client[i]->sigTCPClientReadData.connect([this](const ReceivedParameter& param)
//               {
//                   // 将接收到的参数传递给 sigAllTCPClientReadData
//                   sigAllTCPClientReadData(param);
//               });
//               break;
//           }
//           if(i==ClientNumber-1 &&client[i]!=nullptr)
//               return false;
//       }
//    return true;
}


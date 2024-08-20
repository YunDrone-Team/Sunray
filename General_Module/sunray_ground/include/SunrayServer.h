// #ifndef TcpServer
// #define TcpServer

#include <string>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>

class TcpServer
{
public:
    TcpServer(std::string strIp, unsigned int uPort);
    virtual ~TcpServer();

    // 初始化网络服务端
    bool InitServer();

    // 监听客户端连接
    bool WaitForClient();

    // 发送数据
    bool SendMsg(const std::string &strMsg);

    // 接收数据并打印
    bool RecvMsg();

    // 收到的消息往队列中存放
    void ListenAndStoreMsg();

    // 开启一个线程用来监听消息
    void startListening();

    // 开启一个线程用来等待客户端连接
    void startWaitForClient();

    // 获取队列中的数据
    char *GetMsg();

    // 检查队列是否为空
    bool HasMsg();

    // 停止服务端
    bool Stop();

    unsigned int m_uPort; // 监听端口
    std::string m_strIp;  // 用于监听本机指定IP地址

private:
    int m_listenSocket = -1; // 监听套接字
    int m_clientSocket = -1; // 客户端套接字

    bool m_bRunning; // 是否正在运行
    bool stop_flag;  // 是否停止

    std::queue<char *> m_msgQueue; // 消息队列
    std::mutex m_mutex;            // 互斥锁
};

TcpServer::TcpServer(std::string strIp = "localhost", unsigned int uPort = 8888) : m_strIp(strIp),
                                                                                         m_uPort(uPort)
{
    m_bRunning = false;
    stop_flag = false;
}

TcpServer::~TcpServer()
{
    if (m_clientSocket)
    {
        shutdown(m_clientSocket, SHUT_RDWR);
        close(m_clientSocket);
        m_clientSocket = NULL;
    }

    if (m_listenSocket)
    {
        shutdown(m_listenSocket, SHUT_RDWR);
        close(m_listenSocket);
        m_listenSocket = NULL;
    }

    // 在Linux环境下，不需要调用WSACleanup()
}

bool TcpServer::InitServer()
{
    // 1. 初始化环境
    if (socket(AF_INET, SOCK_STREAM, 0) == -1)
    {
        std::cout << "Init Socket Failed!\n";
        return false;
    }

    // 2. 创建监听套接字
    if ((m_listenSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        std::cout << "Create socket failed!\n";
        return false;
    }

    // 协议
    sockaddr_in sockadd = {
        0,
    };

    sockadd.sin_family = AF_INET;                // IPV4协议簇
    sockadd.sin_port = htons(m_uPort);           // 监听端口
    if( m_strIp == "0.0.0.0")
        sockadd.sin_addr.s_addr = htonl(INADDR_ANY); // 监听本机任意IP
    else
        sockadd.sin_addr.s_addr = inet_addr(m_strIp.c_str()); // 监听本机指定IP

    // 3. 监听套接字与IP地址及端口绑定
    if (bind(m_listenSocket, (struct sockaddr *)&sockadd, sizeof(sockadd)) == -1)
    {
        close(m_listenSocket);
        m_listenSocket = -1;
        std::cout << "Socket bind failed!\n";
        return false;
    }

    // 4. 监听套接字
    if (listen(m_listenSocket, 1) == -1)
    {
        close(m_listenSocket);
        m_listenSocket = -1;
        std::cout << "Socket listen failed!\n";
        return false;
    }

    return true;
}

bool TcpServer::WaitForClient()
{
    sockaddr_in addr = {
        0,
    };
    socklen_t addrlen = sizeof(addr);

    // 5. 等待客户端连接
    std::cout << "开始等待连接!\n";
    while (!stop_flag)
    {
        if (!m_bRunning)
        {
            m_clientSocket = accept(m_listenSocket, (struct sockaddr *)&addr, &addrlen);

            if (m_clientSocket == -1)
            {
                close(m_clientSocket);
                m_clientSocket = -1;
                std::cout << "Socket accept failed!\n";
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒后重试
            }
            else
            {
                std::cout << "客户端已连接!\n";
                m_bRunning = true;
            }
        }
    }
}

bool TcpServer::SendMsg(const std::string &strMsg)
{
    if (!m_clientSocket)
        return false;

    if (write(m_clientSocket, strMsg.c_str(), strMsg.length()) != -1)
    {
        std::cout << "发送成功:" << strMsg << "\n";
        return true;
    }
    else
    {
        std::cout << "发送失败!\n";
        return false;
    }
}

bool TcpServer::RecvMsg()
{
    if (!m_clientSocket)
        return false;

    const int iBufSize = 1024;
    char recvBuf[iBufSize] = {
        0,
    };
    auto iRecvSize = read(m_clientSocket, recvBuf, iBufSize);

    if (iRecvSize <= 0)
    {
        std::cout << "接收失败!\n";
        return false;
    }
    else
    {
        std::cout << "接收成功:" << recvBuf << "\n";
        return true;
    }
}

void TcpServer::ListenAndStoreMsg()
{
    const int iBufSize = 1024;
    char recvBuf[iBufSize] = {
        0,
    };

    while (!stop_flag)
    {
        if (m_bRunning)
        {
            int iRecvSize = recv(m_clientSocket, recvBuf, iBufSize, 0);

            if (iRecvSize <= 0)
            {
                std::cout << "连接断开!\n";
                m_bRunning = false;
            }
            else
            {
                std::cout << "接收成功:" << iRecvSize << "\n";
                // 加锁
                m_mutex.lock();

                // 将收到的消息存入队列
                char *data = new char[iRecvSize + 1];
                memcpy(data, recvBuf, iRecvSize); // 只复制实际接收到的数据
                data[iRecvSize + 1] = '\0';       // 确保字符串以null结尾
                m_msgQueue.push(data);

                // 解锁
                m_mutex.unlock();
            }
        }
    }
}

void TcpServer::startListening()
{
    std::cout << "开启消息监听线程" << std::endl;
    std::thread listenThread(&TcpServer::ListenAndStoreMsg, this);
    listenThread.detach(); // 让线程在后台运行
}

char *TcpServer::GetMsg()
{
    // 加锁
    m_mutex.lock();

    // 检查队列是否为空
    if (!m_msgQueue.empty())
    {
        char *data = m_msgQueue.front();
        m_msgQueue.pop();

        // 解锁
        m_mutex.unlock();

        return data;
    }
    else
    {
        // 解锁
        m_mutex.unlock();

        return nullptr;
    }
}

void TcpServer::startWaitForClient()
{
    std::cout << "开启等待客户端连接线程" << std::endl;
    std::thread waitForClientThread(&TcpServer::WaitForClient, this);
    waitForClientThread.detach(); // 让线程在后台运行
}

bool TcpServer::HasMsg()
{
    bool hasMsg;

    // 加锁
    m_mutex.lock();

    // 检查队列是否为空
    hasMsg = !m_msgQueue.empty();

    // 解锁
    m_mutex.unlock();

    return hasMsg;
}

bool TcpServer::Stop()
{
    stop_flag = true;
}

// #endif

// 新建一个UDP客户端类
class UDPServer
{
public:
    UDPServer(std::string strIp, unsigned int uPort);
    virtual ~UDPServer();

    // 初始化UDP客户端
    bool InitUDPClient();

    // 发送UDP数据
    bool SendUDPMsg(const char *strMsg);

    unsigned int m_uPort; // 监听端口
    std::string m_strIp;  // 用于监听本机指定IP地址

private:
    int m_udpSocket = -1;                             // UDP套接字
    sockaddr_in m_clientAddr;                         // 客户端地址
    socklen_t m_clientAddrLen = sizeof(m_clientAddr); // 客户端地址长度
};

UDPServer::UDPServer(std::string strIp = "localhost", unsigned int uPort = 8888) : m_strIp(strIp),
                                                                                   m_uPort(uPort)
{
}

UDPServer::~UDPServer()
{
    if (m_udpSocket)
    {
        shutdown(m_udpSocket, SHUT_RDWR);
        close(m_udpSocket);
        m_udpSocket = NULL;
    }
}

bool UDPServer::InitUDPClient()
{
    m_clientAddr.sin_family = AF_INET;                         // IPV4协议簇
    m_clientAddr.sin_port = htons(m_uPort);                      // 监听端口
    inet_pton(AF_INET, m_strIp.c_str(), &m_clientAddr.sin_addr); // 监听指定IP

    // 初始化UDP客户端
    if (socket(AF_INET, SOCK_DGRAM, 0) == -1)
    {
        std::cout << "Init UDP Socket Failed!\n";
        return false;
    }

    // 创建UDP客户端套接字
    if ((m_udpSocket = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        std::cout << "Create UDP socket failed!\n";
        return false;
    }

    return true;
}

bool UDPServer::SendUDPMsg(const char *strMsg)
{
    if (!m_udpSocket)
        return false;

    if (sendto(m_udpSocket, strMsg, 1024, 0, (struct sockaddr *)&m_clientAddr, m_clientAddrLen) != -1)
    {
        // std::cout << "发送成功:" << strMsg << "\n";
        return true;
    }
    else
    {
        std::cout << "发送失败!\n";
        return false;
    }
}

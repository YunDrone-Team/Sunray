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
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <thread>
#include <atomic>


#include "pcl/point_cloud.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/compression/compression_profiles.h"

#include "Communication/Cell.h"
#include "Communication/MSG.h"
#include "Communication/DecoderInterfaceBase.h"
#include "Communication/MessageQueue.h"
#include "Communication/LatestDataHolder.h"
#include "Communication/Codec.h"

// 压缩任务结构体
struct CompressionTask {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    float octreeResolution=0.01;
    std::string targetIp;
    uint16_t targetPort;
    uint8_t robotID;
};

// 压缩结果结构体
struct CompressionResult {
    std::vector<uint8_t> compressedData;
    size_t dataSize;
    std::string errorMsg;
    std::string targetIp;
    uint16_t targetPort;
    uint8_t robotID;
};


class CommunicationUDPSocket
{
public:
//    static CommunicationUDPSocket * getInstance();              //获取一个实例
    static void releaseInstance();                              // 手动释放（程序关闭时调用）
    static CommunicationUDPSocket& getInstance()
    {
        static CommunicationUDPSocket instance;
        return instance;
    }
    boost::signals2::signal<void(std::vector<uint8_t>, std::string)> sigReadData;
    boost::signals2::signal<void(ReceivedParameter)> sigUDPUnicastReadData;
    boost::signals2::signal<void(int)> sigUDPError;
    boost::signals2::signal<void()> sigSendPointCloudData;


    void setDecoderInterfacePtr(DecoderInterfaceBase* ptr);

    void setFilterNetworkCard(std::string ip); //设置要过滤的网卡IP，在InitSocket和Bind前调用才有效
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
    int sendUDPData(std::vector<uint8_t> sendData,std::string targetIp="",uint16_t targetPort=9898);              //发送数据接口
    int sendUDPBroadcastData(std::vector<uint8_t> sendData,uint16_t targetPort=9898);
    int sendUDPMulticastData(std::vector<uint8_t> sendData,uint16_t targetPort=9898);

    void setUDPReadState(bool state);   //设置UDP是否循环读取


    /**
     * @param cloud 待压缩的PCL PointCloud2点云
     * @param octree_resolution 八叉树分辨率（越小精度越高，压缩率越低）
     * @return 压缩后的二进制字节数组
     * @throw std::runtime_error 压缩失败时抛出异常
     */
    std::vector<uint8_t> compressPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, float octreeResolution = 0.01);
    void compressionThread();
    void pushSendPointCloudDataQueue(pcl::PointCloud<pcl::PointXYZ> pclCloud,uint8_t robotID,std::string targetIp="",uint16_t targetPort=9898,float octreeResolution = 0.01);
    void onSendPointCloudData();

    //    void sendUDPData(std::vector<char> sendData,std::string targetIp,unsigned short targetPort);

    void setRunState(bool state);
    int findStdVectorComponent(uint8_t a,uint8_t b,std::vector<uint8_t> Data);

    void UpdateMulticast();
    void Close(SOCKET tempSock);
    SOCKET SocketConfiguration(SOCKET tempSock);
    ~CommunicationUDPSocket();
private:
    CommunicationUDPSocket();

    SOCKET UpdateMulticastConfiguration(SOCKET tempSock);

    std::vector<NetworkInterface> getNetworkInterfaces();// 遍历网卡函数
    bool isLinuxInterfaceActive(const std::string& interfaceName); // 检查 Linux 网卡是否活动
    std::string wstringToString(const std::wstring& wstr);// 宽字符转窄字符函数
    void UDPUnicastManagingData(std::vector<uint8_t>& data,std::string IP,uint16_t port);
    void ResetFdRead(fd_set& fdRead );
    std::vector<uint8_t> UDPUnicastCacheData;//UDPUnicast缓存数据，用于缓存数据未收全的情况
    bool HandleUdpSocketReadEvent(SOCKET tempSock,fd_set& fdRead);
    bool resetMaxSock();
    int SendDataToMulticastTarget(SOCKET tempSock, std::vector<uint8_t> sendData,uint16_t targetPort);
    int SendDataToTarget(SOCKET tempSock,  std::vector<uint8_t> sendData, std::string targetIp, uint16_t targetPort);

    void startIOContext();//启动IO上下文线程
    void startPeriodicTimer(int ms);//启动周期定时器
    void startOneShotTimer(int delay_ms);//启动单次定时器
    void stop();//停止所有定时器和IO线程
    void onTimerCallback(const boost::system::error_code& ec);//周期定时器回调函数 ec 错误码
    void onSingleTimerCallback();//单次定时器回调函数

    static CommunicationUDPSocket* CommunicationPtr;
    std::atomic<bool> UDPReadState;
    std::atomic<bool> runState;
    std::unordered_map<std::string, SOCKET> ipSocketMap;

    std::mutex mutexSocket;
    std::mutex mutexSend;

    char* _buffer;
    int _bufferSize;
    char* _souIp;
    uint16_t* _linuxPort;
    unsigned short* _winPort;
    DecoderInterfaceBase* decoderInterfacePtr=nullptr;
    std::string multicastIP;
    SOCKET maxSock=INVALID_SOCKET;
    SOCKET defaultSock=INVALID_SOCKET;

    boost::asio::io_context IOContext;                          //Asio事件循环
    boost::asio::steady_timer timer;                            //周期定时器
    std::atomic<bool> TimeRunning;                              //运行标志（原子变量）
    std::thread IOThread;                                       //IO事件循环线程
    int timingMsec;                                             //周期定时，毫秒

    MessageQueue<CompressionTask> compressionTaskQueue;         //压缩任务队列
    LatestDataHolder<CompressionResult> compressionDataQueue;   //压缩数据队列
    std::atomic<bool> isCompressionThreadRunning;

    Codec codec;
    std::string filterNetworkCard;//要过滤的网卡IP
};

#endif // COMMUNICATIONUDPSOCKET_H

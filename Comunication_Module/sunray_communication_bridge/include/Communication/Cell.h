#ifndef CELL_H
#define CELL_H

#include <string>

//SOCKET
#ifdef _WIN32
    #define FD_SETSIZE      256
    #define WIN32_LEAN_AND_MEAN
    #define _WINSOCK_DEPRECATED_NO_WARNINGS
//    #include<windows.h>
//    #include<WinSock2.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <winsock2.h>

//#include <ws2atm.h>
//#include <ws2bth.h>
//#include <ws2def.h>
//#include <ws2dnet.h>
//#include <ws2ipdef.h>
//#include <ws2spi.h>
#include <iphlpapi.h>


//    #pragma comment(lib,"ws2_32.lib")
#else
    #include<unistd.h> //uni std
    #include<arpa/inet.h>
    #include<string.h>
    #include<signal.h>
//新加的，待验证
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <net/if.h>


    #define SOCKET int
    #define INVALID_SOCKET  (SOCKET)(~0)
    #define SOCKET_ERROR            (-1)
#endif


struct SocketIP
{
    SOCKET socket;
    std::string IP;
};

// 定义网卡信息结构体
struct NetworkInterface {
    std::string name;
    std::string ip;
};

#endif // CELL_H

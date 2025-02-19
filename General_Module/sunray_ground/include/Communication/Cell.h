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


//    #pragma comment(lib,"ws2_32.lib")
#else
    #include<unistd.h> //uni std
    #include<arpa/inet.h>
    #include<string.h>
    #include<signal.h>

    #define SOCKET int
    #define INVALID_SOCKET  (SOCKET)(~0)
    #define SOCKET_ERROR            (-1)
#endif
//


//
//#include<stdio.h>

//缓冲区最小单元大小
#ifndef RECV_BUFF_SZIE
#define RECV_BUFF_SZIE 8192
#define SEND_BUFF_SZIE 10240
#endif // !RECV_BUFF_SZIE

enum ActorState
{
    Server =1,
    Client =2
};

struct SocketIP
{
    SOCKET socket;
    std::string IP;
};

#endif // CELL_H

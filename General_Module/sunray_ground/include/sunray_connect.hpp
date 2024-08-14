#ifndef sunray_connect
#define sunray_connect

#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <queue>
#include <mutex>
#include <unistd.h>

class sunray_socket
{
public:
	sunray_socket();
	~sunray_socket();

	// TCP socket
	int tcpSocket;
	struct sockaddr_in tcpAddr;

	// UDP socket
	int udpSocket;
	struct sockaddr_in udpAddr;

	// Initialize TCP socket with IP and port
	void initTCPSocket(const std::string &ip, int port);

	// Initialize UDP socket with IP and port
	void initUDPSocket(const std::string &ip, int port);

	// Bind TCP socket
	void bindTCPSocket();

	// Bind UDP socket
	void bindUDPSocket();

	// Listen for incoming TCP messages
	void listenTCP();

	// Listen for incoming UDP messages
	void listenUDP();

	// Send message over TCP
	void sendTCPMessage(const char *message);

	// Send message over UDP
	void sendUDPMesssage(const char *message);

	// Message queue for TCP messages
	std::queue<char *> tcpMessageQueue;

	// Mutex for TCP message queue
	std::mutex tcpMessageQueueMutex;

	// Message queue for UDP messages
	std::queue<char *> udpMessageQueue;

	// Mutex for UDP message queue
	std::mutex udpMessageQueueMutex;

private:
	// Create a socket
	int createSocket(int domain, int type, int protocol);

	// Thread function to handle incoming TCP messages
	void handleTCPMessages();

	// Thread function to handle incoming UDP messages
	void handleUDPMessages();
};

sunray_socket::sunray_socket() : tcpSocket(-1), udpSocket(-1) {}

sunray_socket::~sunray_socket()
{
	if (tcpSocket != -1)
	{
		close(tcpSocket);
	}
	if (udpSocket != -1)
	{
		close(udpSocket);
	}
}

void sunray_socket::initTCPSocket(const std::string &ip, int port)
{
	tcpSocket = createSocket(AF_INET, SOCK_STREAM, 0);
	if (tcpSocket == -1)
	{
		std::cerr << "Failed to create TCP socket" << std::endl;
		return;
	}

	tcpAddr.sin_family = AF_INET;
	tcpAddr.sin_port = htons(port);
	inet_pton(AF_INET, ip.c_str(), &tcpAddr.sin_addr);
}

void sunray_socket::initUDPSocket(const std::string &ip, int port)
{
	udpSocket = createSocket(AF_INET, SOCK_DGRAM, 0);
	if (udpSocket == -1)
	{
		std::cerr << "Failed to create UDP socket" << std::endl;
		return;
	}

	udpAddr.sin_family = AF_INET;
	udpAddr.sin_port = htons(port);
	inet_pton(AF_INET, ip.c_str(), &udpAddr.sin_addr);
}

void sunray_socket::bindTCPSocket()
{
	if (bind(tcpSocket, (struct sockaddr *)&tcpAddr, sizeof(tcpAddr)) == -1)
	{
		std::cerr << "Failed to bind TCP socket" << std::endl;
	}
}

void sunray_socket::bindUDPSocket()
{
	if (bind(udpSocket, (struct sockaddr *)&udpAddr, sizeof(udpAddr)) == -1)
	{
		std::cerr << "Failed to bind UDP socket" << std::endl;
	}
}

int sunray_socket::createSocket(int domain, int type, int protocol)
{
	int sockfd = socket(domain, type, protocol);
	if (sockfd == -1)
	{
		std::cerr << "Failed to create socket" << std::endl;
	}
	return sockfd;
}

void sunray_socket::listenTCP()
{
	// Create a thread to handle incoming TCP messages
	std::thread handleTCPMessagesThread([this]
										{ handleTCPMessages(); });
	handleTCPMessagesThread.detach();

	// Listen for incoming TCP connections
	if (listen(tcpSocket, 3) == -1)
	{
		std::cerr << "Failed to listen on TCP socket" << std::endl;
	}
}

void sunray_socket::listenUDP()
{
	// Create a thread to handle incoming UDP messages
	std::thread handleUDPMessagesThread([this]
										{ handleUDPMessages(); });
	handleUDPMessagesThread.detach();
}

void sunray_socket::handleTCPMessages()
{
	while (true)
	{
		int clientSocket = accept(tcpSocket, nullptr, nullptr);
		if (clientSocket == -1)
		{
			std::cerr << "Failed to accept TCP connection" << std::endl;
			continue;
		}

		char buffer[1024];
		int bytesRead = read(clientSocket, buffer, 1024);
		if (bytesRead == -1)
		{
			std::cerr << "Failed to read from TCP socket" << std::endl;
			continue;
		}

		char *message = new char[bytesRead + 1];
		memcpy(message, buffer, bytesRead);
		message[bytesRead] = '\0'; // null-terminate the string

		{
			std::lock_guard<std::mutex> lock(tcpMessageQueueMutex);
			tcpMessageQueue.push(message);
		}

		close(clientSocket);
	}
}

void sunray_socket::handleUDPMessages()
{
	while (true)
	{
		char udpBuffer[1024];
		socklen_t udpAddrLen = sizeof(udpAddr);
		int udpBytesRead = recvfrom(udpSocket, udpBuffer, 1024, 0, (struct sockaddr *)&udpAddr, &udpAddrLen);
		if (udpBytesRead == -1)
		{
			std::cerr << "Failed to read from UDP socket" << std::endl;
			continue;
		}

		char *udpMessage = new char[udpBytesRead + 1];
		memcpy(udpMessage, udpBuffer, udpBytesRead);
		udpMessage[udpBytesRead] = '\0'; // null-terminate the string

		{
			std::lock_guard<std::mutex> lock(udpMessageQueueMutex);
			udpMessageQueue.push(udpMessage);
		}
	}
}

void sunray_socket::sendTCPMessage(const char *message)
{
	int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == -1)
	{
		std::cerr << "Failed to create TCP client socket" << std::endl;
		return;
	}

	if (connect(clientSocket, (struct sockaddr *)&tcpAddr, sizeof(tcpAddr)) == -1)
	{
		std::cerr << "Failed to connect to TCP server" << std::endl;
		close(clientSocket);
		return;
	}

	send(clientSocket, message, strlen(message), 0);
	close(clientSocket);
}

void sunray_socket::sendUDPMesssage(const char *message)
{
	sendto(udpSocket, message, strlen(message), 0, (struct sockaddr *)&udpAddr, sizeof(udpAddr));
}


#endif
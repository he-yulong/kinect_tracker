#include "szl/udp_sender.h"

szl_kinect::UDPSender::UDPSender(const std::string& ip, const int& port)
{
	// Initialize socket
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	std::cout << ip << ":" << port << " is ready." << std::endl;
	// Config server
	//recvAddr.sin_port = htons(port);
	//InetPton(AF_INET, (PCWSTR)ip.c_str(), &recvAddr.sin_addr.s_addr);
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(port);
	recvAddr.sin_addr.s_addr = inet_addr(ip.c_str()); 
	//recvAddr.sin_addr.s_addr = inet_pton(ip.c_str());
}

szl_kinect::UDPSender::~UDPSender()
{
	std::printf("finished sending,close socket.\n");
	closesocket(sendSocket);
	WSACleanup();
}

void szl_kinect::UDPSender::Send(std::string data)
{
	char* cdata = (char*)(data.data());
	std::printf("[%zd]: %s\n", strlen(cdata), cdata);
	sendto(sendSocket, cdata, strlen(cdata), 0, (SOCKADDR*)&recvAddr, sizeof(recvAddr));
}

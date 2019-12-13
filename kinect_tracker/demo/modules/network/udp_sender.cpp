#include "szl/udp_sender.h"
using namespace szl_kinect; 

#include <iostream>
#include <WS2tcpip.h>
using namespace std;


#pragma comment(lib, "ws2_32.lib")

UDPSender::UDPSender(const string& ip, const int& port)
{
	// Initialize socket
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	cout << ip << ":" << port << " is ready." << endl;
	// Config server
	//recvAddr.sin_port = htons(port);
	//InetPton(AF_INET, (PCWSTR)ip.c_str(), &recvAddr.sin_addr.s_addr);
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(port);
	recvAddr.sin_addr.s_addr = inet_addr(ip.c_str()); 
	//recvAddr.sin_addr.s_addr = inet_pton(ip.c_str());
}

UDPSender::~UDPSender()
{
	printf("finished sending,close socket.\n");
	closesocket(sendSocket);
	WSACleanup();
}

void UDPSender::Send(string data)
{
	char* cdata = (char*)(data.data());
	printf("[%zd]: %s\n", strlen(cdata), cdata);
	sendto(sendSocket, cdata, strlen(cdata), 0, (SOCKADDR*)&recvAddr, sizeof(recvAddr));
}

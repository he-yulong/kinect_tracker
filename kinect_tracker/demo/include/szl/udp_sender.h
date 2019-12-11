#pragma once
#include <WinSock2.h>
#include <string>
using namespace std;

namespace szl_kinect {
	class UDPSender
	{
	public:
		UDPSender(const string& ip, const int& port);
		~UDPSender();

		void Send(string data);

	private:
		WSADATA wsaData;
		SOCKET sendSocket;
		sockaddr_in recvAddr;
	};
}

#pragma once
#include <iostream>
using std::string;

namespace szl_kinect {
	class SingleUDPTracker
	{
	public:
		int Run(string udp_ip, int udp_port, bool all_joints);
	};
}

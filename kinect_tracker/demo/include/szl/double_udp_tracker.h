#pragma once
#include <iostream>
using std::string;

namespace szl_kinect {
	class DoubleUDPTracker
	{
	public:
		int Run(string udp_ip_sub, string udp_ip_master, int udp_port_sub, int udp_port_master, bool all_joints, int max_frame);
	};
}

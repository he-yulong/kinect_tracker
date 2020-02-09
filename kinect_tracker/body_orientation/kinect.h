#pragma once
#include <k4a/k4a.h>
#include <k4abt.h>
#include <iostream>
#include <string>
#include <assert.h>

class Kinect
{
public:
	Kinect(int device_idx = 0);
	void Open();
	void PrintBodyOrientation(int max_frame = 100);
	void Close();

private:
	int device_index;
	k4a_device_configuration_t device_config;
	k4a_device_t device;
	k4abt_tracker_t tracker;

	// 内联成员函数：用于验证
	// 存在 C26812 的小问题
	void verify(const k4a_result_t& result, const std::string& msg, const std::string& error)
	{
		if (result != K4A_RESULT_SUCCEEDED)
		{
			std::cout << error << std::endl;
			std::cout << " - (File: " << __FILE__ << ", Function: " << __FUNCTION__ << ", Line: " << __LINE__ << std::endl;
			exit(1);
		}
		std::cout << msg << std::endl;
	};
	void print_body_information(k4abt_body_t body);
	void print_body_index_map_middle_line(k4a_image_t body_index_map);
};


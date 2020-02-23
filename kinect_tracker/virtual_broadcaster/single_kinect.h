#pragma once
#include <iostream>

#include <k4a/k4a.h>
#include <k4abt.h>

#include "FloorDetector.h"
//#include "Utilities.h"
#include "PointCloudGenerator.h"

class SingleKinect
{
public:
	SingleKinect(int device_index_val = 0);
	void Open();
	void Running(int max_frame = 200);
	void Close();

private:
	int device_index;
	k4a_device_t device;
	k4a_device_configuration_t device_config;
	k4a_calibration_t sensor_calibration;
	k4a_capture_t sensor_capture;
	Samples::PointCloudGenerator point_cloud_generator;
	Samples::FloorDetector floor_detector;

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
};


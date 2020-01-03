#pragma once
#include <iostream>
#include <k4a/k4a.h>
#include <k4abt.h>
using std::cout;
using std::endl;
using std::string;
#include <vector>
using std::vector;
#include "szl/udp_sender.h"
using szl_kinect::UDPSender;
#include <Eigen/Dense>
#include <sstream>


#define VERIFY(result, msg, error)                                                                       \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \
	printf("%s \n", msg);																				 \


namespace szl_kinect {
	class TrackerProcessor
	{
	public:
		TrackerProcessor(Eigen::Matrix3f rotation_matrix);
		TrackerProcessor&  Process(k4abt_skeleton_t skeleton);

		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;

		//static vector<string> Split(const string& input, const string& delim);

		//TrackerProcessor& FromString(string skeleton_string);
		TrackerProcessor& FixView(k4abt_skeleton_t skeleton);
		string ToString();
		//string ToString(vector<k4abt_joint_t> mSkeleton0);
		//string ToString(vector<k4abt_joint_t> mSkeleton0, Matrix3f stereo_rotate);
		

	private:
		Eigen::Vector3f shift_matrix;
		Eigen::Matrix3f rotation_matrix;
		vector<k4abt_joint_t> skeleton_matrix;
		bool mHasShift;
	};
	class QuaternionUDPTracker
	{
	public:
		k4a_wait_result_t get_capture_result;
		k4a_wait_result_t queue_capture_result;
		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result;
		k4abt_skeleton_t skeleton;
		TrackerProcessor processor;
		QuaternionUDPTracker(int device_index, Eigen::Matrix3f rotation_matrix, k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL);
		void Open();
		void Initialize();
		void Close();
		void UpdateCapture();
		bool IsGetCaptureResultSucceeded();
		bool IsGetCaptureResultTimeout();
		void UpdateTrackerEnqueue();
		bool IsQueueCaptureResultGood();
		void UpdateBodyFrame();
		

	private:
		int device_index;
		k4a_device_configuration_t config;
		k4a_device_t device = NULL;
		k4a_calibration_t sensor_calibration;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		k4abt_tracker_t tracker = NULL;
		k4a_capture_t sensor_capture;
	};

	class SeveralQuaternionUDPTracker
	{
	public:
		SeveralQuaternionUDPTracker(int device_num = 2, std::string udp_ip = "127.0.0.1", int udp_port = 8999);
		int Run(int max_frame = 100);
	private:
		int device_num;
		vector<QuaternionUDPTracker> several_kinects;
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		UDPSender* udp_sender = NULL;
		void initialize();
		void close();  // close several kinects.
	};

}

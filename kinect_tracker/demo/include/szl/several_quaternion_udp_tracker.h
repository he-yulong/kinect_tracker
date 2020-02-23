#pragma once
// 用于多个 Kinect 设备的同步跟踪。
// 其中使用 UDP 发送跟踪数据（基于四元数）
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

// 验证函数
#define VERIFY(result, msg, error)                                                                       \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \
	printf("%s \n", msg);																				 \

// 定义命名空间
namespace szl_kinect {
	// TrackerProcessor 类用于处理原生跟踪数据，转换为客户端需要的跟踪数据
	class TrackerProcessor
	{
	public:
		// 每个 Kinect 所接收的旋转矩阵并不相同，该旋转矩阵是相对于世界坐标下的旋转矩阵
		// 一种简单的实现是把其中一个 Kinect 的相机坐标系定义为世界坐标系
		TrackerProcessor(Eigen::Matrix3f view_rotation);
		// ?????
		TrackerProcessor& Process(k4abt_skeleton_t skeleton);
		// FixView 方法用于调整视图，经过一定的旋转和平移，使得客户端视角可以方便地看到人物。
		TrackerProcessor& FixView(k4abt_skeleton_t skeleton);
		// 给出客户端所需要的字符串
		string ToString();
	private:
		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;
		Eigen::Vector3f shift_matrix;  // 调整视图的平移
		Eigen::Matrix3f view_rotation;  // 调整视图的旋转
		Eigen::Matrix3f qua_rotation;  // 需要加在四元数上面的旋转
		vector<k4abt_joint_t> skeleton_matrix;
		bool mHasShift = false;
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

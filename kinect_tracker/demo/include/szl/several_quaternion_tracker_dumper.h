#pragma once
// 用于多个 Kinect 设备的同步跟踪。
// 把需要发的跟踪数据（基于四元数）持久化
#include <iostream>
#include <k4a/k4a.h>
#include <k4abt.h>
using std::cout;
using std::endl;
using std::string;
#include <vector>
using std::vector;
#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <direct.h>
#include <io.h>
#include <ctime>																 \


// 定义命名空间
namespace szl_kinect_dump {
	// 验证函数
	inline void verify(k4a_result_t result, string msg, string error);

	// 创建一个新目录
	static string makeDir();

	// TrackerProcessor 类用于处理原生跟踪数据，转换为客户端需要的跟踪数据
	class TrackerProcessor
	{
	public:
		// 每个 Kinect 所接收的旋转矩阵并不相同，该旋转矩阵是相对于世界坐标下的旋转矩阵
		// 一种简单的实现是把其中一个 Kinect 的相机坐标系定义为世界坐标系
		TrackerProcessor(Eigen::Matrix3f view_rotation, Eigen::Matrix3f qua_rotation);
		// ?????
		TrackerProcessor&  Process(k4abt_skeleton_t skeleton);
		// FixView 方法用于调整视图，经过一定的旋转和平移，使得客户端视角可以方便地看到人物。
		TrackerProcessor& FixView(k4abt_skeleton_t* skeleton);
		// 对四元数进行旋转
		void RotateQuaternion(k4abt_skeleton_t* skeleton);
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
	// QuaternionUDPTracker 类抽象单个Kinect设备
	class QuaternionUDPTracker
	{
	public:
		// 若干返回结果
		k4a_wait_result_t get_capture_result;
		k4a_wait_result_t queue_capture_result;
		k4a_wait_result_t pop_frame_result;

		k4abt_frame_t body_frame{};  // 人体数据帧
		k4abt_skeleton_t skeleton;  // 人体骨骼
		TrackerProcessor processor;  // 数据处理器

		// 构造函数
		// 传入设备序号、针对世界坐标系的旋转矩阵、设备配置项
		QuaternionUDPTracker(int device_index, Eigen::Matrix3f rotation_matrix, Eigen::Matrix3f qua_matrix, k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL);
		// 设备的开启
		void Open();
		// 设备的初始化
		void Initialize();
		// 设备的关闭
		void Close();
		// 设备深度帧的捕获
		void UpdateCapture();
		// 判断捕捉是否成功
		bool IsGetCaptureResultSucceeded();
		bool IsGetCaptureResultTimeout();
		// 更新 tracker 队列
		void UpdateTrackerEnqueue();
		// 判断队列结果是否成功
		bool IsQueueCaptureResultGood();
		// 更新人体数据帧
		void UpdateBodyFrame();
		// 写入 CSV 文件
		void WriteCSV(std::ofstream* out_file, int frame_id);
		
	private:
		int device_index{};  // 设备序号
		k4a_device_configuration_t config{};  // 配置项
		k4a_device_t device{};  // 设备 handler
		k4a_calibration_t sensor_calibration{};  // 设备标定相关
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;  // traker 配置
		k4abt_tracker_t tracker{};  // tracker handler
		k4a_capture_t sensor_capture{};  // 感知器获取器
		uint64_t device_timestamp{};  // 设备时间戳
		// 节点相关
		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;
	};

	// SeveralQuaternionTrackerDumper 类控制多个设备交互的高层逻辑
	class SeveralQuaternionTrackerDumper
	{
	public:
		// Dumper类的初始化
		SeveralQuaternionTrackerDumper(int device_num = 2);
		// 程序运行入口
		int Run(int max_frame = 100);
	private:
		// 设备数量
		int device_num;
		// 设备类的容器
		vector<QuaternionUDPTracker> several_kinects;
		// 默认设备的配置
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		// 设备的初始化
		void initialize();
		// 设备的关闭
		void close();
	};

}

#pragma once
// ���ڶ�� Kinect �豸��ͬ�����١�
// ����Ҫ���ĸ������ݣ�������Ԫ�����־û�
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


// ���������ռ�
namespace szl_kinect_dump {
	// ��֤����
	inline void verify(k4a_result_t result, string msg, string error);

	// ����һ����Ŀ¼
	static string makeDir();

	// TrackerProcessor �����ڴ���ԭ���������ݣ�ת��Ϊ�ͻ�����Ҫ�ĸ�������
	class TrackerProcessor
	{
	public:
		// ÿ�� Kinect �����յ���ת���󲢲���ͬ������ת��������������������µ���ת����
		// һ�ּ򵥵�ʵ���ǰ�����һ�� Kinect ���������ϵ����Ϊ��������ϵ
		TrackerProcessor(Eigen::Matrix3f view_rotation, Eigen::Matrix3f qua_rotation);
		// ?????
		TrackerProcessor&  Process(k4abt_skeleton_t skeleton);
		// FixView �������ڵ�����ͼ������һ������ת��ƽ�ƣ�ʹ�ÿͻ����ӽǿ��Է���ؿ������
		TrackerProcessor& FixView(k4abt_skeleton_t* skeleton);
		// ����Ԫ��������ת
		void RotateQuaternion(k4abt_skeleton_t* skeleton);
		// �����ͻ�������Ҫ���ַ���
		string ToString();
	private:
		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;
		Eigen::Vector3f shift_matrix;  // ������ͼ��ƽ��
		Eigen::Matrix3f view_rotation;  // ������ͼ����ת
		Eigen::Matrix3f qua_rotation;  // ��Ҫ������Ԫ���������ת
		vector<k4abt_joint_t> skeleton_matrix;
		bool mHasShift = false;
	};
	// QuaternionUDPTracker ����󵥸�Kinect�豸
	class QuaternionUDPTracker
	{
	public:
		// ���ɷ��ؽ��
		k4a_wait_result_t get_capture_result;
		k4a_wait_result_t queue_capture_result;
		k4a_wait_result_t pop_frame_result;

		k4abt_frame_t body_frame{};  // ��������֡
		k4abt_skeleton_t skeleton;  // �������
		TrackerProcessor processor;  // ���ݴ�����

		// ���캯��
		// �����豸��š������������ϵ����ת�����豸������
		QuaternionUDPTracker(int device_index, Eigen::Matrix3f rotation_matrix, Eigen::Matrix3f qua_matrix, k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL);
		// �豸�Ŀ���
		void Open();
		// �豸�ĳ�ʼ��
		void Initialize();
		// �豸�Ĺر�
		void Close();
		// �豸���֡�Ĳ���
		void UpdateCapture();
		// �жϲ�׽�Ƿ�ɹ�
		bool IsGetCaptureResultSucceeded();
		bool IsGetCaptureResultTimeout();
		// ���� tracker ����
		void UpdateTrackerEnqueue();
		// �ж϶��н���Ƿ�ɹ�
		bool IsQueueCaptureResultGood();
		// ������������֡
		void UpdateBodyFrame();
		// д�� CSV �ļ�
		void WriteCSV(std::ofstream* out_file, int frame_id);
		
	private:
		int device_index{};  // �豸���
		k4a_device_configuration_t config{};  // ������
		k4a_device_t device{};  // �豸 handler
		k4a_calibration_t sensor_calibration{};  // �豸�궨���
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;  // traker ����
		k4abt_tracker_t tracker{};  // tracker handler
		k4a_capture_t sensor_capture{};  // ��֪����ȡ��
		uint64_t device_timestamp{};  // �豸ʱ���
		// �ڵ����
		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;
	};

	// SeveralQuaternionTrackerDumper ����ƶ���豸�����ĸ߲��߼�
	class SeveralQuaternionTrackerDumper
	{
	public:
		// Dumper��ĳ�ʼ��
		SeveralQuaternionTrackerDumper(int device_num = 2);
		// �����������
		int Run(int max_frame = 100);
	private:
		// �豸����
		int device_num;
		// �豸�������
		vector<QuaternionUDPTracker> several_kinects;
		// Ĭ���豸������
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		// �豸�ĳ�ʼ��
		void initialize();
		// �豸�Ĺر�
		void close();
	};

}

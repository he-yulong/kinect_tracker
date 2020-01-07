#include "szl/several_quaternion_udp_tracker.h"

szl_kinect::QuaternionUDPTracker::QuaternionUDPTracker(int device_index, Eigen::Matrix3f rotation_matrix, k4a_device_configuration_t config): device_index(device_index), config(config), processor(TrackerProcessor(rotation_matrix))
{
	cout << "Kinect " << device_index << " instance created.";
}

void szl_kinect::QuaternionUDPTracker::Open()
{
	// opening two k4a device
	cout << device_index << ": Started opening K4A device..." << endl;
	VERIFY(k4a_device_open(device_index, &device), "Open K4A Device succeed.", "Open K4A Device failed!");
	VERIFY(k4a_device_start_cameras(device, &config), "Start K4A cameras succeed.", "Start K4A cameras failed!");
	cout << device_index << ": Finished opening K4A device." << endl;
}

void szl_kinect::QuaternionUDPTracker::Initialize()
{
	// Sensor calibration.
	VERIFY(k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &sensor_calibration),
		"Get calibration succeed.",
		"Get calibration failed!");

	tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization succeed.", "Body tracker initialization failed!");
}

void szl_kinect::QuaternionUDPTracker::Close()
{
	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
}

void szl_kinect::QuaternionUDPTracker::UpdateCapture()
{
	get_capture_result = k4a_device_get_capture(device, &sensor_capture, 0);
}

bool szl_kinect::QuaternionUDPTracker::IsGetCaptureResultSucceeded()
{
	return get_capture_result == K4A_WAIT_RESULT_SUCCEEDED;
}

bool szl_kinect::QuaternionUDPTracker::IsGetCaptureResultTimeout()
{
	return get_capture_result == K4A_WAIT_RESULT_TIMEOUT;
}

void szl_kinect::QuaternionUDPTracker::UpdateTrackerEnqueue()
{
	queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
	k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
}

bool szl_kinect::QuaternionUDPTracker::IsQueueCaptureResultGood()
{
	if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		printf("Error! Add capture to tracker process queue timeout!\n");
		return false;
	}
	else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
	{
		printf("Error! Add capture to tracker process queue failed!\n");
		return false;
	}
	return true;
}

void szl_kinect::QuaternionUDPTracker::UpdateBodyFrame()
{
	pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	//vector<k4abt_joint_t> mSkeleton0;
	if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
		printf("%zu bodies are detected!\n", num_bodies);
		cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame) << endl;
		// Get skeletons. We only extract the first skeleton
		if (num_bodies > 0)
		{
			k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);
			if (skeleton_result == K4A_RESULT_SUCCEEDED)
			{
				// Output joints
				string skeleton_result = "";
				skeleton_result = processor.FixView(skeleton).ToString();
				cout << skeleton_result << endl;
				//mSkeleton0 = processor.mSkeleton;
			}
			else if (skeleton_result == K4A_RESULT_FAILED)
			{
				printf("Error! Get body skeleton failed!\n");
			}
		}
		k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
	}
	else
	{
		printf("Pop body frame result failed!\n");
	}
}


szl_kinect::SeveralQuaternionUDPTracker::SeveralQuaternionUDPTracker(int device_num, string udp_ip, int udp_port): device_num(device_num)
{
	// Initialize UDP sender
	udp_sender = &UDPSender(udp_ip, udp_port);

	// Set configuration.
	cout << "Setting " << device_num << "kinect(s)' configuration." << endl;
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_resolution = K4A_COLOR_RESOLUTION_OFF;

	// Creat instances.
	cout << "Creating " << device_num << "kinect(s)' instance(s)." << endl;
	Eigen::Matrix3f m[2];
	m[0] << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	m[1] << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	for (int i = 0; i < device_num; i++) {
		several_kinects.emplace_back(QuaternionUDPTracker(i, m[i], config));
	}

}

void szl_kinect::SeveralQuaternionUDPTracker::initialize()
{
	for (int i = 0; i < device_num; i++) {
		several_kinects[i].Open();
		several_kinects[i].Initialize();
	}
}

int szl_kinect::SeveralQuaternionUDPTracker::Run(int max_frame)
{
	initialize();

	int frame_count = 0;
	do
	{
		bool succeeded = true;
		bool timeout = false;
		for (int i = 0; i < device_num; i++) {
			several_kinects[i].UpdateCapture();
			succeeded = succeeded && several_kinects[i].IsGetCaptureResultSucceeded();
			timeout = timeout || several_kinects[i].IsGetCaptureResultTimeout();
		}

		if (succeeded) {
			cout << "----------------" << "frame: " << frame_count << "----------------" << endl;
			frame_count++;

			bool is_queue_capture_good = true;
			for (int i = 0; i < device_num; i++) {
				several_kinects[i].UpdateTrackerEnqueue();
				is_queue_capture_good = is_queue_capture_good && several_kinects[i].IsQueueCaptureResultGood();
			}
			if (!is_queue_capture_good) break;

			for (int i = 0; i < device_num; i++) {
				several_kinects[i].UpdateBodyFrame();
			}

			//udp_sender->Send(skeleton_result);
		}
		else if (timeout) {
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else {
			for (int i = 0; i < device_num; i++) {
				printf("Get depth capture returned error, get_capture_result: %d\n", several_kinects[i].get_capture_result);
			}
			break;
		}

	} while (frame_count < max_frame || max_frame == -1);

	close();
	return 0;
}

void szl_kinect::SeveralQuaternionUDPTracker::close()
{
	for (int i = 0; i < device_num; i++) {
		several_kinects[i].Close();
	}
}

// 初始化 TrackerProcessor
// 只是简单地对 rotation_matrix 进行了赋值
szl_kinect::TrackerProcessor::TrackerProcessor(Eigen::Matrix3f view_rotation) : view_rotation(view_rotation), qua_rotation(qua_rotation)
{
}

// 接收 k4abt_skeleton_t 类型；返回对象本身
// 利用 BodyTracking API 计算得到的 skeleton，对临时变量 tmp 进行赋值
// 进而对 tmp 进行旋转和平移
// 最后把 tmp 的坐标值赋值给 skeleton_matrix
// 注意：skeleton_matrix 更新的仅仅是坐标值，四元数等信息并没有经过变换
szl_kinect::TrackerProcessor& szl_kinect::TrackerProcessor::FixView(k4abt_skeleton_t skeleton)
{
	using Eigen::Matrix3Xf;

	// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
	// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		skeleton_matrix.push_back(skeleton.joints[i]);
	}

	// 创建一个临时变量 tmp 用于保存关节点数据
	// 把 skeleton_matrix 的节点 3D 位置赋值给 tmp
	// 这里把 tmp(i, j) 中的 i 作为了 X、Y、Z，未来可能需要做优化
	Matrix3Xf tmp;
	tmp.resize(3, skeleton_matrix.size());
	for (int i = 0; i < skeleton_matrix.size(); i++) {
		tmp(0, i) = skeleton_matrix.at(i).position.xyz.x;
		tmp(1, i) = skeleton_matrix.at(i).position.xyz.y;
		tmp(2, i) = skeleton_matrix.at(i).position.xyz.z;
	}
	
	// 使用 TrackerProcessor 成员变量旋转矩阵对 tmp 进行旋转
	tmp = view_rotation * tmp;

	// 计算平移量
	// mHasShift 相当于开关，保证计算平移量的逻辑只进行一次。
	if (!mHasShift) {
		float sumX = 0;  // 用于保存所有节点的 X 值的和
		float sumY = 0;  // 用于保存所有节点的 Y 值的和
		float minZ = 9999;  // 用于保存所有节点的 Z 值得最小值
		for (int i = 0; i < tmp.cols(); i++) {
			sumX += tmp(0, i);
			sumY += tmp(1, i);

			if (minZ > tmp(2, i)) {
				minZ = tmp(2, i);
			}
		}
		shift_matrix << -sumX / skeleton_matrix.size(), -sumY / skeleton_matrix.size(), -minZ;  // 该平移矩阵是：-X平均值、-Y平均值、-Z最小值
		// 关闭开关，该代码块的逻辑不会再调用
		mHasShift = true;
	}

	// 依次给每个列向量加上位移量
	// 可优化
	for (int i = 0; i < tmp.cols(); i++) {
		tmp(0, i) += shift_matrix(0);
		tmp(1, i) += shift_matrix(1);
		tmp(2, i) += shift_matrix(2);
	}

	// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
	// 仅仅是坐标值，四元数等信息并没有经过调整
	assert(skeleton_matrix.size() == tmp.cols());
	for (int i = 0; i < skeleton_matrix.size(); i++) {
		skeleton_matrix[i].position.xyz.x = tmp(0, i);
		skeleton_matrix[i].position.xyz.y = tmp(1, i);
		skeleton_matrix[i].position.xyz.z = tmp(2, i);
	}

	return *this;
}

string szl_kinect::TrackerProcessor::ToString() {
	std::stringstream ss;
	vector<k4abt_joint_t>::iterator iter;

	// 生成坐标值，此时坐标已经经过变换
	for (iter = skeleton_matrix.begin(); iter != skeleton_matrix.end(); iter++) {
		ss << iter->position.xyz.x << " ";
		ss << iter->position.xyz.y << " ";
		ss << iter->position.xyz.z << ", ";
	}
	// 生成四元数，此时四元数未经过
	ss << "| ";
	for (iter = skeleton_matrix.begin(); iter != skeleton_matrix.end(); iter++) {
		Eigen::Quaternionf result = Eigen::Quaternionf(qua_rotation) * Eigen::Quaternionf(view_rotation) * Eigen::Quaternionf(iter->orientation.wxyz.w, iter->orientation.wxyz.x, iter->orientation.wxyz.y, iter->orientation.wxyz.z);
		ss << result.coeffs()(3, 0) << " ";
		ss << result.coeffs()(0, 0) << " ";
		ss << result.coeffs()(1, 0) << " ";
		ss << result.coeffs()(2, 0) << ", ";
	}
	return ss.str();
}

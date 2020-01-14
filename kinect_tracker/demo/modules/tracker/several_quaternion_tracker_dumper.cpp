#include "szl/several_quaternion_tracker_dumper.h"


inline void szl_kinect_dump::verify(k4a_result_t result, string msg, string error)
{
	if (result != K4A_RESULT_SUCCEEDED)
	{
		cout << error << endl;
		cout << " - (File: " << __FILE__ << ", Function: " << __FUNCTION__ << ", Line: " << __LINE__ << ")\n";
		exit(1);
	}
	cout << msg << endl;
}

// 初始化 TrackerProcessor
// 只是简单地对 rotation_matrix 和 qua_rotation 进行了赋值
szl_kinect_dump::TrackerProcessor::TrackerProcessor(Eigen::Matrix3f view_rotation, Eigen::Matrix3f qua_rotation) : view_rotation(view_rotation), qua_rotation(qua_rotation)
{
}

// 接收 k4abt_skeleton_t 类型；返回对象本身
// 利用 BodyTracking API 计算得到的 skeleton，对临时变量 tmp 进行赋值
// 进而对 tmp 进行旋转和平移
// 最后把 tmp 的坐标值赋值给 skeleton_matrix
// 注意：skeleton_matrix 更新的仅仅是坐标值，四元数等信息并没有经过变换
szl_kinect_dump::TrackerProcessor& szl_kinect_dump::TrackerProcessor::FixView(k4abt_skeleton_t* skeleton)
{
	using Eigen::Matrix3Xf;

	// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
	// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		skeleton_matrix.push_back(skeleton->joints[i]);
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
	for (int i = 0; i < KINECT_JOINT_NUM; i++) {
		skeleton->joints[i].position.xyz.x = tmp(0, i);
		skeleton->joints[i].position.xyz.y = tmp(1, i);
		skeleton->joints[i].position.xyz.z = tmp(2, i);
	}

	return *this;
}

// 旋转四元数
// 待优化
void szl_kinect_dump::TrackerProcessor::RotateQuaternion(k4abt_skeleton_t* skeleton)
{
	//for (int i = 0; i < KINECT_JOINT_NUM; i++) {
	//	Eigen::Quaternionf result = Eigen::Quaternionf(qua_rotation) * Eigen::Quaternionf(view_rotation) * 
	//		Eigen::Quaternionf(skeleton->joints[i].orientation.wxyz.w, skeleton->joints[i].orientation.wxyz.x, skeleton->joints[i].orientation.wxyz.y, skeleton->joints[i].orientation.wxyz.z);
	//	skeleton->joints[i].orientation.wxyz.w = result.coeffs()(3, 0);
	//	skeleton->joints[i].orientation.wxyz.x = result.coeffs()(0, 0);
	//	skeleton->joints[i].orientation.wxyz.y = result.coeffs()(1, 0);
	//	skeleton->joints[i].orientation.wxyz.z = result.coeffs()(2, 0);
	//}

	for (int i = 0; i < KINECT_JOINT_NUM; i++) {
		Eigen::Quaternionf result = Eigen::Quaternionf(view_rotation) * Eigen::Quaternionf(qua_rotation) *
		//Eigen::Quaternionf result = Eigen::Quaternionf(qua_rotation) * Eigen::Quaternionf(view_rotation) *
			Eigen::Quaternionf(skeleton->joints[i].orientation.wxyz.w, skeleton->joints[i].orientation.wxyz.x, skeleton->joints[i].orientation.wxyz.y, skeleton->joints[i].orientation.wxyz.z);
		skeleton->joints[i].orientation.wxyz.w = result.coeffs().w();
		skeleton->joints[i].orientation.wxyz.x = result.coeffs().x();
		skeleton->joints[i].orientation.wxyz.y = result.coeffs().y();
		skeleton->joints[i].orientation.wxyz.z = result.coeffs().z();
	}
}

string szl_kinect_dump::TrackerProcessor::ToString() {
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


szl_kinect_dump::QuaternionUDPTracker::QuaternionUDPTracker(int device_index, Eigen::Matrix3f rotation_matrix, Eigen::Matrix3f qua_matrix, k4a_device_configuration_t config): device_index(device_index), config(config), processor(TrackerProcessor(rotation_matrix, qua_matrix))
{
	cout << "Kinect " << device_index << " instance created.";
}

void szl_kinect_dump::QuaternionUDPTracker::Open()
{
	// opening two k4a device
	cout << device_index << ": Started opening K4A device..." << endl;
	verify(k4a_device_open(device_index, &device), "Open K4A Device succeed.", "Open K4A Device failed!");
	verify(k4a_device_start_cameras(device, &config), "Start K4A cameras succeed.", "Start K4A cameras failed!");
	cout << device_index << ": Finished opening K4A device." << endl;
}

void szl_kinect_dump::QuaternionUDPTracker::Initialize()
{
	// Sensor calibration.
	verify(k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &sensor_calibration),
		"Get calibration succeed.",
		"Get calibration failed!");

	tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
	verify(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization succeed.", "Body tracker initialization failed!");
}

void szl_kinect_dump::QuaternionUDPTracker::Close()
{
	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
}

// 设备深度帧的捕获
void szl_kinect_dump::QuaternionUDPTracker::UpdateCapture()
{
	get_capture_result = k4a_device_get_capture(device, &sensor_capture, 0);
}

bool szl_kinect_dump::QuaternionUDPTracker::IsGetCaptureResultSucceeded()
{
	return get_capture_result == K4A_WAIT_RESULT_SUCCEEDED;
}

bool szl_kinect_dump::QuaternionUDPTracker::IsGetCaptureResultTimeout()
{
	return get_capture_result == K4A_WAIT_RESULT_TIMEOUT;
}

void szl_kinect_dump::QuaternionUDPTracker::UpdateTrackerEnqueue()
{
	queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
	k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
}

bool szl_kinect_dump::QuaternionUDPTracker::IsQueueCaptureResultGood()
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

void szl_kinect_dump::QuaternionUDPTracker::UpdateBodyFrame()
{
	pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
		printf("%zu bodies are detected!\n", num_bodies);
		device_timestamp = k4abt_frame_get_device_timestamp_usec(body_frame);
		cout << "Device timestamp: " << device_timestamp << endl;
		// Get skeletons. We only extract the first skeleton
		if (num_bodies > 0)
		{
			k4a_result_t frame_get_body_skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);
			if (frame_get_body_skeleton_result == K4A_RESULT_SUCCEEDED)
			{
				//processor.FixView(&skeleton);
				processor.RotateQuaternion(&skeleton);
			}
			else if (frame_get_body_skeleton_result == K4A_RESULT_FAILED)
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

// 单个设备跟踪数据写入 CSV 文件
void szl_kinect_dump::QuaternionUDPTracker::WriteCSV(std::ofstream* out_file, int frame_id)
{
	for (int i = 0; i < KINECT_JOINT_NUM; i++) {
		*out_file << device_timestamp << ',' << device_index << ',' << frame_id << "," << i << ","
			<< skeleton.joints[i].position.xyz.x << "," << skeleton.joints[i].position.xyz.y << "," << skeleton.joints[i].position.xyz.z << ","
			<< skeleton.joints[i].orientation.wxyz.w << "," << skeleton.joints[i].orientation.wxyz.x << "," << skeleton.joints[i].orientation.wxyz.y << "," << skeleton.joints[i].orientation.wxyz.z
			<< endl;
	}
}

// 该函数用于创建一个目录，并返回该目录路径的字符串
static string szl_kinect_dump::makeDir()
{
	// 获取时间，作为本次保存的文件夹名
	tm ltm;
	time_t now;
	time(&now);
	localtime_s(&ltm, &now);
	int year = 1900 + ltm.tm_year;
	int month = 1 + ltm.tm_mon;
	int day = ltm.tm_mday;
	int hour = ltm.tm_hour;
	int min = ltm.tm_min;
	int sec = ltm.tm_sec;
	std::stringstream ss;
	ss << year << "-" << month << "-" << day << "-" << hour << "-" << min << "-" << sec;
	string save_dir = "save/" + ss.str();
	
	int i = 0;
	int iRet;
	int iLen;
	char* pszDir;

	pszDir = _strdup(save_dir.c_str());
	iLen = strlen(pszDir);

	// 创建中间目录
	for (i = 0; i < iLen; i++)
	{
		if (pszDir[i] == '\\' || pszDir[i] == '/')
		{
			pszDir[i] = '\0';

			//如果不存在,创建
			iRet = _access(pszDir, 0);
			if (iRet != 0)
			{
				iRet = _mkdir(pszDir);
				if (iRet != 0)
				{
					return "wrong";
				}
			}
			//支持linux,将所有\换成/
			pszDir[i] = '/';
		}
	}

	iRet = _mkdir(pszDir);
	free(pszDir);
	return save_dir;
}

// 构造函数
szl_kinect_dump::SeveralQuaternionTrackerDumper::SeveralQuaternionTrackerDumper(int device_num): device_num(device_num)
{

	// Set configuration.
	cout << "Setting " << device_num << "kinect(s)' configuration." << endl;
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_resolution = K4A_COLOR_RESOLUTION_OFF;

	// Creat instances.
	cout << "Creating " << device_num << "kinect(s)' instance(s)." << endl;
	// 硬编码旋转矩阵
	// 待优化
	Eigen::Matrix3f m[2];
	Eigen::Matrix3f m2[2];
	m[0] << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	m[1] << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	m2[0] << 0.98486496, -0.01640388, 0.17254544, 0.01373559, 0.99976708, 0.01664696, -0.17277833, -0.01402499, 0.98486088;
	//m2[0] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	m2[1] << 8.18877299e-01, 3.96199618e-02, -5.72599535e-01, -4.88580353e-02, 9.98805443e-01, -7.61618621e-04, 5.71885357e-01, 2.85997605e-02, 8.19834857e-01;
	//m2[1] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	//m2[1] << 0.67419386, -0.10978471, 0.7303492,0.09096784, 0.99370414, 0.06539825,-0.73293076, 0.02234719, 0.67993611;
	for (int i = 0; i < device_num; i++) {
		several_kinects.emplace_back(QuaternionUDPTracker(i, m[i], m2[i], config));
	}

}

// 依次开启和初始化设备
void szl_kinect_dump::SeveralQuaternionTrackerDumper::initialize()
{
	for (int i = 0; i < device_num; i++) {
		several_kinects[i].Open();
		several_kinects[i].Initialize();
	}
}

// 依次关闭设备
void szl_kinect_dump::SeveralQuaternionTrackerDumper::close()
{
	for (int i = 0; i < device_num; i++) {
		several_kinects[i].Close();
	}
}

// 该 Demo 的入口程序
int szl_kinect_dump::SeveralQuaternionTrackerDumper::Run(int max_frame)
{
	using std::ofstream;
	using std::ios;

	initialize();

	// 创建一个新目录，生成一个文件句柄 out_file
	string save_dir = makeDir();
	ofstream out_file;  // 写文件
	out_file.open(save_dir + "/data.csv", ios::out); // 打开模式可省略
	out_file << "device_timestamp," << "device_id," << "frame_id," << "node_id,"
		<< "x," << "y," << "z,"
		<< "qua_w," << "qua_x," << "qua_y," << "qua_z" << endl;

	int frame_count = 0;
	while (frame_count < max_frame || max_frame == -1)
	{
		bool succeeded = true;
		bool timeout = false;
		for (int i = 0; i < device_num; i++) {
			several_kinects[i].UpdateCapture();
			succeeded = succeeded && several_kinects[i].IsGetCaptureResultSucceeded();
			timeout = timeout || several_kinects[i].IsGetCaptureResultTimeout();
		}

		if (succeeded) {
			// 如果一切正常，会到达这一步
			cout << "----------------" << "frame: " << frame_count << "----------------" << endl;
			frame_count++;

			bool is_queue_capture_good = true;
			for (int i = 0; i < device_num; i++) {
				several_kinects[i].UpdateTrackerEnqueue();
				is_queue_capture_good = is_queue_capture_good && several_kinects[i].IsQueueCaptureResultGood();
			}
			if (!is_queue_capture_good) break;

			for (int i = 0; i < device_num; i++) {
				several_kinects[i].UpdateBodyFrame();  // 调用 BodyTracking 更新结果
				several_kinects[i].WriteCSV(&out_file, frame_count);  // 将结果写入到 CSV 文件
			}
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

	}

	close();
	out_file.close();  // 关闭 CSV 文件
	return 0;
}



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

szl_kinect::TrackerProcessor::TrackerProcessor(Eigen::Matrix3f rotation_matrix) : rotation_matrix(rotation_matrix), mHasShift(false)
{
}

szl_kinect::TrackerProcessor& szl_kinect::TrackerProcessor::FixView(k4abt_skeleton_t skeleton)
{
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		skeleton_matrix.push_back(skeleton.joints[i]);
	}

	// Get skeleton matrix
	Eigen::Matrix3Xf tmp;
	tmp.resize(3, skeleton_matrix.size());
	for (int i = 0; i < skeleton_matrix.size(); i++) {
		tmp(0, i) = skeleton_matrix.at(i).position.xyz.x;
		tmp(1, i) = skeleton_matrix.at(i).position.xyz.y;
		tmp(2, i) = skeleton_matrix.at(i).position.xyz.z;
	}
	// Rotation
	tmp = rotation_matrix * tmp;

	// Get the translation shift
	if (!mHasShift) {
		// Initialize mShift
		float sumX = 0;
		float sumY = 0;
		float minZ = 9999;
		for (int i = 0; i < tmp.cols(); i++) {
			sumX += tmp(0, i);
			sumY += tmp(1, i);

			if (minZ > tmp(2, i)) {
				minZ = tmp(2, i);
			}
		}
		shift_matrix << -sumX / skeleton_matrix.size(), -sumY / skeleton_matrix.size(), -minZ;
		mHasShift = true;
	}

	// Translation
	for (int i = 0; i < tmp.cols(); i++) {
		tmp(0, i) += shift_matrix(0);
		tmp(1, i) += shift_matrix(1);
		tmp(2, i) += shift_matrix(2);
	}

	// Make change happen
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
	for (iter = skeleton_matrix.begin(); iter != skeleton_matrix.end(); iter++) {
		ss << iter->position.xyz.x << " ";
		ss << iter->position.xyz.y << " ";
		ss << iter->position.xyz.z << ", ";
	}
	// Output rotation
	ss << "| ";
	for (iter = skeleton_matrix.begin(); iter != skeleton_matrix.end(); iter++) {
		Eigen::Quaternionf result = Eigen::Quaternionf(rotation_matrix) * Eigen::Quaternionf(iter->orientation.wxyz.w, iter->orientation.wxyz.x, iter->orientation.wxyz.y, iter->orientation.wxyz.z);
		ss << result.coeffs()(3, 0) << " ";
		ss << result.coeffs()(0, 0) << " ";
		ss << result.coeffs()(1, 0) << " ";
		ss << result.coeffs()(2, 0) << ", ";
	}
	return ss.str();
}

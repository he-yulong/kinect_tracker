#include "szl/double_udp_tracker.h"
using szl_kinect::DoubleUDPTracker;

#include <iostream>
using std::string;
//#include <gflags/gflags.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include "szl/udp_sender.h"
//#include "szl/single_skeleton_processor.h"
#include "szl/quaternion_skeleton_processor.h"

#define VERIFY(result, error)																				\
	if (result != K4A_RESULT_SUCCEEDED)																		\
	{																										\
		printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__);	\
		exit(1);																							\
	}																										\

//// GFLAGS definitions
//DEFINE_string(udp_ip, "127.0.0.1", "IP address for udp transmission");
//DEFINE_int32(udp_port, 8999, "Port for udp transmission");
//DEFINE_bool(all_joints, true, "If output all joints");

int DoubleUDPTracker::Run(string udp_ip_sub, string udp_ip_master, int udp_port_sub, int udp_port_master, bool all_joints, int max_frame)
{
	cout << udp_port_sub << ";" << udp_port_master << endl;
	//// Parse args
	//gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Initialize UDP sender
	//UDPSender udpSender(FLAGS_udp_ip, FLAGS_udp_port);
	UDPSender udp_sender_sub = UDPSender(udp_ip_sub, udp_port_sub);
	UDPSender udp_sender_master = UDPSender(udp_ip_master, udp_port_master);

	// Set configuration
	// k4a_device_configuration_t: Configuration parameters for an Azure Kinect device.
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	//config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.color_resolution = K4A_COLOR_RESOLUTION_OFF;

	// opening two k4a device
	cout << "Started opening K4A device..." << endl;

	k4a_device_t dev_sub = NULL;
	VERIFY(k4a_device_open(1, &dev_sub), "Open K4A Device 1(Subordinate) succeed.", "Open K4A Device 1(Subordinate) failed!");
	VERIFY(k4a_device_start_cameras(dev_sub, &config), "Start K4A cameras 1(Subordinate) succeed.", "Start K4A cameras 1(Subordinate) failed!");

	k4a_device_t dev_master = NULL;
	VERIFY(k4a_device_open(K4A_DEVICE_DEFAULT, &dev_master), "Open K4A Device 0(Master) succeed.", "Open K4A Device 0(Master) failed!");
	VERIFY(k4a_device_start_cameras(dev_master, &config), "Start K4A cameras 0(Master) succeed.", "Start K4A cameras 0(Master) failed!");

	cout << "Finished opening K4A device." << endl;

	// sensor calibration
	k4a_calibration_t sensor_calibration_sub;
	VERIFY(k4a_device_get_calibration(dev_sub, config.depth_mode, config.color_resolution, &sensor_calibration_sub),
		"Get Subordinate's calibration succeed.",
		"Get Subordinate's calibration failed!");

	k4a_calibration_t sensor_calibration_master;
	VERIFY(k4a_device_get_calibration(dev_master, config.depth_mode, config.color_resolution, &sensor_calibration_master),
		"Get Master's calibration succeed.",
		"Get Master's calibration failed!");

	cout << "calibration sobordinate: " << sensor_calibration_sub.extrinsics << endl;
	cout << "calibration master: " << &sensor_calibration_master.extrinsics << endl;

	// tracker
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;

	k4abt_tracker_t tracker_sub = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_sub, tracker_config, &tracker_sub), "Body tracker(Subordinate) initialization succeed.", "Body tracker(Subordinate) initialization failed!");
	k4abt_tracker_t tracker_master = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_master, tracker_config, &tracker_master), "Body tracker(Master) initialization succeed.", "Body tracker(Master) initialization failed!");

	int frame_count = 0;
	// k4a_wait_result_t: Result code returned by Azure Kinect APIs.
	k4a_wait_result_t get_capture_result_sub;
	k4a_wait_result_t get_capture_result_master;
	// k4a_capture_t: Handle to an Azure Kinect capture.
	k4a_capture_t sensor_capture_sub;
	k4a_capture_t sensor_capture_master;

	Matrix3f stereo_rotate;
	stereo_rotate << 0.26021786, -0.00209256, 0.96554766, 0.10242372, 0.99441529, -0.02544836, -0.9601021, 0.1055171, 0.25897894;

	do
	{
		get_capture_result_sub = k4a_device_get_capture(dev_sub, &sensor_capture_sub, 0);
		get_capture_result_master = k4a_device_get_capture(dev_master, &sensor_capture_master, 0);

		if (get_capture_result_sub == K4A_WAIT_RESULT_SUCCEEDED && get_capture_result_master == K4A_WAIT_RESULT_SUCCEEDED) {
			cout << "----------------" << "frame: " << frame_count << "----------------" << endl;
			frame_count++;
			k4a_wait_result_t queue_capture_result_sub = k4abt_tracker_enqueue_capture(tracker_sub, sensor_capture_sub, K4A_WAIT_INFINITE);
			k4a_wait_result_t queue_capture_result_master = k4abt_tracker_enqueue_capture(tracker_master, sensor_capture_master, K4A_WAIT_INFINITE);
			k4a_capture_release(sensor_capture_sub); // Remember to release the sensor capture once you finish using it
			k4a_capture_release(sensor_capture_master); // Remember to release the sensor capture once you finish using it

			if (queue_capture_result_sub == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!(Subordinate)\n");
				break;
			}
			else if (queue_capture_result_sub == K4A_WAIT_RESULT_FAILED)
			{
				printf("Error! Add capture to tracker process queue failed!(Subordinate)\n");
				break;
			}

			if (queue_capture_result_master == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!(Master)\n");
				break;
			}
			else if (queue_capture_result_master == K4A_WAIT_RESULT_FAILED)
			{
				printf("Error! Add capture to tracker process queue failed!(Master)\n");
				break;
			}

			k4abt_frame_t body_frame_sub = NULL;
			// body_frame_sub: Handle to a k4a body tracking frame.
			k4a_wait_result_t pop_frame_result_sub = k4abt_tracker_pop_result(tracker_sub, &body_frame_sub, K4A_WAIT_INFINITE);

			vector<k4abt_joint_t> mSkeleton0;
			if (pop_frame_result_sub == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing

				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame_sub);
				printf("%zu bodies are detected!(Subordinate)\n", num_bodies);
				cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame_sub) << endl;

				// Get skeletons. We only extract the first skeleton
				if (num_bodies > 0)
				{
					// k4abt_skeleton_t: Structure to define joints for skeleton.
					// --> k4abt_joint_t 	joints[K4ABT_JOINT_COUNT]
					// k4abt_joint_t: Structure to define a single joint.
					// --> k4a_float3_t/k4a_quaternion_t/k4abt_joint_confidence_level_t
					k4abt_skeleton_t skeleton1;
					// k4a_result_t: Result code returned by Azure Kinect APIs.
					// --> K4A_RESULT_SUCCEEDED/K4A_RESULT_FAILED
					k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame_sub, 0, &skeleton1);
					if (skeleton_result == K4A_RESULT_SUCCEEDED)
					{
						/*P1: !!opencv - matrix
						rows : 3
						cols : 4
						dt : d
						data : [9.6122983480071571e+02, 0., 3.2602759008407593e+03, 0., 0.,
						9.6122983480071571e+02, 5.7175109815597534e+02, 0., 0., 0., 1.,
						0.]*/
						//for (int i = 0; i < 32; i++) {
						//	float x = 9.6122983480071571e+02 * skeleton.joints[i].position.xyz.x + 0 + 3.2602759008407593e+03 * skeleton.joints[i].position.xyz.z + 0;
						//	float y = 0 + 9.6122983480071571e+02 * skeleton.joints[i].position.xyz.y + 5.7175109815597534e+02 * skeleton.joints[i].position.xyz.z + 0;
						//	float z = 0 + 0 + 0 + 1;
						//	skeleton.joints[i].position.xyz.x = x / 10000.;
						//	skeleton.joints[i].position.xyz.y = y / 10000.;
						//	skeleton.joints[i].position.xyz.z = z;
						//}
						//cout << skeleton.joints[2].position.xyz.x << endl;
						
						// Successfully get skeleton for the i-th person. Start processingS
						//SkeletonProcessor processor(skeleton);
						QuaternionSkeletonProcessor processor(skeleton1);

						// Output joints
						string skeleton_result = "";

						
						//if (FLAGS_all_joints) {
						if (all_joints) {
							skeleton_result = processor.FixView().ToString();
						}
						else {
							skeleton_result = processor.ToUnity().FixView().ToString();
						}

						mSkeleton0 = processor.mSkeleton;

						// Send results with udp
						cout << "------sub skeleton result-------" << endl;
						udp_sender_sub.Send(skeleton_result);
					}
					else if (skeleton_result == K4A_RESULT_FAILED)
					{
						printf("Error! Get body skeleton failed!\n");
					}
				}

				//string test_string = "30.2037 31.2427 2169.18, 27.9222 -142.545 2131.87, 30.8757 -284.333 2122.78, 47.5984 -500.006 2123.14, 67.3322 -464.324 2133.66, 201.348 -508.926 2115.27, 467.89 -453.058 2155.86, 695.445 -440.858 2117.44, 789.354 -439.368 2137.04, 857.734 -398.234 2209.31, 816.508 -370.859 2154.33, 16.4992 -466.71 2135.19, -108.943 -504.287 2139.92, -377.026 -498.867 2222.45, -609.891 -519.221 2211.22, -700.485 -490.086 2251.42, -768.522 -439.3 2314.6, -697.98 -414.344 2265.24, 121.02 30.577 2166.72, 129.173 420.204 2251.78, 137.229 771.952 2398.52, 178.519 882.391 2253.81, -51.6895 31.8464 2171.39, -60.0051 423.201 2245.82, -51.6256 783.78 2381.13, -93.6894 880.428 2245.54, 51.5642 -575.258 2092.38, 46.1344 -601.179 1934.48, 83.678 -635.105 1957.93, 145.385 -642.888 2067.87, 14.3434 -634.819 1957.18, -44.077 -643.345 2073.61,";
				//SkeletonProcessor processor;
				//string result = processor.FromString(test_string).ToUnity().FixView().ToString();
				//udpSender.Send(result);
				k4abt_frame_release(body_frame_sub); // Remember to release the body frame once you finish using it
			}
			else if (pop_frame_result_sub == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!(Subordinate)\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!(Subordinate)\n");
				break;
			}

			k4abt_frame_t body_frame_master = NULL;
			k4a_wait_result_t pop_frame_result_master = k4abt_tracker_pop_result(tracker_master, &body_frame_master, K4A_WAIT_INFINITE);
			if (pop_frame_result_master == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing

				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame_master);
				printf("%zu bodies are detected!(Master)\n", num_bodies);
				cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame_master) << endl;

				// Get skeletons. We only extract the first skeleton
				if (num_bodies > 0)
				{
					k4abt_skeleton_t skeleton2;
					k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame_master, 0, &skeleton2);
					if (skeleton_result == K4A_RESULT_SUCCEEDED)
					{
						

						/*P2: !!opencv - matrix
						   rows: 3
						   cols: 4
						   dt: d
						   data: [ 9.6122983480071571e+02, 0., 3.2602759008407593e+03,
							   -1.8990038594056032e+03, 0., 9.6122983480071571e+02,
							   5.7175109815597534e+02, 0., 0., 0., 1., 0. ]
						*/
						//for (int i = 0; i < 32; i++) {
						//	float x = 9.6122983480071571e+02 * skeleton.joints[i].position.xyz.x + 0 + 3.2602759008407593e+03 * skeleton.joints[i].position.xyz.z + -1.8990038594056032e+03;
						//	float y = 0 + 9.6122983480071571e+02 * skeleton.joints[i].position.xyz.y + 5.7175109815597534e+02 * skeleton.joints[i].position.xyz.z + 0;
						//	float z = 0 + 0 + skeleton.joints[i].position.xyz.z + 0;
						//	skeleton.joints[i].position.xyz.x = x / 10000.;
						//	skeleton.joints[i].position.xyz.y = y / 10000.;
						//	skeleton.joints[i].position.xyz.z = z;
						//}
						//cout << skeleton.joints[2].position.xyz.x << endl;

						// Successfully get skeleton for the i-th person. Start processingS
						//SkeletonProcessor processor(skeleton);
						QuaternionSkeletonProcessor processor(skeleton2);

						// Output joints
						string skeleton_result = "";
						
						//if (FLAGS_all_joints) {
						if (all_joints) {
							skeleton_result = processor.ToString(mSkeleton0, stereo_rotate);
						}
						else {
							skeleton_result = processor.ToUnity().FixView().ToString(mSkeleton0);
						}

						// Send results with udp
						cout << "------master skeleton result-------" << endl;
						udp_sender_master.Send(skeleton_result);
					}
					else if (skeleton_result == K4A_RESULT_FAILED)
					{
						printf("Error! Get body skeleton failed!\n");
					}
				}

				k4abt_frame_release(body_frame_master); // Remember to release the body frame once you finish using it
			}
			else if (pop_frame_result_master == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!(Master)\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!(Master)\n");
				break;
			}
		}
		else if (get_capture_result_sub == K4A_WAIT_RESULT_TIMEOUT) {
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Subordinate Error! Get depth frame time out!\n");
			break;
		}
		else if (get_capture_result_master == K4A_WAIT_RESULT_TIMEOUT) {
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Master Error! Get depth frame time out!\n");
			break;
		}
		else {
			printf("Get depth capture returned error, get_capture_result_sub: %d\n", get_capture_result_sub);
			printf("Get depth capture returned error, get_capture_result_master: %d\n", get_capture_result_master);
			break;
		}

	} while (frame_count < max_frame || max_frame == -1);

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker_sub);
	k4abt_tracker_destroy(tracker_sub);
	k4a_device_stop_cameras(dev_sub);
	k4a_device_close(dev_sub);

	k4abt_tracker_shutdown(tracker_master);
	k4abt_tracker_destroy(tracker_master);
	k4a_device_stop_cameras(dev_master);
	k4a_device_close(dev_master);

	return 0;
}

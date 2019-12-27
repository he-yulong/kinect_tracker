#include "szl/single_udp_tracker.h"
using szl_kinect::SingleUDPTracker;

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

int SingleUDPTracker::Run(string udp_ip, int udp_port, bool all_joints)
{
	//// Parse args
	//gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Initialize UDP sender
	//UDPSender udpSender(FLAGS_udp_ip, FLAGS_udp_port);
	UDPSender udpSender(udp_ip, udp_port);

	k4a_device_t device = NULL;
	k4a_device_open(1, &device);

	// Strart Camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration), "Get depth camera calibration failed!");

	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

	int frame_count = 0;
	do
	{
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			frame_count++;
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
			k4a_capture_release(sensor_capture);	// Remember to release the sensor capture once you finish using it
			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!\n");
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
			{
				printf("Error! Add capture to tracker process queue failed!\n");
				break;
			}

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%zu bodies are detected!\n", num_bodies);

				// Get skeletons. We only extract the first skeleton
				if (num_bodies > 0)
				{
					k4abt_skeleton_t skeleton;
					k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);
					if (skeleton_result == K4A_RESULT_SUCCEEDED)
					{
						// Successfully get skeleton for the i-th person. Start processingS
						//SkeletonProcessor processor(skeleton);
						QuaternionSkeletonProcessor processor(skeleton);

						// Output joints
						string skeleton_result = "";
						//if (FLAGS_all_joints) {
						if (all_joints) {
							cout << "????????" << processor.ToString(processor.mSkeleton) << "???????????" << endl;

							skeleton_result = processor.FixView().ToString(processor.mSkeleton);

							cout << "????????" << skeleton_result << "???????????" << endl;
							cout << "???????????????????" << endl;
						}
						else {
							skeleton_result = processor.ToUnity().FixView().ToString(processor.mSkeleton);
						}

						// Send results with udp
						udpSender.Send(skeleton_result);
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

				k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!\n");
				break;
			}
		}
		else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else
		{
			printf("Get depth capture returned error: %d\n", get_capture_result);
			break;
		}
	} while (true);

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	return 0;
}

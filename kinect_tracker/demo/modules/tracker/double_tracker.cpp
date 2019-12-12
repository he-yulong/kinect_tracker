#include "szl/double_tracker.h"
using szl_kinect::DoubleTracker;

#include <iostream>
using namespace std;
using std::printf;

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <k4a/k4a.h>
#include <k4abt.h>

#define VERIFY(result, msg, error)                                                                       \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \
	printf("%s \n", msg);																				 \


int DoubleTracker::RunWithSimpleInformation(int max_frame)
{
	// set configuration
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	//config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	//config.synchronized_images_only = true;

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

	// tracker
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	k4abt_tracker_t tracker_sub = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_sub, tracker_config, &tracker_sub), "Body tracker(Subordinate) initialization succeed.", "Body tracker(Subordinate) initialization failed!");
	k4abt_tracker_t tracker_master = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_master, tracker_config, &tracker_master), "Body tracker(Master) initialization succeed.", "Body tracker(Master) initialization failed!");

	int frame_count = 0;
	k4a_wait_result_t get_capture_result_sub;
	k4a_wait_result_t get_capture_result_master;
	k4a_capture_t sensor_capture_sub;
	k4a_capture_t sensor_capture_master;

	do
	{
		get_capture_result_sub = k4a_device_get_capture(dev_master, &sensor_capture_sub, 0);
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
			} else if (queue_capture_result_sub == K4A_WAIT_RESULT_FAILED)
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
			k4a_wait_result_t pop_frame_result_sub = k4abt_tracker_pop_result(tracker_sub, &body_frame_sub, K4A_WAIT_INFINITE);
			if (pop_frame_result_sub == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing

				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame_sub);
				printf("%zu bodies are detected!(Subordinate)\n", num_bodies);
				cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame_sub) << endl;
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

	printf("over.\n");

	return 0;
}

void print_double_body_information(k4abt_body_t body)
{
	printf("Body ID: %u\n", body.id);
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
	{
		k4a_float3_t position = body.skeleton.joints[i].position;
		k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
		k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[i].confidence_level;
		printf("Joint[%d]: Position[mm] ( %f, %f, %f ); Orientation ( %f, %f, %f, %f); Confidence Level (%d) \n",
			i, position.v[0], position.v[1], position.v[2], orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3], confidence_level);
	}
}

void print_double_body_index_map_middle_line(k4a_image_t body_index_map)
{
	uint8_t* body_index_map_buffer = k4a_image_get_buffer(body_index_map);

	// Given body_index_map pixel type should be uint8, the stride_byte should be the same as width
	// TODO: Since there is no API to query the byte-per-pixel information, we have to compare the width and stride to
	// know the information. We should replace this assert with proper byte-per-pixel query once the API is provided by
	// K4A SDK.
	assert(k4a_image_get_stride_bytes(body_index_map) == k4a_image_get_width_pixels(body_index_map));

	int middle_line_num = k4a_image_get_height_pixels(body_index_map) / 2;
	body_index_map_buffer = body_index_map_buffer + middle_line_num * k4a_image_get_width_pixels(body_index_map);

	printf("BodyIndexMap at Line %d:\n", middle_line_num);
	for (int i = 0; i < k4a_image_get_width_pixels(body_index_map); i++)
	{
		printf("%u, ", *body_index_map_buffer);
		body_index_map_buffer++;
	}
	printf("\n");
}

int DoubleTracker::RunWithDetailedInformation(int max_frame)
{

	// set configuration
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	//config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	//config.synchronized_images_only = true;

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

	// tracker
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	k4abt_tracker_t tracker_sub = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_sub, tracker_config, &tracker_sub), "Body tracker(Subordinate) initialization succeed.", "Body tracker(Subordinate) initialization failed!");
	k4abt_tracker_t tracker_master = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration_master, tracker_config, &tracker_master), "Body tracker(Master) initialization succeed.", "Body tracker(Master) initialization failed!");

	int frame_count = 0;
	k4a_wait_result_t get_capture_result_sub;
	k4a_wait_result_t get_capture_result_master;
	k4a_capture_t sensor_capture_sub;
	k4a_capture_t sensor_capture_master;

	do
	{
		get_capture_result_sub = k4a_device_get_capture(dev_master, &sensor_capture_sub, 0);
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
			k4a_wait_result_t pop_frame_result_sub = k4abt_tracker_pop_result(tracker_sub, &body_frame_sub, K4A_WAIT_INFINITE);
			if (pop_frame_result_sub == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame_sub);
				printf("%zu bodies are detected!(Subordinate)\n", num_bodies);

				for (size_t i = 0; i < num_bodies; i++)
				{
					k4abt_body_t body;
					VERIFY(k4abt_frame_get_body_skeleton(body_frame_sub, i, &body.skeleton), "Get body from body frame succeed.", "Get body from body frame failed!");
					body.id = k4abt_frame_get_body_id(body_frame_sub, i);

					cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame_sub) << endl;
					print_double_body_information(body);
				}

				k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame_sub);
				if (body_index_map != NULL)
				{
					//print_double_body_index_map_middle_line(body_index_map);
					k4a_image_release(body_index_map);
				}
				else
				{
					printf("Error: Fail to generate bodyindex map!\n");
				}

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

				for (size_t i = 0; i < num_bodies; i++)
				{
					k4abt_body_t body;
					VERIFY(k4abt_frame_get_body_skeleton(body_frame_master, i, &body.skeleton), "Get body from body frame succeed.", "Get body from body frame failed!");
					body.id = k4abt_frame_get_body_id(body_frame_master, i);

					cout << "Device timestamp: " << k4abt_frame_get_device_timestamp_usec(body_frame_master) << endl;
					print_double_body_information(body);
				}

				k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame_master);
				if (body_index_map != NULL)
				{
					//print_double_body_index_map_middle_line(body_index_map);
					k4a_image_release(body_index_map);
				}
				else
				{
					printf("Error: Fail to generate bodyindex map!\n");
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
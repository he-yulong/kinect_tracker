#include "kinect.h"

Kinect::Kinect(int device_index_val) : device_index(device_index_val), device_config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL), tracker(NULL)
{
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
}

void Kinect::Open()
{
	verify(k4a_device_open(device_index, &device), "Open K4A Device succeed.", "Open K4A Device failed!");
	verify(k4a_device_start_cameras(device, &device_config), "Start K4A cameras succeed.", "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	verify(k4a_device_get_calibration(device, device_config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration),
		"Get depth camera calibration succeed.",
		"Get depth camera calibration failed!");

	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	verify(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization succeed.", "Body tracker initialization failed!");
}

void Kinect::PrintBodyOrientation(int max_frame)
{
	int frame_count = 0;
	do
	{
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, 3000);

		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			frame_count++;

			printf("Start processing frame %d\n", frame_count);

			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

			k4a_capture_release(sensor_capture);
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
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%zu bodies are detected!\n", num_bodies);

				for (size_t i = 0; i < num_bodies; i++)
				{
					k4abt_body_t body;
					verify(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame succeed.", "Get body from body frame failed!");
					body.id = k4abt_frame_get_body_id(body_frame, i);

					print_body_information(body);
				}

				//k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
				//if (body_index_map != NULL)
				//{
				//	print_body_index_map_middle_line(body_index_map);
				//	k4a_image_release(body_index_map);
				//}
				//else
				//{
				//	printf("Error: Fail to generate bodyindex map!\n");
				//}

				k4abt_frame_release(body_frame);
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
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

	} while (frame_count < max_frame || max_frame == -1);
}

void Kinect::Close()
{
	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
}

void Kinect::print_body_information(k4abt_body_t body)
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

void Kinect::print_body_index_map_middle_line(k4a_image_t body_index_map)
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
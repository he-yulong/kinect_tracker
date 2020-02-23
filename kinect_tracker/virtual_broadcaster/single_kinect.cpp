#include "single_kinect.h"

// 初始化私有成员
SingleKinect::SingleKinect(int device_index_val) : 
    device_index(device_index_val), 
    device(nullptr), 
    device_config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
    sensor_calibration({}),
    point_cloud_generator({ sensor_calibration }),
    floor_detector({}),
    sensor_capture(nullptr),
    tracker(NULL)
{}

// 打开并启动 kinect
void SingleKinect::Open()
{
    verify(k4a_device_open(0, &device), "Open K4A Device succeed!", "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    verify(k4a_device_start_cameras(device, &device_config), "Start K4A cameras succeed!", "Start K4A cameras failed!");

    // Get calibration information.
    verify(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration),
        "Get depth camera calibration succeed!",
        "Get depth camera calibration failed!");

    // Start imu for gravity vector.
    verify(k4a_device_start_imu(device), "Start IMU succeed!", "Start IMU failed!");
}

void SingleKinect::Running(int max_frame)
{
    int frame_count = 0;
    // 不断处理每一帧
    while (frame_count < max_frame || max_frame == -1)
    {
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, 3000); // timeout_in_ms is set to 3000

        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t depth_image = k4a_capture_get_depth_image(sensor_capture);

            // Capture an IMU sample for sensor orientation.
            k4a_imu_sample_t imu_sample;
            if (k4a_device_get_imu_sample(device, &imu_sample, 0) == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Update point cloud.
                point_cloud_generator.Update(depth_image);

                // Get down-sampled cloud points.
                const int downsampleStep = 2;
                const auto& cloudPoints = point_cloud_generator.GetCloudPoints(downsampleStep);

                // Detect floor plane based on latest visual and inertial observations.
                const size_t minimumFloorPointCount = 1024 / (downsampleStep * downsampleStep);
                const auto& maybeFloorPlane = floor_detector.TryDetectFloorPlane(cloudPoints, imu_sample, sensor_calibration, minimumFloorPointCount);

                // Visualize point cloud.
                //window3d.UpdatePointClouds(depthImage);

                // Visualize the floor plane.
                if (maybeFloorPlane.has_value())
                {
                    // For visualization purposes, make floor origin the projection of a point 1.5m in front of the camera.
                    Samples::Vector cameraOrigin = { 0, 0, 0 };
                    Samples::Vector cameraForward = { 0, 0, 1 };

                    auto p = maybeFloorPlane->ProjectPoint(cameraOrigin) + maybeFloorPlane->ProjectVector(cameraForward) * 1.5f;
                    auto n = maybeFloorPlane->Normal;

                    std::cout << n.X << std::endl;
                    //window3d.SetFloorRendering(true, p.X, p.Y, p.Z, n.X, n.Y, n.Z);
                }
                else
                {
                    //window3d.SetFloorRendering(false, 0, 0, 0);
                }
            }

            // Release the sensor capture and depth image once they are no longer needed.
            k4a_capture_release(sensor_capture);
            k4a_image_release(depth_image);

        }
        else if (get_capture_result != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << get_capture_result << std::endl;
            break;
        }
    } 
}

void SingleKinect::Close()
{
    printf("Finished body tracking processing!\n");

    //k4abt_tracker_shutdown(tracker);
    //k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_stop_imu(device);
    k4a_device_close(device);
}

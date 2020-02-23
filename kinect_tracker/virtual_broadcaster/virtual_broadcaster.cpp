// virtual_broadcaster.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include <k4a/k4a.h>

#include "FloorDetector.h"
#include "Utilities.h"
#include "PointCloudGenerator.h"
//#include "Window3dWrapper.h"

#include "single_kinect.h"


// Global State and Key Process Function
//bool s_isRunning = true;
//
//int64_t ProcessKey(void* /*context*/, int key)
//{
//    // https://www.glfw.org/docs/latest/group__keys.html
//    switch (key)
//    {
//        // Quit
//    case GLFW_KEY_ESCAPE:
//        s_isRunning = false;
//        break;
//    case GLFW_KEY_H:
//        PrintAppUsage();
//        break;
//    }
//    return 1;
//}
//
//int64_t CloseCallback(void* /*context*/)
//{
//    s_isRunning = false;
//    return 1;
//}

//void patch()
//{
//    head_pos = RootPatch.get_pos(pos, 'HEAD')
//    chest_pos = RootPatch.get_pos(pos, 'SPINE_CHEST')
//    root_pos = RootPatch.get_pos(pos, 'PELVIS')
//    naval_pos = RootPatch.get_pos(pos, 'SPINE_NAVAL')
//    lhip_pos = RootPatch.get_pos(pos, 'HIP_LEFT')
//    rhip_pos = RootPatch.get_pos(pos, 'HIP_RIGHT')
//
//    # 计算向量head to chest
//    vec_head2chest = RootPatch.get_normal_vector(head_pos, chest_pos)
//    # 计算原chest到root长度以及chest到naval长度
//    len_chest2root = np.linalg.norm(root_pos - chest_pos)
//    len_chest2naval = np.linalg.norm(naval_pos - chest_pos)
//    # 计算新的坐标
//    new_root_pos = chest_pos + len_chest2root * vec_head2chest
//    new_naval_pos = chest_pos + len_chest2naval * vec_head2chest
//    new_lhip_pos = lhip_pos - root_pos + new_root_pos
//    new_rhip_pos = rhip_pos - root_pos + new_root_pos
//
//    # 覆盖旧坐标
//    pos[KINECT_SKELETON['PELVIS']] = new_root_pos
//    pos[KINECT_SKELETON['SPINE_NAVAL']] = new_naval_pos
//    pos[KINECT_SKELETON['HIP_LEFT']] = new_lhip_pos
//    pos[KINECT_SKELETON['HIP_RIGHT']] = new_rhip_pos
//
//    return pos
//}


int main()
{
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information.
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");

    // Start imu for gravity vector.
    VERIFY(k4a_device_start_imu(device), "Start IMU failed!");

    //// Initialize the 3d window controller.
    //Window3dWrapper window3d;
    //window3d.Create("3D Visualization", sensorCalibration);
    //window3d.SetCloseCallback(CloseCallback);
    //window3d.SetKeyCallback(ProcessKey);

    // PointCloudGenerator for floor estimation.
    Samples::PointCloudGenerator pointCloudGenerator{ sensorCalibration };
    Samples::FloorDetector floorDetector;

    //while (s_isRunning)
    //{
    //    k4a_capture_t sensorCapture = nullptr;
    //    k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

    //    if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
    //    {
    //        k4a_image_t depthImage = k4a_capture_get_depth_image(sensorCapture);

    //        // Capture an IMU sample for sensor orientation.
    //        k4a_imu_sample_t imu_sample;
    //        if (k4a_device_get_imu_sample(device, &imu_sample, 0) == K4A_WAIT_RESULT_SUCCEEDED)
    //        {
    //            // Update point cloud.
    //            pointCloudGenerator.Update(depthImage);

    //            // Get down-sampled cloud points.
    //            const int downsampleStep = 2;
    //            const auto& cloudPoints = pointCloudGenerator.GetCloudPoints(downsampleStep);

    //            // Detect floor plane based on latest visual and inertial observations.
    //            const size_t minimumFloorPointCount = 1024 / (downsampleStep * downsampleStep);
    //            const auto& maybeFloorPlane = floorDetector.TryDetectFloorPlane(cloudPoints, imu_sample, sensorCalibration, minimumFloorPointCount);

    //            // Visualize point cloud.
    //            //window3d.UpdatePointClouds(depthImage);

    //            // Visualize the floor plane.
    //            if (maybeFloorPlane.has_value())
    //            {
    //                // For visualization purposes, make floor origin the projection of a point 1.5m in front of the camera.
    //                Samples::Vector cameraOrigin = { 0, 0, 0 };
    //                Samples::Vector cameraForward = { 0, 0, 1 };

    //                auto p = maybeFloorPlane->ProjectPoint(cameraOrigin) + maybeFloorPlane->ProjectVector(cameraForward) * 1.5f;
    //                auto n = maybeFloorPlane->Normal;

    //                std::cout << n.X << std::endl;
    //                //window3d.SetFloorRendering(true, p.X, p.Y, p.Z, n.X, n.Y, n.Z);
    //            }
    //            else
    //            {
    //                //window3d.SetFloorRendering(false, 0, 0, 0);
    //            }
    //        }

    //        // Release the sensor capture and depth image once they are no longer needed.
    //        k4a_capture_release(sensorCapture);
    //        k4a_image_release(depthImage);

    //    }
    //    else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
    //    {
    //        std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
    //        break;
    //    }

    //    //window3d.Render();
    //}

    ////window3d.Delete();

    //k4a_device_stop_cameras(device);
    //k4a_device_stop_imu(device);
    //k4a_device_close(device);
}

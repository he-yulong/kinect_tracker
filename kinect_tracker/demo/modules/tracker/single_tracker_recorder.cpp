// legacy code of baoliqiang
// http://gitlab.unionbigdata.com/baoliqiang/TrackerRecorder.git
// branch: master
#include "szl/single_tracker_recorder.h"
using szl_kinect::SingleTrackerRecorder;

#include <iostream>
#include <direct.h>
#include <io.h>
#include <ctime>
#include <sstream>
#include <fstream>
#include <stdio.h>


//#include <string>
//#include <filesystem>
//#include <stdlib.h>
//#include <stdarg.h>
//#include <math.h>
//#include <gflags/gflags.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#define VERIFY(result, error)																				\
	if (result != K4A_RESULT_SUCCEEDED)																		\
	{																										\
		printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__);	\
		exit(1);																							\
	}																										\


using namespace std;

//// GFLAGS definitions
//DEFINE_bool(all_joints, true, "If output all joints");
//DEFINE_bool(save, true, "If save images and outputs");

bool all_joints = true;
bool save = true;


int MakeDirs(const char* pDir)
{
	int i = 0;
	int iRet;
	int iLen;
	char* pszDir;

	if (NULL == pDir)
	{
		return 0;
	}

	pszDir = _strdup(pDir);
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
					return -1;
				}
			}
			//支持linux,将所有\换成/
			pszDir[i] = '/';
		}
	}

	iRet = _mkdir(pszDir);
	free(pszDir);
	return iRet;
}


int SingleTrackerRecorder::Save()
{
	// 获取时间，作为本次保存的文件夹名
	struct tm ltm;
	time_t now;
	time(&now);
	localtime_s(&ltm, &now);
	int year = 1900 + ltm.tm_year;
	int month = 1 + ltm.tm_mon;
	int day = ltm.tm_mday;
	int hour = ltm.tm_hour;
	int min = ltm.tm_min;
	int sec = ltm.tm_sec;
	stringstream ss;
	ss << year << "-" << month << "-" << day << "-" << hour << "-" << min << "-" << sec;
	string save_dir = "save/" + ss.str();
	// 用于关键点保存
	ofstream outFile;
	if (save) {
		MakeDirs(save_dir.c_str());
		cout << "Images saved in " << save_dir << endl;
		outFile.open(save_dir + "/annot.txt", ios::out);
	}


	k4a_device_t device = NULL;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

	// Strart Camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.synchronized_images_only = true;

	// this is commented
	/*deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;*/

	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration), "Get depth camera calibration failed!");

	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

	int frame_count = 0;
	do
	{
		cout << "----------------------------------------------------------------------------------------------" << endl;
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
			frame_count++;

			if (save) {
				// 拿到彩色图和深度图
				k4a_image_t colorImage = k4a_capture_get_color_image(sensor_capture);
				k4a_image_t depthImage = k4a_capture_get_depth_image(sensor_capture);
				if (colorImage != NULL && depthImage != NULL)
				{
					// 彩色图
					uint8_t* colorBuffer = k4a_image_get_buffer(colorImage);
					int color_rows = k4a_image_get_height_pixels(colorImage);
					int color_cols = k4a_image_get_width_pixels(colorImage);
					cv::Mat colorMat(color_rows, color_cols, CV_8UC4, (void*)colorBuffer, cv::Mat::AUTO_STEP);

					// 保存彩色图
					stringstream ss;
					ss << save_dir << "/color-" << frame_count << ".png";
					bool save_color_result = cv::imwrite(ss.str(), colorMat);
					if (!save_color_result)
					{
						cout << "Error! Save color image failed!" << endl;
					}

					// 深度图
					uint8_t* depthBuffer = k4a_image_get_buffer(depthImage);
					int depth_rows = k4a_image_get_height_pixels(depthImage);
					int depth_cols = k4a_image_get_width_pixels(depthImage);
					cv::Mat depthMat(depth_rows, depth_cols, CV_32FC1, (void*)depthBuffer, cv::Mat::AUTO_STEP);

					// 保存深度图
					stringstream depth_ss;
					depth_ss << save_dir << "/depth-" << frame_count << ".png";
					bool save_depth_result = cv::imwrite(depth_ss.str(), depthMat);
					if (!save_depth_result)
					{
						cout << "Error! Save depth image failed!" << endl;
					}

					// Release images
					k4a_image_release(colorImage);
					k4a_image_release(depthImage);
				}
			}

			// 拿到关节点数据
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
			k4a_capture_release(sensor_capture);	// Remember to release the sensor capture once you finish using it
			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				cout << "Error! Add capture to tracker process queue timeout!\n";
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
			{
				cout << "Error! Add capture to tracker process queue failed!\n";
				break;
			}

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%zu bodies are detected!\n", num_bodies);

				// 如果检测到人体
				if (num_bodies > 0)
				{
					// 目标人体骨架
					k4abt_skeleton_t tar_skeleton;

					// 如果只有一个人
					if (num_bodies == 1) {
						k4abt_skeleton_t skeleton;
						k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);
						if (skeleton_result == K4A_RESULT_SUCCEEDED) {
							tar_skeleton = skeleton;
						}
						else if (skeleton_result == K4A_RESULT_FAILED) {
							printf("Error! Get body skeleton failed!\n");
						}
					}
					// 如果有多个人，则找到处于最中央的人
					else if (num_bodies > 1) {
						vector<k4a_float2_t> centroids;
						vector<k4abt_skeleton_t> skeletons;
						for (int i = 0; i < num_bodies; i++)
						{
							k4abt_skeleton_t skeleton;
							k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
							if (skeleton_result == K4A_RESULT_SUCCEEDED)
							{
								// Successfully get skeleton for i-th person. Start processing...
								// 先把skeleton保存起来
								skeletons.push_back(skeleton);

								// 用于计算质心，然后根据质心来使用处于中间的人
								k4a_float2_t centroid;
								centroid.xy.x = 0;
								centroid.xy.y = 0;
								for (int j = 0; j < K4ABT_JOINT_COUNT; j++)
								{
									k4a_float2_t p2d;
									int valid;
									k4a_result_t calibration_result = k4a_calibration_3d_to_2d(&sensor_calibration,
										&skeleton.joints[j].position,
										K4A_CALIBRATION_TYPE_DEPTH,
										K4A_CALIBRATION_TYPE_COLOR,
										&p2d,
										&valid);
									if (calibration_result == K4A_RESULT_SUCCEEDED && valid == 1)
									{
										centroid.xy.x += p2d.xy.x / K4ABT_JOINT_COUNT;
										centroid.xy.y += p2d.xy.y / K4ABT_JOINT_COUNT;
									}
									else
									{
										printf("Error! Calibrate 3d to 2d failed!\n");
									}
								}
								centroids.push_back(centroid);
							}
							else if (skeleton_result == K4A_RESULT_FAILED)
							{
								printf("Error! Get body skeleton failed!\n");
							}
						}

						// 看看哪个人处于中央，找到他的index
						const float CX = 1280 / 2;
						const float CY = 720 / 2;
						int center_id = 0;
						float min_cdist = 9999;
						for (int i = 0; i < centroids.size(); i++) {
							float cdist = sqrt(pow(centroids[i].xy.x - CX, 2) + pow(centroids[i].xy.y - CY, 2));
							if (cdist < min_cdist) {
								min_cdist = cdist;
								center_id = i;
							}
						}

						// 找到了处于中央的人的id，我们现在从skeleton列表中拿到相应的skeleton
						tar_skeleton = skeletons[center_id];
					}

					// 输出目标人体的骨架
					{
						// 得到骨架结果字符串
						stringstream ss;
						ss << ">>>";
						for (k4abt_joint_t joint : tar_skeleton.joints) {
							ss << joint.position.xyz.x << " "
								<< joint.position.xyz.y << " "
								<< joint.position.xyz.z << ", ";
						}
						ss << "|";
						for (k4abt_joint_t joint : tar_skeleton.joints) {
							ss << joint.orientation.wxyz.w << " "
								<< joint.orientation.wxyz.x << " "
								<< joint.orientation.wxyz.y << " "
								<< joint.orientation.wxyz.z << ", ";
						}
						ss << "|";
						for (k4abt_joint_t joint : tar_skeleton.joints) {
							ss << joint.confidence_level << " ";
						}
						ss << "<<<";
						string output = ss.str();

						// 输出到stdout
						cout << output << endl;

						// 将结果保存到文件
						if (save) {
							if (!outFile.is_open()) {
								outFile.open(save_dir + "/annot.txt", ios::app);
							}
							outFile << output << endl;
						}
					}
				}

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
				//break;
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

	// 关闭文件句柄
	outFile.close();

	return 0;
}


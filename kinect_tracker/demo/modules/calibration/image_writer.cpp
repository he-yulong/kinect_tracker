#include "szl/image_writer.h"
using szl_kinect::ImageWriter;

#include "szl/single_tracker_recorder.h"

#include <iostream>
#include <direct.h>
#include <io.h>
#include <ctime>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <k4a/k4a.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 这是第三方声明的头文件
#include "forestsen/StaticImageProperties.h"
#include "forestsen/Pixel.h"
#include "forestsen/DepthPixelColorizer.h"

#include <windows.h>

using namespace std;

#define VERIFY(result, error)																				\
	if (result != K4A_RESULT_SUCCEEDED)																		\
	{																										\
		printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__);	\
		exit(1);																							\
	}																										\

static int MakeDir(const char* pDir)
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

bool all_joints_iw = true;
bool save_iw = true;

int ImageWriter::Write()
{
	string save_dir = "calib_imgs/3";

	if (save_iw) {
		MakeDir(save_dir.c_str());
		cout << "Images saved in " << save_dir << endl;
	}

	// my
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_5;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a::device dev_sub = k4a::device::open(1);
	dev_sub.start_cameras(&config);
	k4a::device dev_master = k4a::device::open(K4A_DEVICE_DEFAULT);
	dev_master.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	std::vector<sen::Pixel> depthTextureBuffer;
	std::vector<sen::Pixel> irTextureBuffer;
	uint8_t* colorTextureBuffer;

	k4a::capture capture_master;
	k4a::capture capture_sub;

	k4a::image depthImage;
	k4a::image colorImage = NULL;
	k4a::image irImage;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat irFrame;

	int frame_count = 0;
	do
	{
		if (dev_master.get_capture(&capture_master, std::chrono::milliseconds(0)) && dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0))) {
			Sleep(4000);
			cout << frame_count << ":----------------------------------------------------------------------------------------------" << endl;
			frame_count++;
			{
				colorImage = capture_sub.get_color_image();
				colorTextureBuffer = colorImage.get_buffer();
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				// 保存彩色图
				stringstream ss;
				ss << save_dir << "/right" << frame_count << ".jpg";
				bool save_color_result = cv::imwrite(ss.str(), colorFrame);
				if (!save_color_result)
				{
					cout << "Error! Save color image failed!" << endl;
				}
			}

			{
				colorImage = capture_master.get_color_image();
				colorTextureBuffer = colorImage.get_buffer();
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				// 保存彩色图
				stringstream ss;
				ss << save_dir << "/left" << frame_count << ".jpg";
				bool save_color_result = cv::imwrite(ss.str(), colorFrame);
				if (!save_color_result)
				{
					cout << "Error! Save color image failed!" << endl;
				}
			}
		}
	} while (frame_count < 20);

	dev_sub.close();
	dev_master.close();
	return 0;
}

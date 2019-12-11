#include "szl/double_view.h"
using szl_kinect::DoubleView;

#include <k4a/k4a.hpp>

#include <iostream>
#include <vector>
#include <array>
using namespace std;


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// 这是第三方声明的头文件
#include "forestsen/StaticImageProperties.h"
#include "forestsen/Pixel.h"
#include "forestsen/DepthPixelColorizer.h"


//// 不过我更期望使用下面官方的头文件，我猜测这与上面基本上是相同的
//#include "k4a_opengl/k4apixel.h"
//#include "k4a_opengl/k4adepthpixelcolorizer.h"
//#include "k4a_opengl/k4astaticimageproperties.h"

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file


int DoubleView::Show()
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
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
	k4a::image colorImage;
	k4a::image irImage;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat irFrame;

	while (1)
	{
		if (dev_master.get_capture(&capture_master, std::chrono::milliseconds(0)) && dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0)))
		{
			{
				depthImage = capture_master.get_depth_image();
				colorImage = capture_master.get_color_image();
				irImage = capture_master.get_ir_image();

				sen::ColorizeDepthImage(depthImage, sen::DepthPixelColorizer::ColorizeBlueToRed, sen::GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
				sen::ColorizeDepthImage(irImage, sen::DepthPixelColorizer::ColorizeGreyscale, sen::GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
				colorTextureBuffer = colorImage.get_buffer();

				depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
				cv::imshow("kinect depth map master", depthFrame);
				cv::imshow("kinect color frame master", colorFrame);
				cv::imshow("kinect ir frame master", irFrame);
			}

			{
				depthImage = capture_sub.get_depth_image();
				colorImage = capture_sub.get_color_image();
				irImage = capture_sub.get_ir_image();

				sen::ColorizeDepthImage(depthImage, sen::DepthPixelColorizer::ColorizeBlueToRed, sen::GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
				sen::ColorizeDepthImage(irImage, sen::DepthPixelColorizer::ColorizeGreyscale, sen::GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
				colorTextureBuffer = colorImage.get_buffer();

				depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
				cv::imshow("kinect depth map sub", depthFrame);
				cv::imshow("kinect color frame sub", colorFrame);
				cv::imshow("kinect ir frame sub", irFrame);
			}
		}
		if (cv::waitKey(30) == 27 || cv::waitKey(30) == 'q')
		{
			dev_master.close();
			break;
		}
	}
	return 0;
}


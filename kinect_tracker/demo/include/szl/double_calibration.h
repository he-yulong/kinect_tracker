#pragma once
// Calibration of the two kinects.
#include <string>
#include <iostream>
using std::string;
using std::cout;
using std::endl;

#include <vector>
#include <direct.h>
#include <io.h>
#include <stdio.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

// ------------------------------------
#include <k4a/k4a.hpp>
// must set after <k4a/k4a.hpp>
#include <windows.h>
// ------------------------------------

using std::string;
using std::vector;
using cv::Point3f;
using cv::Point2f;
using cv::Mat;
using cv::Size;

namespace szl_kinect {
	static int makeDir(const char* pDir);
	static bool doesExist(const std::string& name);

	class DoubleCalibration
	{
	public:
		DoubleCalibration(string save_dir = "calib_imgs/temp", string extension = "jpg");
		int CollectColorImages(int image_num = 25, int sleep_time = 5000);
		int CalibrateIntrinsic(int board_width = 8, int board_height = 6, int num_imgs = 25, float square_size = 0.03);
		int CalibrateStereo(int image_num = 25);
	private:
		string save_dir;
		string extension;
		vector< vector< Point3f > > object_points;
		vector< vector< Point2f > > image_points;
		vector< Point2f > corners;
		vector< vector< Point2f > > left_img_points;
		Mat img, gray;
		Size im_size;

		vector< vector< Point3f > > stereo_object_points;
		vector< vector< Point2f > > imagePoints1, imagePoints2;
		vector< Point2f > corners1, corners2;
		vector< vector< Point2f > > stereo_left_img_points, right_img_points;

		Mat img1, img2, gray1, gray2;

		void setup_calibration(int board_width, int board_height, int num_imgs,  float square_size, string imgs_directory, string imgs_filename);
		double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints, const vector< vector< Point2f > >& imagePoints, const vector< Mat >& rvecs, const vector< Mat >& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs);
		void load_image_points(int board_width, int board_height, int num_imgs, float square_size);
	};
}

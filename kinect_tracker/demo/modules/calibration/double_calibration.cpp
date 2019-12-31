#include "szl/double_calibration.h"

szl_kinect::DoubleCalibration::DoubleCalibration(string save_dir, string extension): save_dir(save_dir), extension(extension)
{
	cout << "Ready for calibration..." << endl;
}


static int szl_kinect::makeDir(const char* pDir)
{
	int i = 0;
	int iLen;
	char* pszDir;

	pszDir = _strdup(pDir);
	iLen = strlen(pszDir);

	for (i = 0; i < iLen; i++)
	{
		if (pszDir[i] == '\\' || pszDir[i] == '/')
		{
			pszDir[i] = '\0';

			// if directory not exist, then create one.
			if (_access(pszDir, 0) != 0)
			{
				_mkdir(pszDir);
			}
			// linux supported, replace "\" to "/"
			pszDir[i] = '/';
		}
	}

	_mkdir(pszDir);
	free(pszDir);
	return 0;
}

int szl_kinect::DoubleCalibration::CollectColorImages(int image_num, int sleep_time)
{
	// making directory.
	makeDir(save_dir.c_str());
	cout << "Collecting images..." << endl;
	cout << "Making Directory..." << endl << "Images will be saved in " << save_dir << endl;

	// checking devices.
	if (k4a::device::get_installed_count() == 0)
	{
		cout << "No azure kinect devices detected!" << endl;
		return 1;
	}
	
	// setting two kinects' configuration.
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_5;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.synchronized_images_only = true;

	// opening kinects.
	cout << "Started opening K4A device 1(sub)..." << endl;
	k4a::device dev_sub = k4a::device::open(1);
	dev_sub.start_cameras(&config);
	cout << "Started opening K4A device 0(master)..." << endl;
	k4a::device dev_master = k4a::device::open(K4A_DEVICE_DEFAULT);
	dev_master.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	uint8_t* colorTextureBuffer;
	k4a::capture capture_master;
	k4a::capture capture_sub;
	k4a::image colorImage = NULL;
	cv::Mat colorFrame;

	int frame_count = 0;
	do
	{
		if (dev_master.get_capture(&capture_master, std::chrono::milliseconds(0)) && dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0))) {
			Sleep(sleep_time);
			frame_count++;
			cout << "saving two kinects' images: " << frame_count << endl;
			{
				colorImage = capture_sub.get_color_image();
				colorTextureBuffer = colorImage.get_buffer();
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				// 保存彩色图
				std::stringstream ss;
				ss << save_dir << "/sub" << frame_count << "." << extension;
				bool save_color_result = cv::imwrite(ss.str(), colorFrame);
				if (!save_color_result)
				{
					cout << "Error! Saving color image failed!" << endl;
				}
			}

			{
				colorImage = capture_master.get_color_image();
				colorTextureBuffer = colorImage.get_buffer();
				colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
				// 保存彩色图
				std::stringstream ss;
				ss << save_dir << "/master" << frame_count << "." << extension;
				bool save_color_result = cv::imwrite(ss.str(), colorFrame);
				if (!save_color_result)
				{
					cout << "Error! Save color image failed!" << endl;
				}
			}
		}
	} while (frame_count < image_num);

	dev_sub.close();
	dev_master.close();
	return 0;
}


static bool szl_kinect::doesExist(const std::string& name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

void szl_kinect::DoubleCalibration::setup_calibration(int board_width, int board_height, int num_imgs, float square_size, string imgs_directory, string imgs_filename)
{
	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	for (int k = 1; k <= num_imgs; k++) {
		char img_file[100];
		std::sprintf(img_file, "%s/%s%d.%s", imgs_directory, imgs_filename, k, extension);
		if (!doesExist(img_file))
			continue;
		img = imread(img_file, cv::IMREAD_COLOR);
		cv::cvtColor(img, gray, CV_BGR2GRAY);

		bool found = false;
		found = cv::findChessboardCorners(img, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_size, corners, found);
		}

		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found) {
			cout << k << ". Found corners!" << endl;
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
		else {
			cout << k << "Not found!" << endl;
		}
	}
}

double szl_kinect::DoubleCalibration::computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints, const vector< vector< Point2f > >& imagePoints, 
	const vector< Mat >& rvecs, const vector< Mat >& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs)
{
	vector< Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector< float > perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}



int szl_kinect::DoubleCalibration::CalibrateIntrinsic(int board_width, int board_height, int num_imgs, float square_size)
{

	string imgs_filename[2] = { "sub", "master" };
	string out_file[2] = { "cam_sub.yml" , "cam_master.yml"};

	for (int i = 0; i < 2; i++) {
		setup_calibration(board_width, board_height, num_imgs, square_size, save_dir, imgs_filename[i]);
		cout << "[" << imgs_filename[i] << "]" << "Starting Calibration." << endl;

		Mat K;
		Mat D;
		vector< Mat > rvecs, tvecs;
		int flag = 0;
		flag |= cv::CALIB_FIX_K4;
		flag |= cv::CALIB_FIX_K5;
		calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

		cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

		cv::FileStorage fs(save_dir + "/" + out_file[i], cv::FileStorage::WRITE);
		fs << "K" << K;
		fs << "D" << D;
		fs << "board_width" << board_width;
		fs << "board_height" << board_height;
		fs << "square_size" << square_size;
		
		cout << "[" << imgs_filename[i] << "]" << "Done Calibration." << endl;
	}
	
	return 0;
}


void szl_kinect::DoubleCalibration::load_image_points(int board_width, int board_height, int num_imgs, float square_size) {

	using cv::imread;

	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	for (int i = 1; i <= num_imgs; i++) {
		char sub_img[100], master_img[100];
		std::sprintf(sub_img, "%s%s%d.%s", save_dir, "/sub", i, extension);
		std::sprintf(master_img, "%s%s%d.%s", save_dir, "/master", i, extension);
		img1 = imread(sub_img, cv::IMREAD_COLOR);
		img2 = imread(master_img, cv::IMREAD_COLOR);
		cvtColor(img1, gray1, CV_BGR2GRAY);
		cvtColor(img2, gray2, CV_BGR2GRAY);

		bool found1 = false, found2 = false;

		found1 = cv::findChessboardCorners(img1, board_size, corners1,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, board_size, corners2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);


		if (!found1 || !found2) {
			cout << "[Chessboard find error!]" << sub_img << " and " << master_img << endl;
			continue;
		}

		if (found1)
		{
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray1, board_size, corners1, found1);
		}
		if (found2)
		{
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray2, board_size, corners2, found2);
		}

		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found1 && found2) {
			cout << "[OK]" << i << ". Found corners!" << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			stereo_object_points.push_back(obj);
		}
	}
	for (int i = 0; i < imagePoints1.size(); i++) {
		vector< Point2f > v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++) {
			v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
			v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		stereo_left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}
}

int szl_kinect::DoubleCalibration::CalibrateStereo(int image_num)
{
	using cv::FileStorage;
	using cv::Vec3d;

	string out_file = save_dir + "/cam_stereo.yml";
	FileStorage fsl(save_dir + "/cam_sub.yml", FileStorage::READ);
	FileStorage fsr(save_dir + "/cam_master.yml", FileStorage::READ);

	load_image_points(fsl["board_width"], fsl["board_height"], image_num, fsl["square_size"]);

	printf("Starting Calibration\n");
	Mat K1, K2, R, F, E;
	Vec3d T;
	Mat D1, D2;
	fsl["K"] >> K1;
	fsr["K"] >> K2;
	fsl["D"] >> D1;
	fsr["D"] >> D2;
	int flag = 0;
	flag |= cv::CALIB_FIX_INTRINSIC;

	cout << "Read intrinsics" << endl;

	stereoCalibrate(stereo_object_points, stereo_left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);

	cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
	fs1 << "K1" << K1;
	fs1 << "K2" << K2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;

	printf("Done Calibration\n");

	printf("Starting Rectification\n");

	cv::Mat R1, R2, P1, P2, Q;
	stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);

	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;

	printf("Done Rectification\n");
}



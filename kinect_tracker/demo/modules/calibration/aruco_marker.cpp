#include "szl/aruco_marker.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>

#include <cstdlib>
#include <iomanip>


using namespace std;
using namespace cv;


namespace {
	const char* about =
		"Calibration using a ArUco Planar Grid board\n"
		"  To capture a frame for calibration, press 'c',\n"
		"  If input comes from video, press any key for next frame\n"
		"  To finish capturing, press 'ESC' key and calibration starts.\n";
	const char* keys =
		"{w        |       | Number of squares in X direction }"
		"{h        |       | Number of squares in Y direction }"
		"{l        |       | Marker side length (in meters) }"
		"{s        |       | Separation between two consecutive markers in the grid (in meters) }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{@outfile |<none> | Output file with calibrated camera parameters }"
		"{v        |       | Input from video file, if ommited, input comes from camera }"
		"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
		"{dp       |       | File of marker detector parameters }"
		"{rs       | false | Apply refind strategy }"
		"{zt       | false | Assume zero tangential distortion }"
		"{a        |       | Fix aspect ratio (fx/fy) to this value }"
		"{pc       | false | Fix the principal point at the center }"
		"{waitkey  | 10    | Time in milliseconds to wait for key press }";
}

/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters>& params) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
}

int szl_kinect::ArucoMarker::Create()
{

	bool showImage = true;

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat markerImg;
	int markerId = 23;
	int markerSize = 200;
	int borderBits = 1;

	cv::aruco::drawMarker(dictionary, 23, 200, markerImg, 1);

	if (showImage) {
		cv::imshow("This is a marker", markerImg);
		cv::waitKey(0);
	}

	return 0;
}


/**
 */
static bool saveCameraParams(const string& filename, Size imageSize, float aspectRatio, int flags,
	const Mat& cameraMatrix, const Mat& distCoeffs, double totalAvgErr) {
	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm* t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	if (flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;

	return true;
}



/**
 */
int szl_kinect::ArucoMarker::Calibrate() {
	int markersX = 4;
	int markersY = 2;
	float markerLength = 0.04;
	float markerSeparation = 0.022;
	int dictionaryId = 16;
	string outputFile = "calibration_params.yml";

	int calibrationFlags = 0;
	float aspectRatio = 1;

	calibrationFlags = CALIB_ZERO_TANGENT_DIST;
	calibrationFlags = CALIB_FIX_PRINCIPAL_POINT;

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

	bool readOk = readDetectorParameters("detector_params.yml", detectorParams);
	if (!readOk) {
		cerr << "Invalid detector parameters file" << endl;
		return 0;
	}

	bool refindStrategy = false;
	int camId = 0;
	String video;

	int waitTime = 1;

	String videoInput;
	VideoCapture inputVideo;

	bool opened;
	if (!video.empty()) {
		videoInput = video;
		opened = inputVideo.open(video);
	}
	else {
		videoInput = camId;
		opened = inputVideo.open(camId);
	}

	if (!opened) {
		std::cerr << "failed to open video input: " << videoInput << std::endl;
		return 1;
	}

	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	// create board object
	Ptr<aruco::GridBoard> gridboard =
		aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
	Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

	// collected frames for calibration
	vector< vector< vector< Point2f > > > allCorners;
	vector< vector< int > > allIds;
	Size imgSize;

	while (inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);

		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;

		// detect markers
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

		// refind strategy to detect more markers
		if (refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

		// draw results
		image.copyTo(imageCopy);
		if (ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners, ids);
		putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
			Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

		imshow("out", imageCopy);
		char key = (char)waitKey(waitTime);
		if (key == 27) break;
		if (key == 'c' && ids.size() > 0) {
			cout << "Frame captured" << endl;
			allCorners.push_back(corners);
			allIds.push_back(ids);
			imgSize = image.size();
		}
	}

	if (allIds.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return 0;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(allCorners.size());
	for (unsigned int i = 0; i < allCorners.size(); i++) {
		markerCounterPerFrame.push_back((int)allCorners[i].size());
		for (unsigned int j = 0; j < allCorners[i].size(); j++) {
			allCornersConcatenated.push_back(allCorners[i][j]);
			allIdsConcatenated.push_back(allIds[i][j]);
		}
	}
	// calibrate camera
	repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, imgSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags, cameraMatrix,
		distCoeffs, repError);

	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return 0;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Calibration saved to " << outputFile << endl;

	return 0;
}

//const char* about = "Pose estimation of ArUco marker images";
//const char* keys =
//"{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
//"DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
//"DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
//"DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
//"DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
//"{h        |false | Print help }"
//"{l        |      | Actual marker length in meter }"
//"{v        |<none>| Custom video source, otherwise '0' }"
//"{h        |false | Print help }"
//"{l        |      | Actual marker length in meter }"
//"{v        |<none>| Custom video source, otherwise '0' }"

#define CV_AA 16

int szl_kinect::ArucoMarker::PoseEstimate() {

	
	int dictionaryId = 16;
	float marker_length_m = 0.136;
	int wait_time = 10;

	if (marker_length_m <= 0) {
		std::cerr << "marker length must be a positive value in meter"
			<< std::endl;
		return 1;
	}

	cv::String videoInput = "0";
	cv::VideoCapture in_video;

	char* end = nullptr;
	int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
		10));
	if (!end || end == videoInput.c_str()) {
		in_video.open(videoInput); // url
	}
	else {
		in_video.open(source); // id
	}

	if (!in_video.isOpened()) {
		std::cerr << "failed to open video input: " << videoInput << std::endl;
		return 1;
	}

	cv::Mat image, image_copy;
	cv::Mat camera_matrix, dist_coeffs;
	std::ostringstream vector_to_marker;

	cv::Ptr<cv::aruco::Dictionary> dictionary =
		cv::aruco::getPredefinedDictionary(\
			cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);

	fs["camera_matrix"] >> camera_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;

	std::cout << "camera_matrix\n" << camera_matrix << std::endl;
	std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

	while (in_video.grab())
	{
		in_video.retrieve(image);
		image.copyTo(image_copy);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);

		// if at least one marker detected
		if (ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
				camera_matrix, dist_coeffs, rvecs, tvecs);

			
			// Draw axis for each marker
			for (int i = 0; i < ids.size(); i++)
			{
				cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
					rvecs[i], tvecs[i], 0.1);

				// This section is going to print the data for all the detected
				// markers. If you have more than a single marker, it is
				// recommended to change the below section so that either you
				// only print the data for a specific marker, or you print the
				// data for each marker separately.
				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "x: " << std::setw(8) << tvecs[0](0);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);

				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "y: " << std::setw(8) << tvecs[0](1);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);

				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "z: " << std::setw(8) << tvecs[0](2);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);
			}
		}

		imshow("Pose estimation", image_copy);
		char key = (char)cv::waitKey(wait_time);
		if (key == 27)
			break;
	}

	in_video.release();
	return 0;
}




//namespace {
//	const char* about = "Draw cube on ArUco marker images";
//	const char* keys =
//		"{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
//		"DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
//		"DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
//		"DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
//		"DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
//		"{h        |false | Print help }"
//		"{l        |      | Actual marker length in meter }"
//		"{v        |<none>| Custom video source, otherwise '0' }"
//		;
//}

void drawCubeWireframe(
	cv::InputOutputArray image, cv::InputArray cameraMatrix,
	cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
	float l
);


int szl_kinect::ArucoMarker::DrawCube()
{
	int dictionaryId = 16;
	float marker_length_m = 0.136;
	int wait_time = 100;

	if (marker_length_m <= 0) {
		std::cerr << "marker length must be a positive value in meter"
			<< std::endl;
		return 1;
	}

	cv::String videoInput = "0";
	cv::VideoCapture in_video;

	in_video.open(0);


	if (!in_video.isOpened()) {
		std::cerr << "failed to open video input: " << videoInput << std::endl;
		return 1;
	}

	cv::Mat image, image_copy;
	cv::Mat camera_matrix, dist_coeffs;
	std::ostringstream vector_to_marker;

	cv::Ptr<cv::aruco::Dictionary> dictionary =
		cv::aruco::getPredefinedDictionary(\
			cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);

	fs["camera_matrix"] >> camera_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;

	std::cout << "camera_matrix\n"
		<< camera_matrix << std::endl;
	std::cout << "\ndist coeffs\n"
		<< dist_coeffs << std::endl;

	int frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);
	int fps = 30;

	while (in_video.grab())
	{

		in_video.retrieve(image);
		image.copyTo(image_copy);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);

		// if at least one marker detected
		if (ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(
				corners, marker_length_m, camera_matrix, dist_coeffs,
				rvecs, tvecs
			);

			// draw axis for each marker
			for (int i = 0; i < ids.size(); i++)
			{
				drawCubeWireframe(
					image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
					marker_length_m
				);

				// This section is going to print the data for all the detected
				// markers. If you have more than a single marker, it is 
				// recommended to change the below section so that either you
				// only print the data for a specific marker, or you print the
				// data for each marker separately.
				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "x: " << std::setw(8) << tvecs[0](0);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);

				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "y: " << std::setw(8) << tvecs[0](1);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);

				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
					<< "z: " << std::setw(8) << tvecs[0](2);
				cv::putText(image_copy, vector_to_marker.str(),
					Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
					Scalar(0, 252, 124), 1, CV_AA);
			}
		}

		cv::imshow("Pose estimation", image_copy);
		char key = (char)cv::waitKey(wait_time);
		if (key == 27)
			break;
	}

	in_video.release();

	return 0;
}

void drawCubeWireframe(
	cv::InputOutputArray image, cv::InputArray cameraMatrix,
	cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
	float l
)
{

	CV_Assert(
		image.getMat().total() != 0 &&
		(image.getMat().channels() == 1 || image.getMat().channels() == 3)
	);
	CV_Assert(l > 0);
	float half_l = l / 2.0;

	// project cube points
	std::vector<cv::Point3f> axisPoints;
	axisPoints.push_back(cv::Point3f(half_l, half_l, l));
	axisPoints.push_back(cv::Point3f(half_l, -half_l, l));
	axisPoints.push_back(cv::Point3f(-half_l, -half_l, l));
	axisPoints.push_back(cv::Point3f(-half_l, half_l, l));
	axisPoints.push_back(cv::Point3f(half_l, half_l, 0));
	axisPoints.push_back(cv::Point3f(half_l, -half_l, 0));
	axisPoints.push_back(cv::Point3f(-half_l, -half_l, 0));
	axisPoints.push_back(cv::Point3f(-half_l, half_l, 0));

	std::vector<cv::Point2f> imagePoints;
	projectPoints(
		axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints
	);

	// draw cube edges lines
	cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[1], imagePoints[5], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[2], imagePoints[6], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[3], imagePoints[7], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[4], imagePoints[7], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[5], imagePoints[6], cv::Scalar(255, 0, 0), 3);
	cv::line(image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 3);
}
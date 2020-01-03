// demo.cpp : This file contains the 'main' function. Program execution begins and ends there.
#include <iostream>
using std::cin;
using std::cout;
using std::endl;
using std::string;

#include "szl/single_startup.h"
using szl_kinect::SingleStartup;
#include "szl/single_view.h"
using szl_kinect::SingleView;
#include "szl/double_view.h"
using szl_kinect::DoubleView;
#include "szl/single_tracker.h"
using szl_kinect::SingleTracker;
#include "szl/single_tracker_recorder.h"
using szl_kinect::SingleTrackerRecorder;
#include "szl/single_udp_tracker.h"
using szl_kinect::SingleUDPTracker;
#include "szl/double_tracker.h"
using szl_kinect::DoubleTracker;
#include "szl/offline_processor.h"
using szl_kinect::OfflineProcessor;
#include "szl/double_udp_tracker.h"
using szl_kinect::DoubleUDPTracker;
#include "szl/aruco_marker.h"
using szl_kinect::ArucoMarker;
#include "szl/double_calibration.h"
using szl_kinect::DoubleCalibration;
#include "szl/several_quaternion_udp_tracker.h"
using szl_kinect::SeveralQuaternionUDPTracker;


int main()
{
	string oper;
	while (true)
	{
		cout << "-----------------------------------------" << endl;
		cout << "Kinect Console Application" << endl << endl;
		cout << "Please enter the operation to perform: " << endl;
		cout << "0: to test single kinect's startup." << endl;
		cout << "1: to open single kinect's viewer." << endl;
		cout << "2: to open two kinect's viewer." << endl;
		cout << "3: to test single kinect's tracker(simple information)." << endl;
		cout << "4: to test single kinect's tracker(detailed information)." << endl;
		cout << "5: to save images and tracking data in the save directory(baoliqiang)." << endl;
		cout << "6: to send single tracking data using UDP(baoliqiang)." << endl;
		cout << "7: to test double kinects's tracker(simple information)." << endl;
		cout << "8: to test double kinects's tracker(detailed information)." << endl;
		cout << "9: to execute offline processing: mkv file to json file of tracking result." << endl;
		cout << "10: to send two kinects' tracking data using UDP." << endl;
		cout << "11: to test aruco marker demo." << endl;
		cout << "12: to get two kinects' extrinsics data." << endl;
		cout << "13: full-fleged stereo calibration(writing images, calibration of intrinsic and extrinsic parameters, stereo calibration)." << endl;
		cout << "14: to test fusion several kinects' quaternion." << endl;
		cout << "q: to quit." << endl;
		cout << "-----------------------------------------" << endl;
		cin >> oper;
		if (oper == "q") {
			cout << "ready to quit..." << endl;
			break;
		}
		else if (oper == "0") {
			SingleStartup kinect;
			kinect.Run();
		}
		else if (oper == "1") {
			SingleView kinect;
			kinect.Show();
		}
		else if (oper == "2") {
			DoubleView kinects;
			kinects.Show();
		}
		else if (oper == "3") {
			cout << "Please set maximum frame(e.g. -1, 100, 200): ";
			int max_frame;
			cin >> max_frame;
			cout << endl;
			SingleTracker kinect;
			kinect.RunWithSimpleInformation(max_frame);
		}
		else if (oper == "4") {
			cout << "Please set maximum frame(e.g. -1, 100, 200): ";
			int max_frame;
			cin >> max_frame;
			cout << endl;
			SingleTracker kinect;
			kinect.RunWithDetailedInformation(max_frame);
		}
		else if (oper == "5") {
			SingleTrackerRecorder kinect;
			kinect.Save();
		}
		else if (oper == "6") {
			string arg;

			string ip_addr;
			cout << "Please set the udp server's IP address(e.g. 127.0.0.1, 172.27.15.101) or type in 'd' to use default '127.0.0.1': ";
			cin >> arg;
			if (arg == "d") {
				ip_addr = "127.0.0.1";
				cout << "You set the udp server's IP address: " << ip_addr << endl;
			}

			int port = 8999;
			cout << "Please set the udp server's port(e.g. 8999, 6666) or type in 'd' to use default '8999': ";
			cin >> arg;
			if (arg != "d") {
				port = atoi(arg.c_str());
				cout << "You set the udp server's port: " << port << endl;
			}
			cout << endl;

			SingleUDPTracker kinect;
			kinect.Run(ip_addr, port, true);
		}
		else if (oper == "7") {
			cout << "Please set maximum frame(e.g. -1, 100, 200): ";
			int max_frame;
			cin >> max_frame;
			cout << endl;
			DoubleTracker kinects;
			kinects.RunWithSimpleInformation(max_frame);
		}
		else if (oper == "8") {
			cout << "Please set maximum frame(e.g. -1, 100, 200): ";
			int max_frame;
			cin >> max_frame;
			cout << endl;
			DoubleTracker kinects;
			kinects.RunWithDetailedInformation(max_frame);
		}
		else if (oper == "9") {
			OfflineProcessor processor;
			processor.Run();
		}
		else if (oper == "10") {
			string arg;

			// sub
			string ip_addr_sub = "127.0.0.1";
			cout << "Kinect Sub: Please set the udp server's IP address(e.g. 127.0.0.1, 172.27.15.101) or type in 'd' to use default '127.0.0.1': ";
			cin >> arg;
			if (arg != "d") {
				ip_addr_sub = arg;
			}
			cout << "Kinect Sub: You set the udp server's IP address: " << ip_addr_sub << endl;

			int port_sub = 8999;
			cout << "Kinect Sub: Please set the udp server's port(e.g. 8999, 6666) or type in 'd' to use default '8999': ";
			cin >> arg;
			if (arg != "d") {
				port_sub = atoi(arg.c_str());
			}
			cout << "Kinect Sub: You set the udp server's port: " << port_sub << endl;

			// master
			string ip_addr_master = "127.0.0.1";
			cout << "Kinect Master: Please set the udp server's IP address(e.g. 127.0.0.1, 172.27.15.101) or type in 'd' to use default '127.0.0.1': ";
			cin >> arg;
			if (arg != "d") {
				ip_addr_master = arg;
			}
			cout << "Kinect Master: You set the udp server's IP address: " << ip_addr_master << endl;

			int port_master = 8998;
			cout << "Kinect Master: Please set the udp server's port(e.g. 8999, 6666) or type in 'd' to use default '8998': ";
			cin >> arg;
			if (arg != "d") {
				port_master = atoi(arg.c_str());
			}
			cout << "Kinect Master: You set the udp server's port: " << port_master << endl;
			cout << endl;

			cout << "Please set maximum frame(e.g. -1, 100, 200): ";
			int max_frame;
			cin >> max_frame;
			cout << endl;

			DoubleUDPTracker kinects;
			kinects.Run(ip_addr_sub, ip_addr_master, port_sub, port_master, true, max_frame);
		}
		else if (oper == "11") {
			ArucoMarker marker;
			//marker.Create();
			//marker.Calibrate();
			//marker.PoseEstimate();
			marker.DrawCube();
		}
		else if (oper == "12") {
			/*DoubleExtrinsics ex;
			ex.Get();*/
		}
		else if (oper == "13") {
			DoubleCalibration cab = DoubleCalibration("calib_imgs/6");
			if (true) {
				cab.CollectColorImages();
			}
			if (true) {
				cab.CalibrateIntrinsic();
			}
			if (true) {
				cab.CalibrateStereo();
			}
		}
		else if (oper == "14") {
			SeveralQuaternionUDPTracker several_kinects = SeveralQuaternionUDPTracker(2, "127.0.0.1", 8999);
			several_kinects.Run();
		}
		else {
			cout << "******************************************" << endl;
			cout << "Unsupported input." << endl;
			cout << "******************************************" << endl << endl << endl;
		}
	}
}

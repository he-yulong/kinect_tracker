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
#include "szl/single_body_tracker.h"
using szl_kinect::SingleBodyTracker;
#include "szl/double_tracker.h"
using szl_kinect::DoubleTracker;
#include "szl/offline_processor.h"
using szl_kinect::OfflineProcessor;

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
		cout << "6: to send tracking data using udp(baoliqiang)." << endl;
		cout << "7: to test double kinects's tracker(simple information)." << endl;
		cout << "8: to test double kinects's tracker(detailed information)." << endl;
		cout << "9: to execute offline processing: mkv file to json file of tracking result." << endl;
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
			
			int port;
			cout << "Please set the udp server's port(e.g. 8999, 6666) or type in 'd' to use default '8999': ";
			cin >> arg;
			if (arg == "d") {
				port = 8999;
				cout << "You set the udp server's port: " << port << endl;
			}
			cout << endl;

			SingleBodyTracker kinect;
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
		else {
			cout << "******************************************" << endl;
			cout << "Unsupported input." << endl;
			cout << "******************************************" << endl << endl << endl;
		}
	}
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

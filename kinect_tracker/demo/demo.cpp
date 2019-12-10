// demo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "szl/single_startup.h"
#include "szl/single_view.h"
#include "szl/double_view.h"
#include "szl/single_tracker.h"

int main()
{
	using namespace std;

	char oper;
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
		cout << "q: to quit." << endl;
		cout << "-----------------------------------------" << endl;
		cin >> oper;
		if (oper == 'q') {
			cout << "ready to quit..." << endl;
			break;
		}
		else if (oper == '0') {
			SingleStartup kinect;
			kinect.Run();
		}
		else if (oper == '1') {
			SingleView kinect;
			kinect.Show();
		}
		else if (oper == '2') {
			DoubleView kinect;
			kinect.Show();
		}
		else if (oper == '3') {
			SingleTracker kinect;
			kinect.RunWithSimpleInformation();
		}
		else if (oper == '4') {
			SingleTracker kinect;
			kinect.RunWithDetailedInformation();
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

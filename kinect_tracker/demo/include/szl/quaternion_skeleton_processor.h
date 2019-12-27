#pragma once
#include <string>
#include <vector>
#include <map>
#include <k4abt.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace szl_kinect {
	class QuaternionSkeletonProcessor
	{
	public:
		QuaternionSkeletonProcessor();
		QuaternionSkeletonProcessor(k4abt_skeleton_t skeleton);
		//QuaternionSkeletonProcessor(k4abt_skeleton_t skeleton1, k4abt_skeleton_t skeleton2);
		~QuaternionSkeletonProcessor();

		static const map<string, int> kinectJointMap;
		static const vector<string> unityJoints;

		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;

		static vector<string> Split(const string& input, const string& delim);

		QuaternionSkeletonProcessor& FromString(string skeleton_string);
		QuaternionSkeletonProcessor& ToUnity();
		QuaternionSkeletonProcessor& FixView();
		string ToString();
		string ToString(vector<k4abt_joint_t> mSkeleton0);
		vector<k4abt_joint_t> mSkeleton;

	private:
		Vector3f mShift;
		bool mHasShift;
		Matrix3f mRotationMatrix;
	};
}

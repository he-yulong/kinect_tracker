#pragma once
#include <string>
#include <vector>
#include <map>
#include <k4abt.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace szl_kinect {
	class SkeletonProcessor
	{
	public:
		SkeletonProcessor();
		SkeletonProcessor(k4abt_skeleton_t skeleton);
		~SkeletonProcessor();

		static const map<string, int> kinectJointMap;
		static const vector<string> unityJoints;

		static const int KINECT_JOINT_NUM = 32;
		static const int UNITY_JOINT_NUM = 17;

		static vector<string> Split(const string& input, const string& delim);

		SkeletonProcessor& FromString(string skeleton_string);
		SkeletonProcessor& ToUnity();
		SkeletonProcessor& FixView();
		string ToString();

	private:
		vector<k4abt_joint_t> mSkeleton;
		Vector3f mShift;
		bool mHasShift;
		Matrix3f mRotationMatrix;
	};
}

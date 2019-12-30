#include "szl/quaternion_skeleton_processor.h"
using szl_kinect::QuaternionSkeletonProcessor;

#include <iostream>
#include <sstream>
#include <regex>
#include <boost/algorithm/string.hpp>


const map<string, int> QuaternionSkeletonProcessor::kinectJointMap = {
	{"PELVIS", 0},
	{"SPINE_NAVAL", 1},
	{"SPINE_CHEST", 2},
	{"NECK", 3},
	{"CLAVICLE_LEFT", 4},
	{"SHOULDER_LEFT", 5},
	{"ELBOW_LEFT", 6},
	{"WRIST_LEFT", 7},
	{"HAND_LEFT", 8},
	{"HANDTIP_LEFT", 9},
	{"THUMB_LEFT", 10},
	{"CLAVICLE_RIGHT", 11},
	{"SHOULDER_RIGHT", 12},
	{"ELBOW_RIGHT", 13},
	{"WRIST_RIGHT", 14},
	{"HAND_RIGHT", 15},
	{"HANDTIP_RIGHT", 16},
	{"THUMB_RIGHT", 17},
	{"HIP_LEFT", 18},
	{"KNEE_LEFT", 19},
	{"ANKLE_LEFT", 20},
	{"FOOT_LEFT", 21},
	{"HIP_RIGHT", 22},
	{"KNEE_RIGHT", 23},
	{"ANKLE_RIGHT", 24},
	{"FOOT_RIGHT", 25},
	{"HEAD", 26},
	{"NOSE", 27},
	{"EYE_LEFT", 28},
	{"EAR_LEFT", 29},
	{"EYE_RIGHT", 30},
	{"EAR_RIGHT", 31}
};

const vector<string> QuaternionSkeletonProcessor::unityJoints = {
	"PELVIS",
	"HIP_RIGHT",
	"KNEE_RIGHT",
	"ANKLE_RIGHT",
	"HIP_LEFT",
	"KNEE_LEFT",
	"ANKLE_LEFT",
	"SPINE_NAVAL",
	"SPINE_CHEST",
	"NECK",
	"HEAD",
	"SHOULDER_LEFT",
	"ELBOW_LEFT",
	"WRIST_LEFT",
	"SHOULDER_RIGHT",
	"ELBOW_RIGHT",
	"WRIST_RIGHT"
};

QuaternionSkeletonProcessor::QuaternionSkeletonProcessor() :
	mHasShift(false) {
	// Initialize rotation matrix
	mRotationMatrix << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	//mRotationMatrix << 1, 0, 0, 0, 1, 0, 0, -0, 1;
}

QuaternionSkeletonProcessor::QuaternionSkeletonProcessor(k4abt_skeleton_t skeleton) :
	mHasShift(false) {
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		mSkeleton.push_back(skeleton.joints[i]);
	}

	// Initialize rotation matrix
	mRotationMatrix << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
	//mRotationMatrix << 1, 0, 0, 0, 1, 0, 0, -0, 1;
	//mRotationMatrix << 1, 0, 0, 0, 0.939693, 0.34202, 0, -0.34202, 0.939693;
}

QuaternionSkeletonProcessor::~QuaternionSkeletonProcessor() {

}

vector<string> QuaternionSkeletonProcessor::Split(const string& input, const string& delim) {
	regex re{ delim };
	return vector<string> {
		sregex_token_iterator(input.begin(), input.end(), re, -1),
			sregex_token_iterator()
	};
}

QuaternionSkeletonProcessor& QuaternionSkeletonProcessor::ToUnity() {
	vector<k4abt_joint_t> unitySkeleton;
	for (int i = 0; i < unityJoints.size(); i++) {
		int kinectIndex = kinectJointMap.at(unityJoints[i]);
		unitySkeleton.push_back(mSkeleton.at(kinectIndex));
	}
	mSkeleton = unitySkeleton;

	return *this;
}

QuaternionSkeletonProcessor& QuaternionSkeletonProcessor::FixView() {
	// Get skeleton matrix
	Matrix3Xf skeletonMatrix;
	skeletonMatrix.resize(3, mSkeleton.size());
	for (int i = 0; i < mSkeleton.size(); i++) {
		skeletonMatrix(0, i) = mSkeleton.at(i).position.xyz.x;
		skeletonMatrix(1, i) = mSkeleton.at(i).position.xyz.y;
		skeletonMatrix(2, i) = mSkeleton.at(i).position.xyz.z;
	}
	// Rotation
	skeletonMatrix = mRotationMatrix * skeletonMatrix;

	// Get the translation shift
	if (!mHasShift) {
		// Initialize mShift
		float sumX = 0;
		float sumY = 0;
		float minZ = 9999;
		for (int i = 0; i < skeletonMatrix.cols(); i++) {
			sumX += skeletonMatrix(0, i);
			sumY += skeletonMatrix(1, i);

			if (minZ > skeletonMatrix(2, i)) {
				minZ = skeletonMatrix(2, i);
			}
		}
		mShift << -sumX / mSkeleton.size(), -sumY / mSkeleton.size(), -minZ;
		mHasShift = true;
	}

	// Translation
	for (int i = 0; i < skeletonMatrix.cols(); i++) {
		skeletonMatrix(0, i) += mShift(0);
		skeletonMatrix(1, i) += mShift(1);
		skeletonMatrix(2, i) += mShift(2);
	}

	// Make change happen
	assert(mSkeleton.size() == skeletonMatrix.cols());
	for (int i = 0; i < mSkeleton.size(); i++) {
		mSkeleton[i].position.xyz.x = skeletonMatrix(0, i);
		mSkeleton[i].position.xyz.y = skeletonMatrix(1, i);
		mSkeleton[i].position.xyz.z = skeletonMatrix(2, i);
	}

	return *this;
}

QuaternionSkeletonProcessor& QuaternionSkeletonProcessor::FromString(string skeleton_string) {
	boost::trim(skeleton_string);
	auto split_vector = QuaternionSkeletonProcessor::Split(skeleton_string, ",");

	// Clear skeleton vector
	mSkeleton.clear();

	for (int i = 0; i < split_vector.size(); i++) {
		boost::trim(split_vector[i]);
		auto split_float = QuaternionSkeletonProcessor::Split(split_vector[i], " ");

		// Make sure split_float has 3 items
		assert(split_float.size() == 3);

		k4abt_joint_t joint;
		joint.position.xyz.x = atof(split_float[0].c_str());
		joint.position.xyz.y = atof(split_float[1].c_str());
		joint.position.xyz.z = atof(split_float[2].c_str());
		mSkeleton.push_back(joint);
	}

	return *this;
}

string QuaternionSkeletonProcessor::ToString() {
	stringstream ss;
	vector<k4abt_joint_t>::iterator iter;
	for (iter = mSkeleton.begin(); iter != mSkeleton.end(); iter++) {
		ss << iter->position.xyz.x << " ";
		ss << iter->position.xyz.y << " ";
		ss << iter->position.xyz.z << ", ";
	}
	// Output rotation
	ss << "| ";

	int k = 0;
	for (iter = mSkeleton.begin(); iter != mSkeleton.end(); iter++) {
		Quaternionf result = Quaternionf(mRotationMatrix) * Quaternionf(iter->orientation.wxyz.w, iter->orientation.wxyz.x, iter->orientation.wxyz.y, iter->orientation.wxyz.z);
		ss << result.coeffs()(3, 0) << " ";
		ss << result.coeffs()(0, 0) << " ";
		ss << result.coeffs()(1, 0) << " ";
		ss << result.coeffs()(2, 0) << ", ";

		//ss << iter->orientation.wxyz.w << " ";
		//ss << iter->orientation.wxyz.x << " ";
		//ss << iter->orientation.wxyz.y << " ";
		//ss << iter->orientation.wxyz.z << ", ";
	}

	return ss.str();
}

string QuaternionSkeletonProcessor::ToString(vector<k4abt_joint_t> mSkeleton0) {
	stringstream ss;
	vector<k4abt_joint_t>::iterator iter;
	for (iter = mSkeleton0.begin(); iter != mSkeleton0.end(); iter++) {
		ss << iter->position.xyz.x << " ";
		ss << iter->position.xyz.y << " ";
		ss << iter->position.xyz.z << ", ";
	}
	// Output rotation
	ss << "| ";

	int k = 0;
	for (iter = mSkeleton.begin(); iter != mSkeleton.end(); iter++) {
		Quaternionf result = Quaternionf(mRotationMatrix) * Quaternionf(iter->orientation.wxyz.w, iter->orientation.wxyz.x, iter->orientation.wxyz.y, iter->orientation.wxyz.z);
		ss << result.coeffs()(3, 0) << " ";
		ss << result.coeffs()(0, 0) << " ";
		ss << result.coeffs()(1, 0) << " ";
		ss << result.coeffs()(2, 0) << ", ";
		
		//ss << iter->orientation.wxyz.w << " ";
		//ss << iter->orientation.wxyz.x << " ";
		//ss << iter->orientation.wxyz.y << " ";
		//ss << iter->orientation.wxyz.z << ", ";
	}

	return ss.str();
}

string QuaternionSkeletonProcessor::ToString(vector<k4abt_joint_t> mSkeleton0, Matrix3f stereo_rotate) {
	stringstream ss;
	vector<k4abt_joint_t>::iterator iter;
	for (iter = mSkeleton0.begin(); iter != mSkeleton0.end(); iter++) {
		ss << iter->position.xyz.x << " ";
		ss << iter->position.xyz.y << " ";
		ss << iter->position.xyz.z << ", ";
	}
	// Output rotation
	ss << "| ";

	int k = 0;
	for (iter = mSkeleton.begin(); iter != mSkeleton.end(); iter++) {
		Quaternionf result = Quaternionf(mRotationMatrix) * Quaternionf(stereo_rotate) * Quaternionf(iter->orientation.wxyz.w, iter->orientation.wxyz.x, iter->orientation.wxyz.y, iter->orientation.wxyz.z);
		ss << result.coeffs()(3, 0) << " ";
		ss << result.coeffs()(0, 0) << " ";
		ss << result.coeffs()(1, 0) << " ";
		ss << result.coeffs()(2, 0) << ", ";

		//ss << iter->orientation.wxyz.w << " ";
		//ss << iter->orientation.wxyz.x << " ";
		//ss << iter->orientation.wxyz.y << " ";
		//ss << iter->orientation.wxyz.z << ", ";
	}

	return ss.str();
}

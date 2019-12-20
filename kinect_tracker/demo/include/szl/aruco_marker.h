#pragma once
namespace szl_kinect {
	class ArucoMarker
	{
	public:
		int Create();
		int Calibrate();
		int PoseEstimate();
		int DrawCube();
	};
}

#pragma once
#include <k4a/k4a.hpp>

#define VERIFY(result, error)																				\
	if (result != K4A_RESULT_SUCCEEDED)																		\
	{																										\
		printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__);	\
		exit(1);																							\
	}																										\

namespace szl_kinect {



	//class SingleStartup
	//{
	//public:
	//	int Run();
	//};
}



#pragma once
#pragma once
#include <string>
#include "inline/SkeletonNode/s_skeletonNode.h"


namespace Export {
	void SaveQuaternionsToFile(skl::SkeletonNode *skeleton, std::string fileName);
	void SaveAxisAngleToFile(skl::SkeletonNode *skeleton, std::string fileName);
	void SaveMatricesToFile(skl::SkeletonNode *skeleton, std::string fileName);
}
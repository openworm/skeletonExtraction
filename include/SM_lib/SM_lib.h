#pragma once
#include <map>
#include <vector>
#include <SkeletonNode\SkeletonNode.h>
#include "MatchingStruct.h"

std::map<int, MatchingSkeletonStruct> MatchSkeletons(SN::SkeletonNode* skl1, SN::SkeletonNode* skl2, float threshold);
std::vector<SN::SkeletonNode* > MatchSkeletons(std::vector<SN::SkeletonNode *> skeletons, std::vector<MatchingStruct>& output, float thresholdPercent = 0.33, float angleThreshold = 20, float axisAngleThreshold = 10, bool symmetric = false);
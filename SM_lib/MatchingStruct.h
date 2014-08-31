#pragma once
#include <vector>

struct MatchingStruct {
	int id;
	int matched;
	int DoFs;

	MatchingStruct(int _id, int _matched, int _Dofs) : id(_id), matched(_matched), DoFs(_Dofs) {};
};

struct MatchingSkeletonStruct {
	std::vector<int> matched;

	MatchingSkeletonStruct() {};
	MatchingSkeletonStruct(int id) { matched.push_back(id); };
};
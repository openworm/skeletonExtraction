#pragma once;
#include <vector>

using namespace std;

struct ValueMatchingStruct
{
public:
	float value;
	vector<int> matching;
	ValueMatchingStruct() : value(0) {};
	ValueMatchingStruct(float val, vector<int>& match) : value(val), matching(match) {};
	bool operator< (const ValueMatchingStruct& rhs) { return value < rhs.value; };
};
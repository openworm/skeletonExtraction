#pragma once
#include <vector>

using namespace std;

template< typename T> bool inArray(vector<T> &array, T elem) {
	for (int i = 0; i < array.size(); i++) {
		if (array[i] == elem) return true;
	}

	return false;
}
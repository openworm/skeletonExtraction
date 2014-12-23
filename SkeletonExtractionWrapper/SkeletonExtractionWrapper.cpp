// SkeletonExtractionWrapper.cpp : Defines the entry point for the console application.
//
#include <tchar.h>
#include "wrapper.h"

int main(int argc, char* argv[])
{
	int timeStep = std::atoi(argv[1]);
	coreFunc(timeStep);
	return 1;
}





// SkeletonExtractionWrapper.cpp : Defines the entry point for the console application.
//
#include <tchar.h>

#include <meshes/structure.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <SDF_lib/sdf_extractor.h>
#include <SDF_lib/Helpers/SDFSettings.h>

#include <Import_lib/WormLoader.h>
#include <Export_lib/ColladaExporter.h>
#include <Export_lib/MatrixExport.h>

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include "openworm_wrapper.h"

int main(int argc, char* argv[])
{
	int timeStep = std::atoi(argv[1]);

	string filePath = string("D:\\Projects\\_shared\\Skeletor\\Models\\OpenWorm\\Export\\");
	Export::ColladaExporter exporter;
	Openworm_Wrapper ow;

	ow.oSDFExtractor.skeletonTesselationFactor = 1;

	ow.oLBSExtractor.sdfHalfVectors = new CVector3[0];
	ow.oLBSExtractor.sdfHalfVectorsMG = new CVector3[0];

	Nastavenia->CLKernelPath = std::string("D:\\Projects\\OpenWorm\\SDF_lib\\OpenCLKernels\\");
	Nastavenia->SDF_Mode = SDF_CPU;

	skl::SkeletonNode * bindPoseSkeleton = new skl::SkeletonNode();

	meshes::Mesh  * meshBindPose = new meshes::Mesh();
	ow.skeletonExtractionSDF(0, filePath, &ow.exporter, bindPoseSkeleton, meshBindPose);
	ow.pSkeletonRootBindPose = bindPoseSkeleton;
	delete meshBindPose;

	ow.calculateAndExportSkeletonTransformationsForTimestep(timeStep, filePath, &exporter, NULL);

	delete bindPoseSkeleton;

	delete[] ow.oLBSExtractor.sdfHalfVectors;
	delete[] ow.oLBSExtractor.sdfHalfVectorsMG;

	return 1;
}




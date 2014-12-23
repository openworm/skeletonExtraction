#include "wrapper.h"

void coreFunc(int timeStep){
	try {
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
		ow.bindPoseSegmentLengths = ow.calculateSkeletonSegmentLengths(bindPoseSkeleton);
		delete meshBindPose;

		ow.calculateAndExportSkeletonTransformationsForTimestep(timeStep, filePath, &exporter, NULL);

		delete bindPoseSkeleton;

		delete[] ow.oLBSExtractor.sdfHalfVectors;
		delete[] ow.oLBSExtractor.sdfHalfVectorsMG;
	} catch (...){
		std::ofstream outfile ("leaks.txt");
		outfile << timeStep;
	    }
}
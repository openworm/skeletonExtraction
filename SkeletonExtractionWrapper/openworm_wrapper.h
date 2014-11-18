#pragma once

#define OW_NUM_OF_CTRL_BONES 4
#define OW_MAX_BONE_MAT 24

#define OW_MAX_WORM_ITERATIONS 4000
#define OW_BINDPOSE_TIMESTEP 3000
#define OW_SINPLIFY_TRIANGLECOUNT 3000
#define OW_ITE_TO_LOAD 200

#include <tchar.h>

#ifdef _LOG
	#include <logs/log.h>
#endif

#include <meshes/structure.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <SDF_lib/sdf_extractor.h>

#include <Import_lib/WormLoader.h>
#include <Import_lib/m_assimpLoader.h>
#include <Export_lib/ColladaExporter.h>
#include <Export_lib/MatrixExport.h>

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include <PCT_lib/pct_PointCloudTriangulation.h>

#include <inline/SkeletonNode/s_skeletonNode.h>

struct Openworm_Wrapper {

	Assimp::Importer importer;

	meshes::IndexedFace * mesh;
	meshes::IndexedFace meshToAdd;
	GLuint meshToAddGLDVerticesId;
	GLuint meshToAddGLDIndicesId;
	skl::SkeletonNode * pSkeletonRootBindPose;
	skl::SkeletonNode * pSkeletonRootForTimestep;
	lbse::Extractor oLBSExtractor;
	sdf::Extractor oSDFExtractor;

	int simulationDirection;
	int wantedNumOfSkeletonNodes;

	Import::WormLoader wormLoader;
	Export::ColladaExporter exporter;
	float adaptVMdmax;

	float maxSDF;
	float minSDF;

	int timeStep;

	bool previewPaused;
	int animTimestep;
	bool manualAnimTimestep;
	bool timeStepChangedProgramaticaly;
	bool b_addSimulationModel;

	Openworm_Wrapper();

	void calculateWormSimulationSkeletonsFromMeshes(string dir);
	void skeletonExtractionSDF(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput, meshes::Mesh  * meshImport);
	void skeletonExtractionSDF_calculateSDF(meshes::Mesh * mesh, lbse::Extractor * pLBSExtractor);
	void skeletonExtractionSDF_extractSkeleton(SN::SkeletonNode * pSkeletonRoot, lbse::Extractor * pLBSExtractor, sdf::Extractor * pSDFExtractor);
	void loadModelToStructures(lbse::Extractor * pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance);
	void calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat);

	// main thread calls

	void prepareGLDDataForSkinningPreview();
	void prepareGLDDataForSimulationPreview(string positionsFile, string membraneFile);
	void playSimulationAndSkinningPreview();
	void pauseSimulationAndSkinningPreview();
	void updateSimulationModelFromMainThread();
	void calculateAndExportSkeletonTransformations(string filePath, Export::ColladaExporter * exporter);
	void calculateAndExportSkeletonTransformationsForTimestep(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput);
	void calculateModelMaxDim(MeshGraph * pMesh, float * adaptVMdmax);

	// animation thread calls

	void renderSkinningPreview();
	void renderSimulationPreview();
	void addSimulationModel(meshes::IndexedFace * mesh);
	void renderSimulationAndSkinningPreview(bool * finished, bool force);

	// skeleton extraction for muscles

	void extractMuscleSkeleton(meshes::IndexedFace * muscle, skl::SkeletonNode * pSkelet);
};

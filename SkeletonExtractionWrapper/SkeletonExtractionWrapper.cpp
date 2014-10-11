// SkeletonExtractionWrapper.cpp : Defines the entry point for the console application.
//
#include <tchar.h>

#include <meshes/structure.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <SDF_lib/sdf_extractor.h>

#include <Import_lib/WormLoader.h>
#include <Export_lib/ColladaExporter.h>
#include <Export_lib/MatrixExport.h>

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include "Wrapper.h"


int _tmain(int argc, _TCHAR* argv[])
{

/*******************************************************************************************************************************/
//                                    PATHS
/*******************************************************************************************************************************/

	string positionsFile;
	string membraneFile;
	string colladaMeshFile;
	string colladaMeshAndSkeletonFile;
	string colladaAnimationFile;

	string transformationsMatrixFile;
	string transformationsQuaternionFile;

/*******************************************************************************************************************************/
//                                    BASIC OBJECTS
/*******************************************************************************************************************************/

	SN::SkeletonNode * pSkeletonRoot = new SN::SkeletonNode();
	lbse::Extractor oLBSExtractor;
	sdf::Extractor oSDFExtractor;
	meshes::IndexedFace * mesh = new meshes::IndexedFace();

	Import::WormLoader wormLoader;
	Export::ColladaExporter exporter;

/*******************************************************************************************************************************/
//                                    SETTINGS, PARAMS ...
/*******************************************************************************************************************************/

	#define NUM_OF_CTRL_BONES 4
	#define MAX_BONE_MAT 24

	#define MAX_WORM_ITERATIONS 4000

	float joiningTolerance = 0.0;
	//oLBSExtractor.laplacianScheme = LS_GLOBAL_JAMA_COTANGENT;
	//oLBSExtractor.numOfIter = 4;
	oLBSExtractor.groupingTolerance = 2.5;
/*	oLBSExtractor.wA = 1.0;
	oLBSExtractor.wB = 0.1;
	oLBSExtractor.wL = 1.0;
	oLBSExtractor.wC = 1.0;
	oLBSExtractor.wH = 1.0;
	oLBSExtractor.sL = 3.0;*/
	oLBSExtractor.wantedNumOfBones = 24;

/*******************************************************************************************************************************/
	// [1] Read the worm particles w/wo triangulation and transform it into meshgraph stucture (using importer)
/*******************************************************************************************************************************/

	// - call Loader_lib, load t3DModel

	wormLoader.SetWormFile(positionsFile);
	wormLoader.SetWormMeshFile(membraneFile);

	wormLoader.ReadWormIteration(mesh, 3000);
	wormLoader.ReadWormMesh(mesh);

	createMeshGraph(mesh, oLBSExtractor.pMesh, oLBSExtractor.wL, oLBSExtractor.wH);

	structure::t3DModel g_3DModel;
	int adaptVMdmax = 0;

	// load model from Worm file, timestep 1

	//loadModelToStructures(&oLBSExtractor, &g_3DModel, &adaptVMdmax, joiningTolerance);


/*******************************************************************************************************************************/
	// [2] Perform SDF median skeleton extraction on the meshgraph
/*******************************************************************************************************************************/

	skeletonExtractionSDF(pSkeletonRoot, &oLBSExtractor, &oSDFExtractor, &adaptVMdmax);
	
/*******************************************************************************************************************************/
	// [3] Calculate skinning weights using inverse geodesic distance
/*******************************************************************************************************************************/

	// - calculate weights using GDSW_lib

	GraphAlgorithms ga;
	Array2D<float> distanceMatrix;

	ga.FloydWarshall(oLBSExtractor.pMesh, distanceMatrix);

	ObjectSkeletonShaderData wormData;

	calculateSkinningData(&ga, &wormData, &g_3DModel, pSkeletonRoot, &distanceMatrix, &oLBSExtractor, NUM_OF_CTRL_BONES, MAX_BONE_MAT);


/*******************************************************************************************************************************/
	// [3] Export mesh and mesh with extracted skeleton and weights into Collada (using Collada exporter)
/*******************************************************************************************************************************/

	// - call Exporter_lib to store mesh and skinning data in .dae

	// export mesh only
	exporter.Export(mesh, colladaMeshFile);

	// export mesh and skinning data
	meshes::MeshSkin * meshSkin = new meshes::MeshSkin(mesh);

	for (int i=0; i < meshSkin->indices.size() / 3; i++){
		meshSkin->influences.push_back(NUM_OF_CTRL_BONES);
	}

	for (int i=0; i < (meshSkin->indices.size() / 3) * NUM_OF_CTRL_BONES; i++){
		meshSkin->jointIDs.push_back(wormData.indices[i]);
	}

	for (int i=0; i < (meshSkin->indices.size() / 3) * NUM_OF_CTRL_BONES; i++){
		meshSkin->skinWeights.push_back(wormData.weights[i]);
	}

	exporter.Export(meshSkin, pSkeletonRoot, colladaMeshAndSkeletonFile);

/*******************************************************************************************************************************/
//	Loop through all theE timesteps of the simulation - extract skeletons and compute skeleton transformations
/*******************************************************************************************************************************/

	int timestep = 0;

	while (timestep < MAX_WORM_ITERATIONS){

		timestep++;

		// [4] Read other worm particle positions (from other timesteps) and extract skeletons

		// repeat steps [1-2] for other timesteps

		skl::SkeletonNode * pSkeletonRootNext = new skl::SkeletonNode();
		int iterationNum;

		// load model from Worm file, timestep 2

		loadModelToStructures(&oLBSExtractor, &g_3DModel, &adaptVMdmax, joiningTolerance);

		skeletonExtractionSDF(pSkeletonRootNext, &oLBSExtractor, &oSDFExtractor, &adaptVMdmax);

		// save skeleton from each timestep for debuging

		// [5] Calculate the transformation between two skeletons (two different timesteps of simulation)

		// used SM_lib to compute the transformations

		SM::CalculateWormTransformationMatrices((skl::SkeletonNode*)pSkeletonRoot, pSkeletonRootNext);

		// [6] Export the transformations into separate files

		Export::SaveQuaternionsToFile(pSkeletonRootNext, transformationsQuaternionFile);
		Export::SaveMatricesToFile(pSkeletonRootNext, transformationsMatrixFile);

		// use Exporter_lib

	}

/*******************************************************************************************************************************/
//	Export whole animation into Collada file
/*******************************************************************************************************************************/

	exporter.Export(meshSkin, pSkeletonRoot, colladaAnimationFile);

	delete[] wormData.indices;
	delete[] wormData.weights;

	return 0;
}




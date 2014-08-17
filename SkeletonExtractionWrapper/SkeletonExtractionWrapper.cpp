// SkeletonExtractionWrapper.cpp : Defines the entry point for the console application.
//
#include <tchar.h>

#include <meshes/structure.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <Import_lib/WormLoader.h>
#include <Export_lib/ColladaExporter.h>

#include "Wrapper.h"


int _tmain(int argc, _TCHAR* argv[])
{

/*******************************************************************************************************************************/
//                                    BASIC OBJECTS
/*******************************************************************************************************************************/

	SN::SkeletonNode * pSkeletonRoot = new SN::SkeletonNode();
	LBSE::Extractor oLBSExtractor;

/*******************************************************************************************************************************/
//                                    SETTINGS, PARAMS ...
/*******************************************************************************************************************************/

	#define NUM_OF_CTRL_BONES 4
	#define MAX_BONE_MAT 24

	float joiningTolerance = 0.0;
	oLBSExtractor.laplacianScheme = LS_GLOBAL_JAMA_COTANGENT;
	oLBSExtractor.numOfIter = 4;
	oLBSExtractor.groupingTolerance = 5;
	oLBSExtractor.wA = 1.0;
	oLBSExtractor.wB = 0.1;
	oLBSExtractor.wL = 1.0;
	oLBSExtractor.wC = 1.0;
	oLBSExtractor.wH = 1.0;
	oLBSExtractor.sL = 3.0;
	oLBSExtractor.wantedNumOfBones = 24;

/*******************************************************************************************************************************/
	// [1] Read the worm particles w/wo triangulation and transform it into meshgraph stucture (using importer)
/*******************************************************************************************************************************/

	// - call Loader_lib, load t3DModel

	t3DModel g_3DModel;
	int adaptVMdmax;

	// load model from Worm file, timestep 1

	loadModelToStructures(&oLBSExtractor, &g_3DModel, &adaptVMdmax, joiningTolerance);


/*******************************************************************************************************************************/
	// [2] Perform skeleton extraction on the meshgraph
/*******************************************************************************************************************************/

	// - call LBSE_lib, store skeleton in SN:SkeletonNode

	int iterationNum;

	// set settings to extractor

	oLBSExtractor.computeSkeleton(&g_3DModel, 0, pSkeletonRoot, &iterationNum, adaptVMdmax);

	//int currentNumberOfSkeletonBones = oLBSExtractor.currentNumberOfSkeletonBones;
	
/*******************************************************************************************************************************/
	// [3] Calculate skinning weights using inverse geodesic distance
/*******************************************************************************************************************************/

	// - calculate weights using GDSW_lib

	GraphAlgorithms ga;
	Array2D<float> distanceMatrix;

	ga.FloydWarshall(oLBSExtractor.pMesh, distanceMatrix);

	ObjectSkeletonShaderData wormData;

	int offset = 0;

	for(int i = 0; i < g_3DModel.numOfObjects; i++){
		if(g_3DModel.pObject.size() <= 0) break;
		t3DObject *pObject = &g_3DModel.pObject[i];

		ObjectSkeletonShaderData skeletonData;

		skeletonData.weights = new float[pObject->numOfVertices * NUM_OF_CTRL_BONES];
		skeletonData.indices = new float[pObject->numOfVertices * NUM_OF_CTRL_BONES];

		ga.computeObjectSkeletonShaderData(pSkeletonRoot, oLBSExtractor.pMesh, pObject, &skeletonData, pObject->numOfVertices, offset, true, distanceMatrix, NUM_OF_CTRL_BONES, MAX_BONE_MAT);

		// copy skeletonData

		wormData.weights = new float[skeletonData.numOfVertices * NUM_OF_CTRL_BONES];
		wormData.indices = new float[skeletonData.numOfVertices * NUM_OF_CTRL_BONES];
		memcpy(skeletonData.indices, skeletonData.indices, skeletonData.numOfVertices * NUM_OF_CTRL_BONES * sizeof(float));
		memcpy(skeletonData.weights, skeletonData.weights, skeletonData.numOfVertices * NUM_OF_CTRL_BONES * sizeof(float));	

		offset += pObject->numOfVerts;
	}



/*******************************************************************************************************************************/
	// [3] Export mesh, extracted skeleton and weights into Collada (using Collada exporter)
/*******************************************************************************************************************************/

	// - call Exporter_lib to store mesh and skinning data in .dae

/*******************************************************************************************************************************/
	// [4] Read other worm particle positions (from other timesteps) and extract skeletons
/*******************************************************************************************************************************/

	// repeat steps [1-2] for other timesteps

	SN::SkeletonNode * pSkeletonRoot2 = new SN::SkeletonNode();

	// load model from Worm file, timestep 2

	loadModelToStructures(&oLBSExtractor, &g_3DModel, &adaptVMdmax, joiningTolerance);

	oLBSExtractor.computeSkeleton(&g_3DModel, 0, pSkeletonRoot, &iterationNum, adaptVMdmax);

/*******************************************************************************************************************************/
	// [5] Calculate the transformation between two skeletons (two different timesteps of simulation)
/*******************************************************************************************************************************/

	// used SM_lib to compute the transformations ??? or another lib ?

/*******************************************************************************************************************************/
	// [6] Export the transformations into separate files
/*******************************************************************************************************************************/

	// use Exporter_lib ? or the wrapper should do this functionality ?

	return 0;
}




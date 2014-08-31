#include "Wrapper.h"

void loadModelToStructures(lbse::Extractor * pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance){
	float dx = pModel->modelbb.x_max - pModel->modelbb.x_min;
	float dy = pModel->modelbb.y_max - pModel->modelbb.y_min;
	float dz = pModel->modelbb.z_max - pModel->modelbb.z_min;

	// Maximal size
	*adaptVMdmax = dx > dy ? dx : dy;
	*adaptVMdmax = *adaptVMdmax > dz ? *adaptVMdmax : dz;

	// create MeshGraph from model

	float * thresholds = new float[pModel->numOfObjects];
	float delta = *adaptVMdmax * 0.00001f;
	for (int i=0; i < pModel->numOfObjects; i++){
		thresholds[i] = *adaptVMdmax / (100.0f / joiningTolerance);
		if (thresholds[i] > FLT_MAX)
			thresholds[i] = 0.0f;
	}

	// REMOVE, NOT NEEDED IN OPENWORM
	VertexJoiningVisualization * pJoining = new VertexJoiningVisualization();

	// REMOVE COMPONENTS, NOT NEEDED IN OPENWORM
	createMeshGraph(pModel, pLBSExtractor->pMesh, pJoining, thresholds, delta, pLBSExtractor->wL, pLBSExtractor->wH, -1, vector<int>());

	copyMeshGraph(pLBSExtractor->pMesh, pLBSExtractor->originalMesh);

	// - if only pointcloud is loaded, use PCT_lib to get global triangulation

	computeTriangulationFromLocalDelaunays(pLBSExtractor->pMesh, pLBSExtractor->pTriangulator);
}

void calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat){
	int offset = 0;

	for(int i = 0; i < pModel->numOfObjects; i++){
		if(pModel->pObject.size() <= 0) break;
		structure::t3DObject *pObject = &(pModel->pObject[i]);

		ObjectSkeletonShaderData skeletonData;

		skeletonData.weights = new float[pObject->numOfVertices * numOfCtrlBones];
		skeletonData.indices = new float[pObject->numOfVertices * numOfCtrlBones];

		pGa->computeObjectSkeletonShaderData(pSkeletonRoot, pLBSExtractor->pMesh, pObject, &skeletonData, pObject->numOfVertices, offset, true, *pDistanceMatrix, numOfCtrlBones, maxBoneMat);

		// copy skeletonData

		pWormData->weights = new float[skeletonData.numOfVertices * numOfCtrlBones];
		pWormData->indices = new float[skeletonData.numOfVertices * numOfCtrlBones];
		memcpy(skeletonData.indices, pWormData->indices, skeletonData.numOfVertices * numOfCtrlBones * sizeof(float));
		memcpy(skeletonData.weights, pWormData->weights, skeletonData.numOfVertices * numOfCtrlBones * sizeof(float));	

		offset += pObject->numOfVerts;
	}
}
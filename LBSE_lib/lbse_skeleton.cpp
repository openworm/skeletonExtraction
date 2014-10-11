//---------------------------------------------------------------------------
#include "lbse_skeleton.h"


void assignFathersForSkeletonTree(SN::SkeletonNode * pNode){
	//pNode->father = new SN::SkeletonNode();
	//pNode->father->point = CVector3(0,0,0);
	pNode->father = NULL;

	vector<SN::SkeletonNode*> queue;
	queue.push_back(pNode);

	while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();

		for (int i=0; i < n->nodes.size(); i++){
			n->nodes[i]->father = n;

			queue.push_back(n->nodes[i]);
		}
	}
}

void copySkeletonNode(SN::SkeletonNode * src, SN::SkeletonNode * dest){
	copySkeletonNodeImpl(src, dest);
	assignFathersForSkeletonTree(dest);
}

void copySkeletonNodeImpl(SN::SkeletonNode * src, SN::SkeletonNode * dest){

	dest->id = src->id;
	dest->point = src->point;
	dest->nodes = vector<SN::SkeletonNode*>();
	//dest->father = src->father;
	for (int i = 0; i < src->nodes.size(); i++) {
		SN::SkeletonNode * p = new SN::SkeletonNode();
		copySkeletonNode(src->nodes[i], p);
		p->father = dest;
		dest->nodes.push_back(p);
	}

}

//---------------------------------------------------------------------------

void copyMeshGraph(MeshGraph * in, MeshGraph * out){
	out->numOfVertices = in->numOfVertices;
	out->faceAreaSum = in->faceAreaSum;
	out->numOfFaces = in->numOfFaces;
	out->wL = in->wL;
	out->wC = in->wC;
	delete[] out->wH;
	out->wH = NULL;
	out->wH = new float[in->numOfVertices];
	delete[] out->wHorig;
	out->wHorig = NULL;
	out->wHorig = new float[in->numOfVertices];
	delete[] out->origOneRingArea;
	out->origOneRingArea = NULL;
	out->origOneRingArea = new float[in->numOfVertices];
	out->origOneRingExtent = NULL;
	out->origOneRingExtent = new float[in->numOfVertices];
	delete[] out->pVerts;
	out->pVerts = NULL;
	out->pVerts = new CVector3[in->numOfVertices];
	out->E = in->E.copy();
	delete[] out->triangleIndices;
	out->triangleIndices = NULL;

	for (int i = 0; i < in->numOfVertices; i++) {
		out->wH[i] = in->wH[i];
		out->wHorig[i] = in->wHorig[i];
		out->origOneRingArea[i] = in->origOneRingArea[i];
		out->origOneRingExtent[i] = in->origOneRingExtent[i];
		out->pVerts[i] = in->pVerts[i];
	}

	out->indices = in->indices;
	//out->glVertexBufferID = in->glVertexBufferID;
	out->numOfVBOlines = in->numOfVBOlines;
	//out->inverseIndices = in->inverseIndices;

	out->triangleIndices = new int[in->numOfFaces * 3];
	for (int i=0; i < in->numOfFaces * 3; i++){
		out->triangleIndices[i] = in->triangleIndices[i];
	}

	/*out->neighPCIndex = in->neighPCIndex;
	out->neighVis = in->neighVis;*/
}

//---------------------------------------------------------------------------
vector<CVector3> findNeighborhood(int ind, vector<CVector3> v, float r){
	vector<CVector3> n;
	for (int i=0; i < v.size(); i++)
	if (Distance(v[ind], v[i]) < r)
		n.push_back(v[i]);
	return n;
}


//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

void collapseTriangle(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int i, int j, int k){
	CVector3 center = (pMesh->pVerts[k] + pMesh->pVerts[i] + pMesh->pVerts[j]) / 3;

	#ifdef _LOG
		logg.log(LOG_LEVEL_WARNING, "Collapsujem, trojuholnik");
		logg.log(LOG_LEVEL_WARNING, "i: ",i);
		logg.log(LOG_LEVEL_WARNING, "j: ",j);
		logg.log(LOG_LEVEL_WARNING, "k: ",k);
	#endif

		for (int m = 0; m < pMesh->numOfVertices; m++) {
			if (pMesh->E[j][m] && k != m) {
				pMesh->E[k][m] = true;
				pMesh->E[m][k] = true;

				pMesh->E[j][m] = false;
				pMesh->E[m][j] = false;
			}
			if (pMesh->E[i][m] && k != m) {
				pMesh->E[k][m] = true;
				pMesh->E[m][k] = true;

				pMesh->E[i][m] = false;
				pMesh->E[m][i] = false;
			}
		}

	#ifdef _LOG
			pMesh->E[j][k] = false;
			pMesh->E[k][j] = false;
			pMesh->E[k][i] = false;
			pMesh->E[i][k] = false;
	#endif
}

bool manifoldCheck(MeshGraph * pMesh, int e1, int e2, int i, int j){
	// kontrola ci bude edge (e1,e2) splnat manifold conectivitu pod half-edge collapse i->j
	Array2D<bool> tmpE = pMesh->E.copy();

	for (int k = 0; k < pMesh->numOfVertices; k++) {
		if (tmpE[i][k] && j != k) {
			tmpE[j][k] = true;
			tmpE[k][j] = true;

			tmpE[i][k] = false;
			tmpE[k][i] = false;
		}
	}

		tmpE[j][i] = false;
		tmpE[i][j] = false;

	int ind=0;
	for (int k = 0; k < pMesh->numOfVertices; k++){
		if (tmpE[i][k] && tmpE[j][k] && i != k && j != k && isMeshTriangle(pMesh, i,j,k)){
				ind++;
		}
	}
	return (ind == 2);
}


//---------------------------------------------------------------------------
int generateIdForTree(SN::SkeletonNode * pNode, vector<int> &meshGraphIndices){
	int idx = 1;
	meshGraphIndices.clear();
	//pNode->id = idx;
	//idx++;

	vector<SN::SkeletonNode*> queue;
	queue.push_back(pNode);

	while(queue.size() > 0){
		SN:;SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();

		meshGraphIndices.push_back(n->id - 1);
		n->id = idx++;

		for (int i = 0; i < n->nodes.size(); i++)
			queue.push_back(n->nodes[i]);
	}

	return idx;
	
	//for (int i = 0; i < pNode->nodes.size(); i++){
	//	generateIdDSF(pNode->nodes[i], &idx);
	//}
	// rather use depthfirst search
}

int generateIdForTree(SN::SkeletonNode * pNode){
	int idx = 1;

	vector<SN::SkeletonNode*> queue;
	queue.push_back(pNode);

	while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();

		n->id = idx++;

		for (int i = 0; i < n->nodes.size(); i++)
			queue.push_back(n->nodes[i]);
	}

	return idx;
}



//---------------------------------------------------------------------------
SN::SkeletonNode * findNodeWithIdInTree(SN::SkeletonNode * node, int id){
	if (node->id == id)
		return node;
	for (int i = 0; i < node->nodes.size(); i++){
	   SN::SkeletonNode * son = findNodeWithIdInTree(node->nodes[i], id);
	   if (son != NULL)
			return son;
	}
	//SN::SkeletonNode * nullNode = new SN::SkeletonNode();
	//nullNode->id = -1;
	//return nullNode;
	return NULL;
}

SN::SkeletonNode * findNodeWithId(SN::SkeletonNode * pRoot, int id){
	vector<SN::SkeletonNode*> queue;
	queue.push_back(pRoot);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		if (pNode->id == id)
			return pNode;

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}

//---------------------------------------------------------------------------
// returns the shortest way between 2 points on the mesh graph structure
/*float GeodesicDistance(MeshGraph * pMesh,  Array2D<float> distanceMatrix, CVector3 p1, int vertexIndex, bool firstSkeletonPoint){
	int idx1 = findClosestPointIndex(pMesh, p1);
	int idx2 = vertexIndex;
	if (idx1 == -1 || idx2 == -1)
		return FLT_MAX;
	if (idx1 == idx2)
		return Distance(p1, pMesh->pVerts[idx2]);
	//if (firstSkeletonPoint)
	//	return distanceMatrix[idx1][idx2] + Distance(p1, pMesh->pVerts[idx1]);
	float d = distanceMatrix[idx1][idx2] + Distance(p1, pMesh->pVerts[idx1]);
	return d;
}*/

//---------------------------------------------------------------------------
// method finds x closest bones from skeleton subtree to the point
/*void findXMinInTree(MeshGraph * pMesh, Array2D<float> distanceMatrix, int x, SN::SkeletonNode *node, SN::SkeletonNode * mins, int vertexIndex){

	SN::SkeletonNode maxNode;
	maxNode.id = -1;
	maxNode.point = CVector3(FLT_MAX, FLT_MAX, FLT_MAX);

	for (int i = 0; i < x ; i++){
		mins[i] = maxNode;
	}

	vector<SN::SkeletonNode*> queue;
	queue.push_back(node);

	while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();
		for (int i = 0; i < n->nodes.size(); i++)
			queue.push_back(n->nodes[i]);

		// find max distanced node in mins array
		int maxIndx = 0;
		float maxDist = 0.0;
		for (int j = 0; j < x; j++) {
			float dist = GeodesicDistance(pMesh, distanceMatrix, mins[j].point, vertexIndex, true);
			if (dist > maxDist) {
				maxIndx = j;
				maxDist = dist;
			}
		}
		//and replace him
		float baseDist = GeodesicDistance(pMesh, distanceMatrix, n->point, vertexIndex, true);
		if (baseDist < maxDist) {
			mins[maxIndx] = *n;
		}
	}
}*/

/*void destroySkeletonNodeSubtree(SN::SkeletonNode * node){
	for (int i=0; i<node->nodes.size(); i++){
		destroySkeletonNodeSubtree(node->nodes[i]);
		delete node->nodes[i];
		node->nodes[i] = NULL;
	}	
}*/

//---------------------------------------------------------------------------
// Connectivity surgery
void createSurgeryGraphFromMeshGraph(MeshGraph * pMesh, SurgeryGraph * pGraph){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createSurgeryGraphFromMeshGraph STARTED");
	#endif
	pGraph->numOfVertices = pMesh->numOfVertices;
	pGraph->E = Array2D< bool >(pMesh->numOfVertices, pMesh->numOfVertices, false);
	delete[] pGraph->pVerts;
	pGraph->pVerts = NULL;
	pGraph->pVerts = new CVector3[pMesh->numOfVertices];

	for (int i = 0; i < pMesh->numOfVertices; i++) {
		pGraph->pVerts[i] = pMesh->pVerts[i];
		for (int j = 0; j < pMesh->numOfVertices; j++){
				pGraph->E[i][j] = pMesh->E[i][j];
		}
	}
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createSurgeryGraphFromMeshGraph ENDED");
	#endif
}

vector<int> getDegenerateIndex(boost::unordered_map<int, vector<int> > mgDegeneratesMapping, int idx){
	return mgDegeneratesMapping[idx];
	/*vector<int> veci;
	veci.push_back(idx);
	return veci;*/
}
//---------------------------------------------------------------------------
float computePointCloudVolume(vector<int> indices, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
	// approximate like avarage distance from centroid
	CVector3 centroid = CVector3(0, 0, 0);
	int num = 0;
	for (int i = 0; i < indices.size(); i++) {
		for (int j = 0; j < getDegenerateIndex(mgDegeneratesMapping, indices[i]).size(); j++){
			centroid = centroid + modelVertices[getDegenerateIndex(mgDegeneratesMapping, indices[i])[j]];
			num ++;
		}
	}
	centroid = centroid / num;

	float dist = 0.0;
	for (int i = 0; i < indices.size(); i++) {
		for (int j = 0; j < getDegenerateIndex(mgDegeneratesMapping, indices[i]).size(); j++){
			dist += Distance(modelVertices[getDegenerateIndex(mgDegeneratesMapping, indices[i])[j]], centroid);
		}
	}
	dist = dist / num;
	return dist;
}
//---------------------------------------------------------------------------

void collapseCloseVertices(SurgeryGraph * pGraph, float groupingTolerance, float groupingToleranceSDFMulti, int * actualNumOfBones, int * wantedNumberOfBones, bool groupingWithoutEdge, CVector3 * sdfValues, bool useSDFBasedGroupingDistance){
	if (groupingTolerance > 0 || useSDFBasedGroupingDistance){
	   float distThreshold = groupingTolerance;
	   vector<int> good;
		for (int i = 0; i < pGraph->numOfVertices; i++){
			bool add = true;
			bool re = false;
			for (int j = 0; j < good.size(); j++){
				if ((pGraph->E[i][good[j]] || groupingWithoutEdge) && i != good[j]) {
					float dist = Distance(pGraph->pVerts[i], pGraph->pVerts[good[j]]);
					if (useSDFBasedGroupingDistance){
						distThreshold = groupingTolerance + (Magnitude(sdfValues[i]) * groupingToleranceSDFMulti);
					}
					if (dist < distThreshold){
						add = false;
						for (int k = 0; k < i; k++)
							if (pGraph->E[i][k]){
								// have to be repated, edges has changed
								re = true;
								break;
							}
						halfEdgeCollapse(pGraph, i, good[j], actualNumOfBones, false);
						if (re){
							i = -1;
							good.clear();
							break;
						}
					}
				}
			}
			if (add)
				good.push_back(i);
		}
	}
}

//---------------------------------------------------------------------------
void createSkeletonFromSurgeryGraphSDFTest(SurgeryGraph * pGraph, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping , SN::SkeletonNode * root, int * wantedNumberOfBones, float groupingTolerance, float groupingToleranceSDFMulti, bool doBranchingSimplification,bool doDisplacementShifting, bool groupingWithoutEdge, bool cyclicSkeleton, CVector3 * sdfValues, bool useSDFBasedGroupingDistance){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createSkeletonFromSurgeryGraph STARTED");
	#endif
	int actualNumOfBones = pGraph->numOfVertices;

	//logg.log(0, "Matica E PRED");
	//logg.log(0, pGraph->E);

	for (int i = 0; i < pGraph->numOfVertices; i++)
		pGraph->pointClouds[i].push_back(i);

	Array2D<bool> originalE = Array2D< bool >( pGraph->numOfVertices,  pGraph->numOfVertices, false);;
	for (int i = 0; i < pGraph->numOfVertices; i++)
		for (int j = 0; j < pGraph->numOfVertices; j++)
			originalE[i][j] = pGraph->E[i][j];

	#ifdef _LOG
		logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones na zaciatku : ",actualNumOfBones);
	#endif
	//collapse very close points

	//collapseCloseVertices(pGraph, groupingTolerance, groupingToleranceSDFMulti, &actualNumOfBones, wantedNumberOfBones, groupingWithoutEdge, sdfValues, useSDFBasedGroupingDistance);

	/*#ifdef _LOG
		logg.log(0, "actualNumOfBones po odstraneni blizkych : ",actualNumOfBones);
		logg.log(0, "Matica E po odstraneni blizkych");
		logg.log(0, pGraph->E);
	#endif*/

	//iterate and collapse until we got wanted number of bones
	/*while(actualNumOfBones > *wantedNumberOfBones) {
		#ifdef _LOG
			logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones : ",actualNumOfBones);
		#endif
		// for every edge
		int minIndexI = 0;
		int minIndexJ = 0;
		float minCost = FLT_MAX;
		for (int i = 0; i < pGraph->numOfVertices; i++)
			for (int j = i; j < pGraph->numOfVertices; j++)
				if (pGraph->E[i][j] && i != j) {
					// calculate shape cost + sampling cost
					//float shapeCT = wA * computeShapeCost(pGraph, i, j);
					//float samplingCT = wB * computeSamplingCost(pGraph, i, j);
					float cost = Distance(pGraph->pVerts[i], pGraph->pVerts[j]);
					// find minimum
					if (cost < minCost) {
						minCost = cost;
						minIndexI = i;
						minIndexJ = j;
					}
				}

	   halfEdgeCollapse(pGraph, minIndexI, minIndexJ, &actualNumOfBones, false);
	   #ifdef _LOG
			logg.log(LOG_LEVEL_C_PARAMS, "iterativny  halfEdgeCollapse : ",actualNumOfBones);
	   #endif
	}*/


	// lets create collapsed hash map from point clouds
	int segInd = 0;
	#ifdef _LOG
		logg.log(LOG_LEVEL_C_PARAMS, "pointClouds na konci");
	#endif

	/*for (int i = 0; i < pGraph->pointClouds.size(); i++){
		if ( pGraph->pointClouds[i].size() > 0){
			logg.log(2, "pointClouds pre vrchol : ",i);
			logg.log(2, "pointClouds velkost : ",(int)pGraph->pointClouds[i].size());
			for (int j = 0; j < pGraph->pointClouds[i].size(); j++){
				pGraph->collapsed[pGraph->pointClouds[i][j]] = i;
				logg.log(2, "clouditem : ",pGraph->pointClouds[i][j]);
			}
		}
	}*/

	*wantedNumberOfBones = actualNumOfBones;

	#ifdef _LOG
		logg.log(LOG_LEVEL_NOTE, "SKELETON TRIMED");
		logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones : ",actualNumOfBones);
	#endif

	/*if (doDisplacementShifting)
		for (int i = 0; i < pGraph->numOfVertices; i++){
			computeWADisplacement(i, pGraph, modelVertices , originalE, root, mgDegeneratesMapping);
		}*/

	//collapseCloseVertices(pGraph, groupingTolerance, groupingToleranceSDFMulti, &actualNumOfBones, wantedNumberOfBones, groupingWithoutEdge, sdfValues, useSDFBasedGroupingDistance);

	// find bigests component node
	int maxComponent = 0;
	int maxVolume = 0;

	for (int i = 0; i <actualNumOfBones; i++){
		float vol = computePointCloudVolume(pGraph->pointClouds[i], modelVertices, mgDegeneratesMapping);
		if (vol > maxVolume) {
			maxVolume = vol;
            maxComponent = i;
		}
	}
	pGraph->segmentIndex[segInd++] = maxComponent;

	root->point =  pGraph->pVerts[maxComponent];
	/*if (doDisplacementShifting) {
		CVector3 dis = computeWADisplacement(maxComponent, pGraph, modelVertices , originalE ,root,  mgDegeneratesMapping);
    }*/
	root->id = maxComponent + 1;
	//((skl::SkeletonNode *)root)->selected = false;

	// Create tree from graph structure

	vector<SN::SkeletonNode*> queue;
	SN::SkeletonNode imaginaryCenter;
	imaginaryCenter.point = CVector3(0,0,0);
	root->father = &imaginaryCenter;
	//((skl::SkeletonNode *)root)->bindPoseMatrices.qRotation = CVector4(1,0,0,0);//QuaternionBetweenVectors(CVector3(0,0,-1), root->point);
	root->cyclic = NULL;
	//Array2D<float>rotationMatrix = QuaternionToMatrix3x3(((skl::SkeletonNode*)root)->bindPoseMatrices.qRotation);
	//Array2D<float>invRotationMatrix = rotationMatrix.invert(rotationMatrix);
	//Array2D<float>trans = CVecToTntVec(root->point);
	//Array2D<float>invRotatedTrans =  trans * invRotationMatrix;
	//((skl::SkeletonNode *)root)->bindPoseMatrices.vTranslation = root->point;

	//((skl::SkeletonNode *)root)->bindPoseMatrices.computeAffineTransformation();
	queue.push_back(root);
	bool * added = new bool[pGraph->numOfVertices];
	for (int i = 0; i < pGraph->numOfVertices; i++)
    	added[i] = false;

	
	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica E pred vytvaranim stromu");
		logg.log(LOG_LEVEL_DUMP, pGraph->E);
	#endif

	vector<int*> cycleEnclosing;

	while(queue.size() > 0){
		SN::SkeletonNode * node = queue[queue.size() - 1];
		queue.pop_back();

		vector<SN::SkeletonNode*> nodes;

		for (int i = 0; i < pGraph->numOfVertices; i++){

			bool isInCycleEnclosing = false;
			int cycleEnclosingId1 = -1;
			int cycleEnclosingId2 = -1;
			for (int c = 0; c < cycleEnclosing.size(); c++){
				if ((cycleEnclosing[c][0] == i && cycleEnclosing[c][1] == node->id - 1) || (cycleEnclosing[c][1] == i && cycleEnclosing[c][0] == node->id - 1))
					isInCycleEnclosing = true;
					cycleEnclosingId1 = cycleEnclosing[c][0] + 1;
					cycleEnclosingId2 = cycleEnclosing[c][1] + 1;
			}

			if (added[i] && isInCycleEnclosing){
				// ((skl::SkeletonNode *)node)->cyclic = ((skl::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId1))
				SN::SkeletonNode * node1 = (SN::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId1); 
				if (node1 != NULL)
					node1->cyclic = findNodeWithIdInTree(root, cycleEnclosingId2);
				SN::SkeletonNode * node2 = (SN::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId2);
				if (node2 != NULL)
					node2->cyclic = findNodeWithIdInTree(root, cycleEnclosingId1);
			}

			if (pGraph->E[node->id - 1][i] && node->id - 1 != i) {
				#ifdef _LOG
					logg.log(LOG_LEVEL_DUMP, "matica E");
					logg.log(LOG_LEVEL_DUMP, pGraph->E);
				#endif
				pGraph->E[node->id - 1][i] = false;
				pGraph->E[i][node->id - 1] = false;

				if (!added[i] || cyclicSkeleton){
						// imaginary node 1
						SN::SkeletonNode *n1 = new SN::SkeletonNode();
						n1->point = pGraph->pVerts[i];
						// displacmenet
						/*if (doDisplacementShifting) {
							CVector3 dis = computeWADisplacement(i, pGraph, modelVertices , originalE, n1, mgDegeneratesMapping);
						}*/

						#ifdef _LOG
							logg.log(LOG_LEVEL_DUMP, "skeleton node isInCycleEnclosing: ", isInCycleEnclosing);
						#endif

						SN::SkeletonNode * node11 = (SN::SkeletonNode *)findNodeWithIdInTree(root, i + 1);
						n1->cyclic = node11;
						if (node11 != NULL){
							node11->cyclic = n1;
							int * pair = new int[2];
							pair[0] = i;
							pair[1] = node->id - 1;
							cycleEnclosing.push_back(pair);
							#ifdef _LOG
								logg.log(LOG_LEVEL_DUMP, "skeleton node cycle: ", n1->cyclic->id);
							#endif
						}

						n1->id = i + 1;
						pGraph->segmentIndex[segInd++] = i;
						//((skl::SkeletonNode *)n1)->selected = false;
						n1->father = node;
						//BonesMatrices current;

						/*current.qRotation = QuaternionBetweenVectors(node->father->point - node->point ,node->point - n->point );

						Array2D<float>rotationMatrix = QuaternionToMatrix3x3(current.qRotation);
						Array2D<float>invRotationMatrix = rotationMatrix.invert(rotationMatrix);
						Array2D<float>trans = CVecToTntVec(n->point - node->point);
						Array2D<float>invRotatedTrans =  trans * invRotationMatrix;
					
						current.vTranslation = TntVecToCVec(invRotatedTrans);*/

						//current.qRotation = CVector4(1,0,0,0);
						//current.vTranslation = n1->point - node->point;

						//current.computeAffineTransformation();
						//((skl::SkeletonNode *)n1)->bindPoseMatrices.currentAffine = ((skl::SkeletonNode *)node)->bindPoseMatrices.currentAffine * current.currentAffine;

						nodes.push_back(n1);

						queue.push_back(n1);
						added[i] = true;

						// imaginary node 2

               }
			}
		}
		node->nodes = nodes;
		//copyBonesMatrices( &(((skl::SkeletonNode *)node)->bindPoseMatrices), &(((skl::SkeletonNode *)node)->matrices));

		/*if (s_doBranchingSimplification){
			// simplify branching structure
			if (node->nodes.size() > 1) {
				// for every branching node check, if merged with adj. node has better centerdness
				// if yes, merge them (with minimum centerdness)
				int bestIndex = -1;
				float originalCenterdness = computeCenterdness(-1, node->id - 1, pGraph);
				float bestCenterdness = originalCenterdness;
				for (int i = 0; i < node->nodes.size(); i++) {
					float s = computeCenterdness(i, node->id - 1, pGraph);
					if (s < 0.9 * originalCenterdness) {
						if (s < bestCenterdness){
							bestIndex = i;
							bestCenterdness = s;
						}
					}
				}
				if (bestIndex != -1) {
					mergeAdjNodeWithBran(node, bestIndex);
				}
			}
		}*/
	}

	delete[] added;
	added = NULL;
}

//---------------------------------------------------------------------------
void createSkeletonFromSurgeryGraph(bool applyLBSEPostprocessing, SurgeryGraph * pGraph, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping ,SN::SkeletonNode * root, int * wantedNumberOfBones, float groupingTolerance, float groupingToleranceSDFMulti, bool doBranchingSimplification,bool doDisplacementShifting, bool groupingWithoutEdge, bool cyclicSkeleton, CVector3 * sdfValues, bool useSDFBasedGroupingDistance){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createSkeletonFromSurgeryGraph STARTED");
	#endif
	int actualNumOfBones = pGraph->numOfVertices;

	//logg.log(0, "Matica E PRED");
	//logg.log(0, pGraph->E);

	for (int i = 0; i < pGraph->numOfVertices; i++)
		pGraph->pointClouds[i].push_back(i);

	Array2D<bool> originalE = Array2D< bool >( pGraph->numOfVertices,  pGraph->numOfVertices, false);;
	for (int i = 0; i < pGraph->numOfVertices; i++)
		for (int j = 0; j < pGraph->numOfVertices; j++)
			originalE[i][j] = pGraph->E[i][j];

	if (applyLBSEPostprocessing){
		// compute K matrices
		if (actualNumOfBones > *wantedNumberOfBones) {
			Array2D<Array2D< float > >KMatrices = Array2D<Array2D< float > >( pGraph->numOfVertices,  pGraph->numOfVertices, Array2D< float >(3,  4, 0.0f));
			for (int i = 0; i < pGraph->numOfVertices; i++)
				for (int j = 0; j < pGraph->numOfVertices; j++)
					if (pGraph->E[i][j])
						KMatrices[i][j] = constructKMatrix(pGraph, i, j);
			for (int i = 0; i < pGraph->numOfVertices; i++){
				Array2D< float > Q(4, 4, 0.0f);
				//for (int m=0; m<4; m++)
				//for (int n=0; n<4; n++)
				//Q[m][n] = 0.0f;
				for (int j = 0; j < pGraph->numOfVertices; j++){
					if (pGraph->E[i][j]){
						Array2D< float > E(4, 4, 0.0f);
						E = KMatrices[i][j].transpose(KMatrices[i][j]) * KMatrices[i][j];
						Q = Q + E;
					}
				}
				pGraph->QMatrices.push_back(Q);
			}
		}
	}
#ifdef _LOG
	logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones na zaciatku : ",actualNumOfBones);
#endif
	//collapse very close points

	if (applyLBSEPostprocessing){
		collapseCloseVertices(pGraph, groupingTolerance, groupingToleranceSDFMulti, &actualNumOfBones, wantedNumberOfBones, groupingWithoutEdge, sdfValues, useSDFBasedGroupingDistance);
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones po odstraneni blizkych : ",actualNumOfBones);
		logg.log(LOG_LEVEL_DUMP, "Matica E po odstraneni blizkych");
		logg.log(LOG_LEVEL_DUMP, pGraph->E);
	#endif

	if (applyLBSEPostprocessing){
		//iterate and collapse until we got wanted number of bones
		while(actualNumOfBones > *wantedNumberOfBones) {
			#ifdef _LOG
				logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones : ",actualNumOfBones);
			#endif
			// for every edge
			int minIndexI = 0;
			int minIndexJ = 0;
			float minCost = FLT_MAX;
			for (int i = 0; i < pGraph->numOfVertices; i++)
				for (int j = i; j < pGraph->numOfVertices; j++)
					if (pGraph->E[i][j] && i != j) {
						// calculate shape cost + sampling cost
						float shapeCT = wA * computeShapeCost(pGraph, i, j);
						float samplingCT = wB * computeSamplingCost(pGraph, i, j);
						float cost = shapeCT + samplingCT;
						// find minimum
						if (cost < minCost) {
							minCost = cost;
							minIndexI = i;
							minIndexJ = j;
						}
					}

		   halfEdgeCollapse(pGraph, minIndexI, minIndexJ, &actualNumOfBones, true);
		   #ifdef _LOG
				logg.log(LOG_LEVEL_C_PARAMS, "iterativny  halfEdgeCollapse : ",actualNumOfBones);
		   #endif
		}
	}


	// lets create collapsed hash map from point clouds
	int segInd = 0;
	#ifdef _LOG
		logg.log(LOG_LEVEL_C_PARAMS, "pointClouds na konci");
	#endif

	/*for (int i = 0; i < pGraph->pointClouds.size(); i++){
		if ( pGraph->pointClouds[i].size() > 0){
			logg.log(2, "pointClouds pre vrchol : ",i);
			logg.log(2, "pointClouds velkost : ",(int)pGraph->pointClouds[i].size());
			for (int j = 0; j < pGraph->pointClouds[i].size(); j++){
				pGraph->collapsed[pGraph->pointClouds[i][j]] = i;
				logg.log(2, "clouditem : ",pGraph->pointClouds[i][j]);
			}
		}
	}*/

	*wantedNumberOfBones = actualNumOfBones;

	#ifdef _LOG
		logg.log(LOG_LEVEL_NOTE, "SKELETON TRIMED");
		logg.log(LOG_LEVEL_C_PARAMS, "actualNumOfBones : ",actualNumOfBones);
	#endif

	if (doDisplacementShifting && applyLBSEPostprocessing)
		for (int i = 0; i < pGraph->numOfVertices; i++){
			computeWADisplacement(i, pGraph, modelVertices , originalE, root, mgDegeneratesMapping);
		}

	if (applyLBSEPostprocessing){
		collapseCloseVertices(pGraph, groupingTolerance, groupingToleranceSDFMulti, &actualNumOfBones, wantedNumberOfBones, groupingWithoutEdge, sdfValues, useSDFBasedGroupingDistance);
	}

	// find bigests component node
	int maxComponent = 0;
	int maxVolume = 0;

	for (int i = 0; i <actualNumOfBones; i++){
		float vol = computePointCloudVolume(pGraph->pointClouds[i], modelVertices, mgDegeneratesMapping);
		if (vol > maxVolume) {
			maxVolume = vol;
            maxComponent = i;
		}
	}
	pGraph->segmentIndex[segInd++] = maxComponent;

	root->point =  pGraph->pVerts[maxComponent];
	if (doDisplacementShifting && applyLBSEPostprocessing) {
		CVector3 dis = computeWADisplacement(maxComponent, pGraph, modelVertices , originalE ,root,  mgDegeneratesMapping);
    }
	root->id = maxComponent + 1;
	//((skl::SkeletonNode *)root)->selected = false;

	// Create tree from graph structure

	vector<SN::SkeletonNode*> queue;
	SN::SkeletonNode imaginaryCenter;
	imaginaryCenter.point = CVector3(0,0,0);
	root->father = &imaginaryCenter;
	//((skl::SkeletonNode *)root)->bindPoseMatrices.qRotation = CVector4(1,0,0,0);//QuaternionBetweenVectors(CVector3(0,0,-1), root->point);
	root->cyclic = NULL;
	//Array2D<float>rotationMatrix = QuaternionToMatrix3x3(((skl::SkeletonNode*)root)->bindPoseMatrices.qRotation);
	//Array2D<float>invRotationMatrix = rotationMatrix.invert(rotationMatrix);
	//Array2D<float>trans = CVecToTntVec(root->point);
	//Array2D<float>invRotatedTrans =  trans * invRotationMatrix;
	//((skl::SkeletonNode *)root)->bindPoseMatrices.vTranslation = root->point;

	//((skl::SkeletonNode *)root)->bindPoseMatrices.computeAffineTransformation();
	queue.push_back(root);
	bool * added = new bool[pGraph->numOfVertices];
	for (int i = 0; i < pGraph->numOfVertices; i++)
    	added[i] = false;

	
	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica E pred vytvaranim stromu");
		logg.log(LOG_LEVEL_DUMP, pGraph->E);
	#endif

	vector<int*> cycleEnclosing;

	while(queue.size() > 0){
		SN::SkeletonNode * node = queue[queue.size() - 1];
		queue.pop_back();

		vector<SN::SkeletonNode*> nodes;

		for (int i = 0; i < pGraph->numOfVertices; i++){

			bool isInCycleEnclosing = false;
			int cycleEnclosingId1 = -1;
			int cycleEnclosingId2 = -1;
			for (int c = 0; c < cycleEnclosing.size(); c++){
				if ((cycleEnclosing[c][0] == i && cycleEnclosing[c][1] == node->id - 1) || (cycleEnclosing[c][1] == i && cycleEnclosing[c][0] == node->id - 1))
					isInCycleEnclosing = true;
					cycleEnclosingId1 = cycleEnclosing[c][0] + 1;
					cycleEnclosingId2 = cycleEnclosing[c][1] + 1;
			}

			if (added[i] && isInCycleEnclosing){
				// ((skl::SkeletonNode *)node)->cyclic = ((skl::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId1))
				SN::SkeletonNode * node1 = (SN::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId1); 
				if (node1 != NULL)
					node1->cyclic = findNodeWithIdInTree(root, cycleEnclosingId2);
				SN::SkeletonNode * node2 = (SN::SkeletonNode *)findNodeWithIdInTree(root, cycleEnclosingId2);
				if (node2 != NULL)
					node2->cyclic = findNodeWithIdInTree(root, cycleEnclosingId1);
			}

			if (pGraph->E[node->id - 1][i] && node->id - 1 != i) {
				#ifdef _LOG
					logg.log(LOG_LEVEL_DUMP, "matica E");
					logg.log(LOG_LEVEL_DUMP, pGraph->E);
				#endif
				pGraph->E[node->id - 1][i] = false;
				pGraph->E[i][node->id - 1] = false;

				if (!added[i] || cyclicSkeleton){
						// imaginary node 1
						SN::SkeletonNode *n1 = new SN::SkeletonNode();
						n1->point = pGraph->pVerts[i];
						// displacmenet
						if (doDisplacementShifting && applyLBSEPostprocessing) {
							CVector3 dis = computeWADisplacement(i, pGraph, modelVertices , originalE, n1, mgDegeneratesMapping);
						}

						#ifdef _LOG
							logg.log(LOG_LEVEL_DUMP, "skeleton node isInCycleEnclosing: ", isInCycleEnclosing);
						#endif

						SN::SkeletonNode * node11 = (SN::SkeletonNode *)findNodeWithIdInTree(root, i + 1);
						n1->cyclic = node11;
						if (node11 != NULL){
							node11->cyclic = n1;
							int * pair = new int[2];
							pair[0] = i;
							pair[1] = node->id - 1;
							cycleEnclosing.push_back(pair);
							#ifdef _LOG
								logg.log(LOG_LEVEL_DUMP, "skeleton node cycle: ", n1->cyclic->id);
							#endif
						}

						n1->id = i + 1;
						pGraph->segmentIndex[segInd++] = i;
						//((skl::SkeletonNode *)n1)->selected = false;
						n1->father = node;
						//BonesMatrices current;

						/*current.qRotation = QuaternionBetweenVectors(node->father->point - node->point ,node->point - n->point );

						Array2D<float>rotationMatrix = QuaternionToMatrix3x3(current.qRotation);
						Array2D<float>invRotationMatrix = rotationMatrix.invert(rotationMatrix);
						Array2D<float>trans = CVecToTntVec(n->point - node->point);
						Array2D<float>invRotatedTrans =  trans * invRotationMatrix;
					
						current.vTranslation = TntVecToCVec(invRotatedTrans);*/

						//current.qRotation = CVector4(1,0,0,0);
						//current.vTranslation = n1->point - node->point;

						//current.computeAffineTransformation();
						//((skl::SkeletonNode *)n1)->bindPoseMatrices.currentAffine = ((skl::SkeletonNode *)node)->bindPoseMatrices.currentAffine * current.currentAffine;
						nodes.push_back(n1);

						queue.push_back(n1);
						added[i] = true;

						// imaginary node 2

               }
			}
		}
		node->nodes = nodes;
		//copyBonesMatrices( &(((skl::SkeletonNode *)node)->bindPoseMatrices), &(((skl::SkeletonNode *)node)->matrices));

		if (s_doBranchingSimplification && applyLBSEPostprocessing){
			// simplify branching structure
			if (node->nodes.size() > 1) {
				// for every branching node check, if merged with adj. node has better centerdness
				// if yes, merge them (with minimum centerdness)
				int bestIndex = -1;
				float originalCenterdness = computeCenterdness(-1, node->id - 1, pGraph);
				float bestCenterdness = originalCenterdness;
				for (int i = 0; i < node->nodes.size(); i++) {
					float s = computeCenterdness(i, node->id - 1, pGraph);
					if (s < 0.9 * originalCenterdness) {
						if (s < bestCenterdness){
							bestIndex = i;
							bestCenterdness = s;
						}
					}
				}
				if (bestIndex != -1) {
					mergeAdjNodeWithBran(node, bestIndex);
				}
			}
		}
	}

	delete[] added;
	added = NULL;
}

//---------------------------------------------------------------------------
float computeCenterdness(int adj, int bran, SurgeryGraph * pGraph){
	int length = pGraph->pointClouds[bran].size();
	if (adj != -1)
    	length++;
	float * distances = new float[length];
	float sum = 0.0f;
	for (int i = 0; i < pGraph->pointClouds[bran].size(); i++) {
		int index = pGraph->pointClouds[bran][i];
		distances[i] = Distance(pGraph->pVerts[bran], pGraph->pVerts[index]);
		sum += distances[i];
	}
	if (adj != -1){
		distances[length - 1] = Distance(pGraph->pVerts[bran], pGraph->pVerts[adj]);
		sum += distances[length - 1];
	}
	float mean = sum / (float)length;
	float nom = 0.0f;
	for (int i = 0; i < length; i++) {
		nom += pow(distances[i] - mean,2);
	}

	delete[] distances;
	distances = NULL;

	return sqrt(nom / (float)length);
}
//---------------------------------------------------------------------------
// merge branching node with his best neighbour which minimize centerdness in skeleton tree
void mergeAdjNodeWithBran(SN::SkeletonNode * pNode, int adjIndex){
	for (int i = 0; i < pNode->nodes[adjIndex]->nodes.size(); i++)
		pNode->nodes.push_back(pNode->nodes[adjIndex]->nodes[i]);
	pNode->nodes.erase(pNode->nodes.begin() + adjIndex);
}
//---------------------------------------------------------------------------
CVector3 computeWADisplacement(int j, SurgeryGraph * pGraph, CVector3 * modelVertices, Array2D<bool> originalE, SN::SkeletonNode * node, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
		CVector3 dis = computeWADisplacement2(j, pGraph, modelVertices, originalE, mgDegeneratesMapping);
		bool relative = false;
		#ifdef _LOG
			logg.log(LOG_LEVEL_DUMP, "dispalcement for vertex : ", j);
			logg.log(LOG_LEVEL_DUMP, "dispalcement vector: ", dis);
		#endif

		if (relative){
			pGraph->pVerts[j] = pGraph->pVerts[j] - dis;
			node->point = node->point - dis;
		} else {
			pGraph->pVerts[j] = dis;
			node->point = dis;
		}
	return dis;
}


/*CVector3 computeWADisplacement0(int j, SurgeryGraph * pGraph, CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
	logg.log(LOG_LEVEL_C_PARAMS, "computeWADisplacement STARTED for vertex : ", j);
	CVector3 d = CVector3(0,0,0);
	float weight = 0.0f;

	float * l = new float[pGraph->pointClouds[j].size()];
	int segmentLength = pGraph->pointClouds[j].size();

	for (int i = 0; i < segmentLength; i++) {
		int index = mgDegeneratesMapping[pGraph->pointClouds[j][i]];
		float length = 0.0f;
		for (int k = 0; k < segmentLength; k++)
			if (originalE[i][k])
				length += Distance(modelVertices[pGraph->pointClouds[j][i]], modelVertices[pGraph->pointClouds[j][k]]);
        l[i] = length;
	}

	for (int i = 0; i < segmentLength; i++) {
		int index = mgDegeneratesMapping[pGraph->pointClouds[j][i]];
		CVector3 vec = pGraph->pVerts[j] - modelVertices[index];
		d = d + vec * l[i];
		weight += l[i];
	}

	delete[] l;
	l = NULL;
  	if (weight != 0)
		return d / weight;
	else
		return CVector3(0,0,0);
}*/

/*CVector3 computeWADisplacement1(int j, SurgeryGraph * pGraph, CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
	logg.log(LOG_LEVEL_C_PARAMS, "computeWADisplacement STARTED for vertex : ", j);
	CVector3 d = CVector3(0,0,0);
	float weight = 0.0f;


	float * l = new float[pGraph->pointClouds[j].size()];
	int segmentLength = pGraph->pointClouds[j].size();

	for (int i = 0; i < segmentLength; i++) {
		int index = mgDegeneratesMapping[pGraph->pointClouds[j][i]];
		float length = 0.0f;
		for (int k = 0; k < segmentLength; k++)
			if (originalE[index][pGraph->pointClouds[j][k]])
				length += Distance(modelVertices[index], modelVertices[pGraph->pointClouds[j][k]]);
        l[i] = length;
	}

	for (int i = 0; i < segmentLength; i++) {
		int index = mgDegeneratesMapping[pGraph->pointClouds[j][i]];
		CVector3 vec = pGraph->pVerts[index] - modelVertices[index];
		d = d + vec * l[i];
		weight += l[i];
	}

	delete[] l;
	l = NULL;
  	if (weight != 0)
		return d / weight;
	else
		return CVector3(0,0,0);
}*/

CVector3 computeWADisplacement2(int j, SurgeryGraph * pGraph, CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
	#ifdef _LOG
		logg.log(LOG_LEVEL_C_PARAMS, "computeWADisplacement2 STARTED for vertex : ", j);
	#endif

	CVector3 d = CVector3(0,0,0);

	int segmentLength = pGraph->pointClouds[j].size();

	int num = 0;
	for (int i = 0; i < segmentLength; i++) {
		for (int l = 0; l < getDegenerateIndex(mgDegeneratesMapping, pGraph->pointClouds[j][i]).size(); l++) {
			int index = getDegenerateIndex(mgDegeneratesMapping,pGraph->pointClouds[j][i])[l];
			d = d + modelVertices[index];
			num++;
		}
	}

  	if (num != 0)
		return d / num;
	else
		return CVector3(0,0,0);
}

/*CVector3 computeWADisplacement3(int j, SurgeryGraph * pGraph, CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping){
	logg.log(LOG_LEVEL_C_PARAMS, "computeWADisplacement STARTED for vertex : ", j);
	CVector3 d = CVector3(0,0,0);
	float weight = 0.0f;


	float * l = new float[pGraph->pointClouds[j].size()];
	int segmentLength = pGraph->pointClouds[j].size();

	for (int i = 0; i < segmentLength; i++) {
		int index = mgDegeneratesMapping[pGraph->pointClouds[j][i]];
		float length = 0.0f;
		for (int k = i; k < segmentLength; k++)
			if (originalE[index][pGraph->pointClouds[j][k]]){
				l[i] = Distance(modelVertices[index], modelVertices[pGraph->pointClouds[j][k]]);
				weight += l[i];
				d = d + ((modelVertices[index] + modelVertices[pGraph->pointClouds[j][k]]) / 2) * l[i];
				break;
			}
	}

	delete[] l;
	l = NULL;
  	if (weight != 0)
		return d / weight;
	else
		return CVector3(0,0,0);
}*/
//---------------------------------------------------------------------------
// Shape Cost functions
float computeShapeCost(SurgeryGraph * pGraph, int i, int j){
	return computeErrorValue(pGraph, i, j) + computeErrorValue(pGraph, j, j);
}
//---------------------------------------------------------------------------
float computeErrorValue(SurgeryGraph * pGraph, int i, int j){
	CVector3 p = pGraph->pVerts[j];
	Array2D< float > P(4, 1, 0.0f);
	P[0][0] = p.x;
	P[1][0] = p.y;
	P[2][0] = p.z;
	P[3][0] = 1.0f;
	Array2D< float > Q = pGraph->QMatrices[i];
	//logg.log(0, "Matica Q");
	//logg.log(0, Q);
	Array2D< float > Res(1, 1, 0.0f);
	Array2D< float > PT(1, 3, 0.0f);
	PT = P.transpose(P);
	Res = PT * Q * P;
	return Res[0][0];
}
//---------------------------------------------------------------------------
Array2D< float > constructKMatrix(SurgeryGraph * pGraph, int i, int j){
	//normalized edge vector a
	CVector3 a = Normalize(pGraph->pVerts[j] - pGraph->pVerts[i]);
	CVector3 b = Cross(a, pGraph->pVerts[i]);
	Array2D< float > matrix(3, 4, 0.0f);
	matrix[1][0] = a.z;
	matrix[2][0] = -a.y;

	matrix[0][1] = -a.z;
	matrix[2][1] = a.x;

	matrix[0][2] = a.y;
	matrix[1][2] = -a.x;

	matrix[0][3] = -b.x;
	matrix[1][3] = -b.y;
	matrix[2][3] = -b.z;

	return matrix;
}
//---------------------------------------------------------------------------
// Sampling Cost functions
float computeSamplingCost(SurgeryGraph * pGraph, int i, int j){
	float sum = 0.0f;
	for (int k = 0; k < pGraph->numOfVertices; k++)
		if (pGraph->E[i][k])
			sum += Distance(pGraph->pVerts[i], pGraph->pVerts[k]);
	return Distance(pGraph->pVerts[i], pGraph->pVerts[j]) + sum;
}





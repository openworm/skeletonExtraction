//---------------------------------------------------------------------------
#include "gdsw_graphAlgorithms.h"


//---------------------------------------------------------------------------
//
//  Dijkstra algorithm to compute the shortest path on the mesh
//  from every vertex to all other vertices
//
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
void GraphAlgorithms::computeDijkstraDistance(MeshGraph * pMesh, Array2D<float>& M)
{

  /*M Array2D<float>(pMesh->numOfVertices, pMesh->numOfVertices, FLT_MAX)

  father = new int[pModel->numOfVertices];
  visit = new bool[pModel->numOfVertices];

  priority_queue<pair<float,int> > queue;
  pair <float,int> nodotmp;
  int i, j;

  for (int i=0; i<pModel->numOfVertices; i++) {
    father[i] = -1;
    visit[i] = false;
  }

  pModel->pDistVert[start] = 0.0;
  queue.push(pair <float,int> (pModel->pDistVert[start], start));

  while(!queue.empty()) {
    nodotmp = queue.top();
	queue.pop();
	i = nodotmp.second;
	if (!visit[i]) {
	  visit[i] = true;
	  vector<int> neighbours = pModel->neighbours[i];
	  for (int k = 0; k<neighbours.size(); k++){
		int j = neighbours[k];
		if (!visit[j]){
			CVector3 p1 = getVertexWithID(pModel, i);
			CVector3 p2 = getVertexWithID(pModel, j);
			float dist = pModel->pDistVert[i] + Distance(p1,p2);
			if (dist < pModel->pDistVert[j]) {
			  pModel->pDistVert[j] =  dist;
			  father[j] = i;
			  queue.push(pair <float,int>(-pModel->pDistVert[j], j));
			}
		  }
		}
	  }
  }

   delete[] father;
   delete[] visit;*/
}

void GraphAlgorithms::FloydWarshall(MeshGraph * pMesh, Array2D<float>& M){
	#ifdef _LOG
		log (LOG_LEVEL_ALGORITHMS, "START - FloydWarshall");
	#endif

	M = Array2D<float>(pMesh->numOfVertices, pMesh->numOfVertices, FLT_MAX);
	for (int i=0; i<pMesh->numOfVertices; i++)
		for (int j=0; j<pMesh->numOfVertices; j++)
			if (pMesh->E[i][j])
				M[i][j] = Distance(pMesh->pVerts[i], pMesh->pVerts[j]);

for (int k=0; k<pMesh->numOfVertices; k++)
	for (int i=0; i<pMesh->numOfVertices; i++)
		for (int j=0; j<pMesh->numOfVertices; j++)
				M[i][j] = min ( M[i][j], M[i][k]+M[k][j] );

	#ifdef _LOG
		log (LOG_LEVEL_ALGORITHMS, "END - FloydWarshall");
	#endif
}

//---------------------------------------------------------------------------
// returns the shortest way between 2 points on the mesh graph structure
float GraphAlgorithms::GeodesicDistance(MeshGraph * pMesh,  Array2D<float> distanceMatrix, CVector3 p1, int vertexIndex, bool firstSkeletonPoint){
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
}

int  GraphAlgorithms::findClosestPointIndex(MeshGraph *pMesh, CVector3 sourcePoint){
	float minDist = FLT_MAX;
	int index = -1;
	for (int i = 0; i < pMesh->numOfVertices; i++){
	   CVector3 p = pMesh->pVerts[i];
	   float dist = Distance(p, sourcePoint);
	   if (dist < minDist){
		   minDist = dist;
		   index = i;
	   }
	}
	return index;
}

//---------------------------------------------------------------------------
// method finds x closest bones from skeleton subtree to the point
void GraphAlgorithms::findXMinInTree(MeshGraph * pMesh, Array2D<float> distanceMatrix, int x, SN::SkeletonNode *node, SN::SkeletonNode * mins, int vertexIndex){

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
}

void GraphAlgorithms::computeObjectSkeletonShaderData(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, structure::t3DObject *pObject, ObjectSkeletonShaderData * data, int length, int indexOffset, bool isMeshGraphComputed, Array2D<float> distanceMatrix, int numOfControlBones, int maxBones){
	if (isMeshGraphComputed)
		pObject->skinning = true;

	// if distance matrix is not computed
	if (distanceMatrix.dim1() == 0 && distanceMatrix.dim2() == 0)
		return;

#ifdef _LOG
	logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeObjectSkeletonShaderData STARTED");
#endif

	data->numOfVertices = length;

	float* weights = new float[pObject->numOfVerts * numOfControlBones];
	float* indices = new float[pObject->numOfVerts * numOfControlBones];

	SN::SkeletonNode * base = new SN::SkeletonNode[numOfControlBones];
	float * dist = new float[numOfControlBones];

	vector<SN::SkeletonNode*> queue;
	queue.push_back((SN::SkeletonNode*)currentSkeletonRoot);


	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		//pNode->point = pNode->bindPoseMatrices.getTranslationFromAffine();

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}

	for (int i = 0; i < pObject->numOfVerts; i++) {
		//UNCOMMENT if (isMeshGraphComputed && pLBSExtractor.pMesh->indices[indexOffset + i] == -1)
			//pObject->skinning = false;	

		// find x control bones which have influence on vertex
		findXMinInTree(pMesh, distanceMatrix, numOfControlBones, currentSkeletonRoot, base, pMesh->indices[indexOffset + i]);

		// if there is less than NUM_OF_CTRL_BONES(4) control points, multiply the best one
		vector<int> empty;
		int bestIndex = -1;
		float minDis = FLT_MAX;
		for (int k = 0; k < numOfControlBones; k++) {
			if (base[k].id < 1)
				empty.push_back(k);
			else {
				float f = GeodesicDistance(pMesh, distanceMatrix, base[k].point, pMesh->indices[indexOffset + i], true);
				if (f < minDis){
					minDis = f;
					bestIndex = k;
				}
			}
		}
		if (bestIndex > -1)
			for (int k=0; k<empty.size(); k++)
				base[empty[k]] = base[bestIndex];

		float distSum = 0.0f;
		for (int k = 0; k < numOfControlBones; k++) {
			indices[i * numOfControlBones + k] = (float)base[k].id - 1;
			float f = Distance(pObject->pVerts[i], base[k].point);
			// toto este nemozem pouzit, lebo ratanie vzdialenosti mi funguje iba jednosmerne top-bottom stromovo, nie oboma smermi ak v grafe
			// je tam problem ked najde najblizsie body k raidiacemu vrcholu a bodu meshu ten isty vrchol grafu
			//float f = GeodesicDistance(pLBSExtractor.pMesh, distanceMatrix, base[k].point, pLBSExtractor.pMesh->indices[indexOffset + i], true);
			if (f == 0)
				dist[k] = FLT_MAX;
			else
				dist[k] = 1.0 / f;
			distSum += dist[k];

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "indexy pre vertex : " ,i);
				logg.log(LOG_LEVEL_DUMP, "index", indices[i * config.NUM_OF_CTRL_BONES + k]);
			#endif
		}

		if (distSum > FLT_MAX)
			distSum = FLT_MAX;

		for (int k = 0; k < numOfControlBones; k++) {
			float invDist;
			if (distSum != 0)
				weights[i*numOfControlBones + k] = (dist[k] / distSum);
			else
				weights[i*numOfControlBones + k] = 1.0f;

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "vahy pre vertex : " , i);
				logg.log(LOG_LEVEL_DUMP, "vaha ",weights[i * config.NUM_OF_CTRL_BONES + k]);
			#endif
		}
	}

	queue.clear();
	queue.push_back((SN::SkeletonNode*)currentSkeletonRoot);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = (SN::SkeletonNode*)queue[queue.size() - 1];
		queue.pop_back();

		/*if (useBindPoseMatrices){
			pNode->point = pNode->bindPoseMatrices.getTranslationFromAffine();
		} else {
			pNode->point = pNode->matrices.getTranslationFromAffine();
		}

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}*/
	}


	if (length != pObject->numOfVerts) {

		int i = 0;
		for(int j = 0; j < pObject->numOfFaces; j++){
			for(int whichVertex = 0; whichVertex < 3; whichVertex++){
				int index = pObject->pFaces[j].vertIndex[whichVertex];
				for (int k = 0; k < numOfControlBones; k++) {
					data->indices[i * numOfControlBones + k] = indices[index * numOfControlBones + k];
					data->weights[i * numOfControlBones + k] = weights[index * numOfControlBones + k];
				}
				i++;
			}
		}
	} else {
		memcpy(data->indices, indices, length * numOfControlBones * sizeof(float));
		memcpy(data->weights, weights, length * numOfControlBones * sizeof(float));
	}


	//delete[] base;
	//base = NULL;
	delete[] dist;
	dist = NULL;
	delete[] indices;
	indices = NULL;
	delete[] weights;
	weights = NULL;

	data->numOfBones = maxBones;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeObjectSkeletonShaderData ENDED");
	#endif
}


#pragma package(smart_init)


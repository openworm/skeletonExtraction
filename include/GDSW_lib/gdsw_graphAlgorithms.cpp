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
		logg.log(0, "START - FloydWarshall");
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
		logg.log(0, "END - FloydWarshall");
	#endif
}

void GraphAlgorithms::FloydWarshallLocaly(SN::SkeletonNode * pSkelRoot, boost::unordered_map<int, std::vector<int> > &closestToSkeletonNode, boost::unordered_map<int, std::vector<int> > &closestSegmentsVertexIndices, MeshGraph * pMesh, Array2D<float>& M){
	#ifdef _LOG
		logg.log(0, "START - locally FloydWarshall");
	#endif

	M = Array2D<float>(pMesh->numOfVertices, pMesh->numOfVertices, FLT_MAX);

	// now for each segment of the skeleton

	vector<SN::SkeletonNode*> queue;
	queue.push_back(pSkelRoot);

	while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();

		// get all close segments and its vertices 

		std::vector<int> closestVertexIndices = closestSegmentsVertexIndices[n->id - 1];

		// and compute local FW in that set

		int localSize = closestVertexIndices.size();

		Array2D<float> localM = Array2D<float>(localSize, localSize, FLT_MAX);

		for (int i=0; i<localSize; i++)
			for (int j=0; j<localSize; j++)
				if (pMesh->E[closestVertexIndices[i]][closestVertexIndices[j]])
					localM[i][j] = Distance(pMesh->pVerts[closestVertexIndices[i]], pMesh->pVerts[closestVertexIndices[j]]);

		for (int k=0; k<localSize; k++)
			for (int i=0; i<localSize; i++)
				for (int j=0; j<localSize; j++)
					localM[i][j] = min ( localM[i][j], localM[i][k] + localM[k][j] );


		// copy local distances into global matrix

		for (int i=0; i<localSize; i++)
			for (int j=0; j<localSize; j++)
				if (localM[i][j] < M[closestVertexIndices[i]][closestVertexIndices[j]])
					M[closestVertexIndices[i]][closestVertexIndices[j]] =  localM[i][j];

		/*#ifdef _LOG
		logg.log(0, "local M for segment");
		logg.log(0, localM);
		#endif*/

		// go for next segments

		for (int i=0; i < n->nodes.size(); i++){
			queue.push_back((SN::SkeletonNode*)n->nodes[i]);
		}
	}

	/*#ifdef _LOG
		logg.log(0, "global M");
		logg.log(0, M);
	#endif*/

	#ifdef _LOG
		logg.log(0, "END - locally FloydWarshall");
	#endif
}


void GraphAlgorithms::computeClosestSegmentVertices(MeshGraph * pMesh, SN::SkeletonNode * pSkelRoot, boost::unordered_map<int, std::vector<int> > &closestToSkeletonNode,  boost::unordered_map<int, vector<int> > &closestSegmentsVertexIndices){

	// iterate trough all skeleton segments and find closests segments
	vector<SN::SkeletonNode*> queue;
	queue.push_back(pSkelRoot);

	while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();

		// closest for node *n
		vector<SN::SkeletonNode*> closestSegments;

		if (n->father != NULL)
			closestSegments.push_back(n->father);

		closestSegments.push_back(n);

		for (int i=0; i < n->nodes.size(); i++){
			closestSegments.push_back(n->nodes[i]);
		}

		vector<int> closestsIndices;

		for (int i=0; i < closestSegments.size(); i++){
			SN::SkeletonNode* seg = closestSegments[i];

			int test2 = closestToSkeletonNode[seg->id - 1].size();

			closestsIndices.insert(closestsIndices.end(),  closestToSkeletonNode[seg->id - 1].begin(),  closestToSkeletonNode[seg->id - 1].end());
		}

		closestSegmentsVertexIndices[n->id - 1] = closestsIndices;

		int test = closestSegmentsVertexIndices[n->id - 1].size();

		// go for next segments

		for (int i=0; i < n->nodes.size(); i++){
			queue.push_back((SN::SkeletonNode*)n->nodes[i]);
		}
	}


}

//---------------------------------------------------------------------------
// returns the shortest way between 2 points on the mesh graph structure
float GraphAlgorithms::GeodesicDistance(MeshGraph * pMesh,  Array2D<float> distanceMatrix, CVector3 p1, int vertexIndex, bool firstSkeletonPoint, vector<int> closestVertexOneRing){
	float minDistance = FLT_MAX;
	for (int i=0; i < closestVertexOneRing.size(); i++){
		int idx1 = closestVertexOneRing[i];//findClosestPointIndex(pMesh, p1); // find whole one ring vertices, than distance is minumum distance -> predpocitat najblzisich
		int idx2 = vertexIndex;
		if (idx1 == -1 || idx2 == -1){
			continue;
		}
		if (idx1 == idx2){
			return Distance(p1, pMesh->pVerts[idx2]);
		}
		//if (firstSkeletonPoint)
		//	return distanceMatrix[idx1][idx2] + Distance(p1, pMesh->pVerts[idx1]);
		float d = distanceMatrix[idx1][idx2] + Distance(p1, pMesh->pVerts[idx1]);
		if (d < minDistance){
			minDistance = d;
		}
	}
	return minDistance;
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
void GraphAlgorithms::findXMinInTree(MeshGraph * pMesh, Array2D<float> distanceMatrix, int x, SN::SkeletonNode *pRoot, SN::SkeletonNode * mins, int vertexIndex, boost::unordered_map<int, vector<int> > &closestVertexOneRing){

	//if (node->father == NULL){
		pRoot->father = new SN::SkeletonNode();
		pRoot->father->point = CVector3(0,0,0);
		pRoot->father->id = -1;
	//}

	SN::SkeletonNode maxNode;
	maxNode.id = -1;
	maxNode.point = CVector3(FLT_MAX, FLT_MAX, FLT_MAX);
	maxNode.father = new SN::SkeletonNode();
	maxNode.father->id = -1;
	maxNode.father->point = CVector3(FLT_MAX, FLT_MAX, FLT_MAX);

	for (int i = 0; i < x ; i++){
		mins[i] = maxNode;
	}


	/*while(queue.size() > 0){
		SN::SkeletonNode * n = queue[queue.size() - 1];
		queue.pop_back();
		for (int i = 0; i < n->nodes.size(); i++)
			queue.push_back(n->nodes[i]);

		// find max distanced node in mins array
		int maxIndx = 0;
		float maxDist = 0.0;
		for (int j = 0; j < x; j++) {
			float dist = GeodesicDistance(pMesh, distanceMatrix, (mins[j].point + mins[j].father->point) / 2.0, vertexIndex, true);
			if (dist > maxDist) {
				maxIndx = j;
				maxDist = dist;
			}
		}
		//and replace him
		float baseDist = GeodesicDistance(pMesh, distanceMatrix, (n->point + n->father->point) / 2.0, vertexIndex, true);
		if (baseDist < maxDist) {
			mins[maxIndx] = *n;
		}
	}*/

	for (int j = 0; j < x; j++) {
		vector<SN::SkeletonNode*> queue;
		queue.push_back(pRoot);

		float minDist = FLT_MAX;
		SN::SkeletonNode * minNode = pRoot; // if there is no closer node, make root as default (the closests, there is no rotation for root)
		while(queue.size() > 0){
			SN::SkeletonNode * n = queue[queue.size() - 1];
			queue.pop_back();
			for (int i = 0; i < n->nodes.size(); i++){
				queue.push_back(n->nodes[i]);
			}

			float dist = GeodesicDistance(pMesh, distanceMatrix, (n->point + n->father->point) / 2.0, vertexIndex, true, closestVertexOneRing[n->id - 1]); //

			bool isNotInMins = true;

			for (int m = 0; m < x; m++) {
				if (mins[m].id == n->id){
					isNotInMins = false;
				}
			}

			if (dist < minDist && isNotInMins && n->id != pRoot->id) { // is not already added and is not root
				minNode = n;
				minDist = dist;
			}

		}

		mins[j] = *minNode;
	}

	#ifdef _LOG
		logg.log(0, "closest segments for vertex:", vertexIndex);
		logg.log(0, "1", mins[0].id);
		logg.log(0, "2", mins[1].id);
	#endif

}

void GraphAlgorithms::calculateClosestOneRingIndices(SN::SkeletonNode* sklRoot, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &closestVertexOneRing){
	vector<SN::SkeletonNode*> queue;
	queue.push_back(sklRoot->nodes[0]);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = (SN::SkeletonNode*)queue[queue.size() - 1];
		queue.pop_back();

		CVector3 point = (pNode->point + pNode->father->point) / 2.0;

		vector<float> distances;
		vector<int> indices;

		// calculate distances

		for (int i=0; i < pMesh->numOfVertices; i++){
			float d = Magnitude(pMesh->pVerts[i] - point);
			distances.push_back(d);
			indices.push_back(i);
		}

		for (int j=0; j < pMesh->numOfVertices; j++){

			int minIndex = j;
			float swapDistance = 0;
			int swapIndex = 0;

			for (int i=j; i < pMesh->numOfVertices; i++){
				if (distances[i] < distances[minIndex]){
					minIndex = i;
				}
			}

			swapIndex = indices[j];
			indices[j] = indices[minIndex];
			indices[minIndex] = swapIndex;

			swapDistance = distances[j];
			distances[j] = distances[minIndex];
			distances[minIndex] = swapDistance;
		}

		// sort distances, the the closests with minsort

		vector<int> closestVertexOneRingVec;

		float delta = abs(distances[0] - distances[1]) + 0.1f;

		closestVertexOneRingVec.push_back(indices[0]);

		for (int i=1; i < pMesh->numOfVertices; i++){
			if (abs(distances[i] - distances[i - 1]) > delta){
				break;
			} else {
				closestVertexOneRingVec.push_back(indices[i]);
				delta = distances[i] - distances[i - 1] + 0.1f;
			}
		}

		closestVertexOneRing[pNode->id - 1] = closestVertexOneRingVec;

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}

void GraphAlgorithms::computeObjectSkeletonShaderData(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, structure::t3DObject *pObject, ObjectSkeletonShaderData * data, int length, int indexOffset, bool isMeshGraphComputed, Array2D<float> distanceMatrix, int numOfControlBones, int maxBones, boost::unordered_map<int, vector<int> > &closestVertexOneRing){

	SN::SkeletonNode* sklRoot = new SN::SkeletonNode();

	if (isMeshGraphComputed)
		pObject->skinning = true;

	// if distance matrix is not computed
	if (distanceMatrix.dim1() == 0 && distanceMatrix.dim2() == 0)
		return;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeObjectSkeletonShaderData STARTED");
	#endif

	// currentSkeletonRoot sa mi koli niecomu uplne dojebka, zmenia sa idecka a stratia synovia...neviem co do nieho siaha
	// skusim na zaciatku ratania skopirovat do druhej isntancie a pracovat s tou

	data->numOfVertices = length;

	float* weights = new float[pObject->numOfVerts * numOfControlBones];
	float* indices = new float[pObject->numOfVerts * numOfControlBones];

	SN::SkeletonNode * base = new SN::SkeletonNode[numOfControlBones];
	float * invDist = new float[numOfControlBones];

	/*vector<SN::SkeletonNode*> queue;
	queue.push_back((SN::SkeletonNode*)sklRoot);


	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		//pNode->point = pNode->bindPoseMatrices.getTranslationFromAffine();

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}*/

	for (int i = 0; i < pObject->numOfVerts; i++) {
		copySkeletonNode(currentSkeletonRoot, sklRoot);

		//UNCOMMENT if (isMeshGraphComputed && pLBSExtractor.pMesh->indices[indexOffset + i] == -1)
			//pObject->skinning = false;	

		// find x control bones which have influence on vertex
		findXMinInTree(pMesh, distanceMatrix, numOfControlBones, sklRoot, base, pMesh->indices[indexOffset + i], closestVertexOneRing);

		// if there is less than NUM_OF_CTRL_BONE control points, multiply the best one
		/*vector<int> empty;
		int bestIndex = -1;
		float minDis = FLT_MAX;
		for (int k = 0; k < numOfControlBones; k++) {
			if (base[k].id < 1)
				empty.push_back(k);
			else {
				float f = GeodesicDistance(pMesh, distanceMatrix, (base[k].point + base[k].father->point) / 2.0, pMesh->indices[indexOffset + i], true);
				if (f < minDis){
					minDis = f;
					bestIndex = k;
				}
			}
		}
		if (bestIndex > -1)
			for (int k=0; k<empty.size(); k++)
				base[empty[k]] = base[bestIndex];*/

		float invDistSum = 0.0f;
		for (int k = 0; k < numOfControlBones; k++) {
			indices[i * numOfControlBones + k] = (float)base[k].id - 1;

			float f = GeodesicDistance(pMesh, distanceMatrix, (base[k].point + base[k].father->point) / 2.0, pMesh->indices[indexOffset + i], true, closestVertexOneRing[base[k].id - 1]);

			//float f = Distance(pObject->pVerts[i], (base[k].point + base[k].father->point) / 2.0 );
			// toto este nemozem pouzit, lebo ratanie vzdialenosti mi funguje iba jednosmerne top-bottom stromovo, nie oboma smermi ak v grafe
			// je tam problem ked najde najblizsie body k raidiacemu vrcholu a bodu meshu ten isty vrchol grafu
			//float f = GeodesicDistance(pLBSExtractor.pMesh, distanceMatrix, base[k].point, pLBSExtractor.pMesh->indices[indexOffset + i], true);

			if (f == 0)
				invDist[k] = FLT_MAX;
			else
				invDist[k] = 1.0 / f;

			invDistSum += invDist[k];

			#ifdef _LOG
				logg.log(0, "indexy pre vertex : " ,i);
				logg.log(0, "index", indices[i * numOfControlBones + k]);
			#endif
		}

		if (invDistSum > FLT_MAX)
			invDistSum = FLT_MAX;

		for (int k = 0; k < numOfControlBones; k++) {
			if (invDistSum != 0)
				weights[i*numOfControlBones + k] = (invDist[k] / invDistSum);
			else
				weights[i*numOfControlBones + k] = 1.0f;

			#ifdef _LOG
				logg.log(0, "vahy pre vertex : " , i);
				logg.log(0, "vaha ", weights[i * numOfControlBones + k]);
			#endif
		}
	}

	/*queue.clear();
	queue.push_back((SN::SkeletonNode*)sklRoot);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = (SN::SkeletonNode*)queue[queue.size() - 1];
		queue.pop_back();

		if (useBindPoseMatrices){
			pNode->point = pNode->bindPoseMatrices.getTranslationFromAffine();
		} else {
			pNode->point = pNode->matrices.getTranslationFromAffine();
		}

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}*/


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
	delete[] invDist;
	invDist = NULL;
	delete[] indices;
	indices = NULL;
	delete[] weights;
	weights = NULL;

	data->numOfBones = maxBones;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeObjectSkeletonShaderData ENDED");
	#endif
}

void GraphAlgorithms::computeMeshGraphSkinningData(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, ObjectSkeletonShaderData * data, bool isMeshGraphComputed, Array2D<float> distanceMatrix, int numOfControlBones, int maxBones, boost::unordered_map<int, vector<int> > &closestVertexOneRing){

	SN::SkeletonNode* sklRoot = new SN::SkeletonNode();

	// if distance matrix is not computed
	if (distanceMatrix.dim1() == 0 && distanceMatrix.dim2() == 0)
		return;

#ifdef _LOG
	logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeObjectSkeletonShaderData STARTED");
#endif

	data->numOfVertices = pMesh->numOfVertices;

	float* weights = new float[pMesh->numOfVertices * numOfControlBones];
	float* indices = new float[pMesh->numOfVertices * numOfControlBones];

	SN::SkeletonNode * base = new SN::SkeletonNode[numOfControlBones];
	float * distInv = new float[numOfControlBones];

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

	for (int i = 0; i < pMesh->numOfVertices; i++) {
		copySkeletonNode(currentSkeletonRoot, sklRoot);
		//UNCOMMENT if (isMeshGraphComputed && pLBSExtractor.pMesh->indices[indexOffset + i] == -1)
			//pObject->skinning = false;	

		// find x control bones which have influence on vertex
		findXMinInTree(pMesh, distanceMatrix, numOfControlBones, sklRoot, base, i, closestVertexOneRing);

		//SN::SkeletonNode b1 = base[0];
		//SN::SkeletonNode b2 = base[1];

		// if there is less than NUM_OF_CTRL_BONES control points, multiply the best one
		/*vector<int> empty;
		int bestIndex = -1;
		float minDis = FLT_MAX;
		for (int k = 0; k < numOfControlBones; k++) {
			if (base[k].id < 1)
				empty.push_back(k);
			else {
				float f = GeodesicDistance(pMesh, distanceMatrix, base[k].point, i, true);
				if (f < minDis){
					minDis = f;
					bestIndex = k;
				}
			}
		}
		if (bestIndex > -1)
			for (int k=0; k<empty.size(); k++)
				base[empty[k]] = base[bestIndex];*/

		float distInvSum = 0.0f;
		for (int k = 0; k < numOfControlBones; k++) {
			float indexxx = (float)base[k].id - 1;
			indices[i * numOfControlBones + k] = indexxx;
			float f = Distance(pMesh->pVerts[i], base[k].point);
			// toto este nemozem pouzit, lebo ratanie vzdialenosti mi funguje iba jednosmerne top-bottom stromovo, nie oboma smermi ak v grafe
			// je tam problem ked najde najblizsie body k raidiacemu vrcholu a bodu meshu ten isty vrchol grafu
			//float f = GeodesicDistance(pLBSExtractor.pMesh, distanceMatrix, base[k].point, pLBSExtractor.pMesh->indices[indexOffset + i], true);
			if (f == 0)
				distInv[k] = FLT_MAX;
			else
				distInv[k] = 1.0 / f;
			distInvSum += distInv[k];

			#ifdef _LOG
				logg.log(0, "indexy pre vertex : " ,i);
				logg.log(0, "index", indices[i * numOfControlBones + k]);
			#endif
		}

		if (distInvSum > FLT_MAX)
			distInvSum = FLT_MAX;

		for (int k = 0; k < numOfControlBones; k++) {
			float invDist;
			if (distInvSum != 0)
				weights[i*numOfControlBones + k] = (distInv[k] / distInvSum);
			else
				weights[i*numOfControlBones + k] = 1.0f;

			#ifdef _LOG
				logg.log(0, "vahy pre vertex : " , i);
				logg.log(0, "vaha ", weights[i * numOfControlBones + k]);
			#endif
		}
	}

	queue.clear();
	queue.push_back((SN::SkeletonNode*)sklRoot);

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


	memcpy(data->indices, indices,  pMesh->numOfVertices * numOfControlBones * sizeof(float));
	memcpy(data->weights, weights,  pMesh->numOfVertices * numOfControlBones * sizeof(float));


	//delete[] base;
	//base = NULL;
	delete[] distInv;
	distInv = NULL;
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


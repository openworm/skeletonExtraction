//---------------------------------------------------------------------------
#include "sdf_extractor.h"

void sdf::Extractor::applyMedianMerge(MeshGraph * polyMesh, MeshGraph * pMesh, float threshold){

	MeshGraph * medianMesh = new MeshGraph();
	copyMeshGraph(pMesh, medianMesh);

	vector<CVector3> polyMeshVerts;
	map<int, set<int> > polyMeshIndices;

	for (int i=0; i < medianMesh->numOfVertices; i++){

		std::vector<int> neighs;
		// find lokal neighbors of vertex

		for (int j=0; j < medianMesh->numOfVertices; j++){

			float dist = Distance(medianMesh->pVerts[i], medianMesh->pVerts[j]);

			if (dist < threshold){
				neighs.push_back(j);
			}
		}

		int k = neighs.size();

		// calculate median

		CVector3 median(0,0,0);

		for (int j=0; j<neighs.size(); j++){
			median = median + medianMesh->pVerts[neighs[j]];
		}

		median = median / neighs.size();

		// move all vertices in neigh into median

		for (int j=0; j<neighs.size(); j++){
			medianMesh->pVerts[neighs[j]] = median;
		}

	}

	/*for (int j=0; j < polyMeshVerts.size(); j++){
		vector<int> indices;
		polyMeshIndices.push_back(indices);
	}*/


	//polyMeshIndices.resize(polyMeshVerts.size());

/*#ifdef _LOG
	logg.log(0, "Matica E pMesh");
	logg.log(0, pMesh->E);
#endif*/

	for (int i=0; i < medianMesh->numOfVertices; i++){
		bool add = true;
		for (int j=0; j < polyMeshVerts.size(); j++){
			if (Distance(medianMesh->pVerts[i], polyMeshVerts[j]) < 0.00001){
				add = false;
				polyMeshIndices[j].insert(i);
			}
		}
		if (add){
			polyMeshVerts.push_back(medianMesh->pVerts[i]);
			polyMeshIndices[polyMeshVerts.size() - 1].insert(i);
		}
	}

	polyMesh->numOfVertices = polyMeshVerts.size();
	polyMesh->pVerts = new CVector3[polyMesh->numOfVertices];
	for (int i=0; i<polyMesh->numOfVertices; i++){
		polyMesh->pVerts[i] = polyMeshVerts[i];
	}

	polyMesh->E = Array2D< bool >(polyMesh->numOfVertices, polyMesh->numOfVertices, false);
	polyMesh->numOfVBOlines = 0;

	for (int i=0; i<polyMesh->numOfVertices; i++){
		vector<int> indicesi;

		#ifdef _LOG
			logg.log(0, "polymesh vertex", polyMesh->pVerts[i]);
		#endif

		indicesi.assign( polyMeshIndices[i].begin(), polyMeshIndices[i].end() );

		/*#ifdef _LOG
				logg.log(0, "vector indexov z polyMesh");
				logg.log(0, "i: ", i);
				logg.log(0, indicesi);
		#endif*/

		for (int j=0; j<polyMesh->numOfVertices; j++){
			vector<int> indicesj;
			indicesj.assign( polyMeshIndices[j].begin(), polyMeshIndices[j].end() );

			for (int ii=0; ii<indicesi.size(); ii++){
				for (int jj=0; jj<indicesj.size(); jj++){
					int iii = indicesi[ii];
					int jjj = indicesj[jj];

					bool edgeee = pMesh->E[iii][jjj];

					if (edgeee){
						polyMesh->E[i][j] = true;
						polyMesh->numOfVBOlines++;
					}
				}
			}
		}
	}

	polyMesh->triangleIndices = new int[0];
	polyMesh->numOfFaces = 0;
}


void sdf::Extractor::subdivideSkeletonMeshGraphByCrossSectionMedians(MeshGraph * polyMesh, MeshGraph * pMesh, int segmentSubdivision){

	MeshGraph * tesselatedMesh = new MeshGraph();

	copyMeshGraph(polyMesh, tesselatedMesh);

	for (int i=0; i<polyMesh->numOfVertices; i++){
		for (int j=i; j<polyMesh->numOfVertices; j++){

			// for each edge in polyMesh, subdivide the edge and move each subdivision point into the median of pMesh vertices around it
			if (polyMesh->E[i][j] && i != j){
				CVector3 edgeVector = polyMesh->pVerts[j] - polyMesh->pVerts[i];

				float tesselatedEdgeSize = Magnitude(edgeVector / (float)segmentSubdivision);

				vector<CVector3> newTesselatedPoints;

				for (int s=0; s < segmentSubdivision; s++){
					CVector3 tesselatedPoint = polyMesh->pVerts[s] + edgeVector * ((float)s / (float)segmentSubdivision);

					int numOfPointsInMedian = 0;
					CVector3 median(0,0,0);
					for (int k=0; k < pMesh->numOfVertices; k++){
						if (Distance(tesselatedPoint, pMesh->pVerts[k]) < tesselatedEdgeSize){
							median = median + pMesh->pVerts[k];
							numOfPointsInMedian++;
						}
					}

					median = median / numOfPointsInMedian;
					newTesselatedPoints.push_back(median);

				}

				// put new tesselated points onto current i-j edge

				CVector3 * newVerts = new CVector3[tesselatedMesh->numOfVertices + newTesselatedPoints.size()];

				for (int v=0; v<tesselatedMesh->numOfVertices; v++){
					newVerts[v] = tesselatedMesh->pVerts[v];
				}

				for (int v=0; v<newTesselatedPoints.size(); v++){
					newVerts[tesselatedMesh->numOfVertices + v] = newTesselatedPoints[v];
				}

				delete[] tesselatedMesh->pVerts;
				tesselatedMesh->pVerts = NULL;

				tesselatedMesh->pVerts = newVerts;

				// update edges

				Array2D<bool> newE = Array2D<bool>(tesselatedMesh->numOfVertices + newTesselatedPoints.size(), tesselatedMesh->numOfVertices + newTesselatedPoints.size(), false);
				

				for (int ii=0; ii < tesselatedMesh->numOfVertices; ii++){
					for (int jj=0; jj < tesselatedMesh->numOfVertices; jj++){
						if (tesselatedMesh->E[ii][jj]){ // && !((ii == i & jj == j) || (ii == j & jj == i))
							newE[ii][jj] = true;
							newE[jj][ii] = true;
						}
					}
				}


				/*int previousEdgeIndex = i;
				for (int ii=0; ii < newTesselatedPoints.size(); ii++){
						newE[previousEdgeIndex][tesselatedMesh->numOfVertices + ii] = true;
						newE[tesselatedMesh->numOfVertices + ii][previousEdgeIndex] = true;

						previousEdgeIndex = tesselatedMesh->numOfVertices + ii;
				}

				newE[previousEdgeIndex][j] = true;
				newE[j][previousEdgeIndex] = true;

				// remove original i-j edge

				newE[i][j] = false;
				newE[j][i] = false;*/

				tesselatedMesh->E = newE.copy();

				// update vertex count

				tesselatedMesh->numOfVertices = tesselatedMesh->numOfVertices + newTesselatedPoints.size();

			}
		}
	}

	copyMeshGraph(tesselatedMesh, polyMesh);
	delete tesselatedMesh;
}

void sdf::Extractor::subdivideSkeletonByCrossSectionMedians(SN::SkeletonNode* pRoot, MeshGraph * pMesh, int segmentSubdivision){

	std::vector<SN::SkeletonNode*> queue;
	queue.push_back(pRoot);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		#ifdef _LOG
			logg.log(0, "skeleton vertex", pNode->point);
		#endif

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = (SN::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);

			// for each edge in polyMesh, subdivide the edge and move each subdivision point into the median of pMesh vertices around it
			CVector3 edgeVector = pSon->point - pNode->point;

			float tesselatedEdgeSize = Magnitude(edgeVector / (float)segmentSubdivision);

			vector<CVector3> newTesselatedPoints;

			for (int s=1; s < segmentSubdivision; s++){
				CVector3 tesselatedPoint = pNode->point + edgeVector * ((float)s / (float)segmentSubdivision);

				int numOfPointsInMedian = 0;
				CVector3 median(0,0,0);
				for (int k=0; k < pMesh->numOfVertices; k++){
					if (Distance(tesselatedPoint, pMesh->pVerts[k]) < (tesselatedEdgeSize / 2.0)){
						median = median + pMesh->pVerts[k];
						numOfPointsInMedian++;
					}
				}

				if (numOfPointsInMedian > 0){
					median = median / numOfPointsInMedian;
					newTesselatedPoints.push_back(median);
				}

			}

			SN::SkeletonNode * father = pNode;
			pNode->nodes.erase(pNode->nodes.begin() + i);

			for (int v = 0; v < newTesselatedPoints.size(); v++){
				SN::SkeletonNode * newSon = new SN::SkeletonNode();
				newSon->point = newTesselatedPoints[v];
				father->nodes.push_back(newSon);
				father = newSon;
			}

			// last child should be original son
			father->nodes.push_back(pSon);

		}

	}

}
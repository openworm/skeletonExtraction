
inline void halfEdgeCollapse(SurgeryGraph * pGraph, int i, int j, int * actualNumOfBones, bool updateQ){
		// half-edge collapse mesh (i->j), copy to j edges from i and set to -1 edges in i
		#ifdef _LOG
			logg.log(LOG_LEVEL_C_PARAMS, "Vrchol I",i);
			logg.log(LOG_LEVEL_C_PARAMS, "do vrchola J ",j);
		#endif

		// update Q matrix
		if (updateQ)
			pGraph->QMatrices[j] = pGraph->QMatrices[j] + pGraph->QMatrices[i];

		#ifdef _LOG
				logg.log(LOG_LEVEL_C_PARAMS, "pGraph->numOfVertices : ", pGraph->numOfVertices);

				logg.log(LOG_LEVEL_C_PARAMS, "pointClouds pre vrchol : ",j);
				logg.log(LOG_LEVEL_C_PARAMS, "pointClouds velkost pred presunom : ",(int)pGraph->pointClouds[j].size());
		#endif

		for (int k=0; k< pGraph->pointClouds[i].size(); k++)
			pGraph->pointClouds[j].push_back(pGraph->pointClouds[i][k]);
		pGraph->pointClouds[i].clear();

		#ifdef _LOG
			logg.log(LOG_LEVEL_C_PARAMS, "pointClouds velkost po presune : ",(int)pGraph->pointClouds[j].size());
		#endif

		for (int k = 0; k < pGraph->numOfVertices; k++) {
			if (pGraph->E[i][k]) {
				// move edges from i to j
				pGraph->E[j][k] = true;
				pGraph->E[k][j] = true;

				pGraph->E[i][k] = false;
				pGraph->E[k][i] = false;

				//bool push = true;
				/*for (int k = 0; k < pGraph->pointClouds[j].size(); k++) {
					if(pGraph->pointClouds[j][k] == i)  {
						push = false;
					}
				}*/
				//if (push) {
				//	pGraph->pointClouds[j].push_back(i);
				//}
			}
		}

		pGraph->E[j][i] = false;
		pGraph->E[i][j] = false;

		#ifdef _LOG
			logg.log(LOG_LEVEL_DUMP, "Matica E");
			logg.log(LOG_LEVEL_DUMP, pGraph->E);
		#endif
		(*actualNumOfBones)--;
}

//---------------------------------------------------------------------------
inline void halfEdgeCollapse(MeshGraph * pMesh, boost::unordered_map<int, std::vector<int> > &mgDegeneratesMapping, int i, int j){
		/*logg.log(0, "E pred HEC: ");
		logg.log(0, pMesh->E);
*/
		//logg.log(LOG_LEVEL_WARNING, "Aplikuje sa kontrakcna half-edge collapse");
		//logg.log(LOG_LEVEL_WARNING, "i: ",i);
		//logg.log(LOG_LEVEL_WARNING, "-> j: ",j);

	/*	CVector3 * newpoints = new CVector3[pMesh->numOfVertices - 1];
		float * newwH = new float[pMesh->numOfVertices - 1];
		float * newwHorig = new float[pMesh->numOfVertices - 1];
		Array2D<bool> newE(pMesh->numOfVertices - 1, pMesh->numOfVertices - 1, false);*/

		for (int k = 0; k < pMesh->numOfVertices; k++) {
			if (pMesh->E[i][k] && j != k) {
				pMesh->E[j][k] = true;
				pMesh->E[k][j] = true;

				pMesh->E[i][k] = false;
				pMesh->E[k][i] = false;
			}
		}

		pMesh->E[j][i] = false;
		pMesh->E[i][j] = false;

		//logg.log(0, "E po HEC: ");
		//logg.log(0, pMesh->E);

	/*	for (int m=0; m < i; m++){
			for (int n=0; n < i; n++)
				newE[m][n] = pMesh->E[m][n];
			for (int n=i+1; n < pMesh->numOfVertices; n++)
				newE[m][n-1] = pMesh->E[m][n];
			newpoints[m] = pMesh->pVerts[m];
			newwH[m] = pMesh->wH[m];
			newwHorig[m] = pMesh->wHorig[m];
		}
		for (int m=i+1; m < pMesh->numOfVertices; m++){
			for (int n=0; n < i; n++)
				newE[m - 1][n] = pMesh->E[m][n];
			for (int n=i+1; n < pMesh->numOfVertices; n++)
				newE[m - 1][n - 1] = pMesh->E[m][n];
			newpoints[m-1] = pMesh->pVerts[m];
			newwH[m-1] = pMesh->wH[m];
			newwHorig[m-1] = pMesh->wHorig[m];
		}
		for (boost::unordered_map<int, int>::iterator it=pMesh->indices.begin(); it != pMesh->indices.end(); it++){
			if (pMesh->indices[it->second] > i){
					pMesh->indices[it->second] = pMesh->indices[it->second] - 1;
			}
			else {
				if (pMesh->indices[it->second] == i)
					if (j > i)
						pMesh->indices[it->second] = j - 1;
					else
						pMesh->indices[it->second] = j;
			}
		}

		boost::unordered_map<int, vector<int> > newmgDegeneratesMapping;

		for (int v=0; v < mgDegeneratesMapping[i].size(); v++)
			mgDegeneratesMapping[j].push_back(mgDegeneratesMapping[i][v]);

		for (boost::unordered_map<int, vector<int> >::iterator it=mgDegeneratesMapping.begin(); it != mgDegeneratesMapping.end();){
			if (it->first < i){
				newmgDegeneratesMapping[it->first] = it->second;
				it++;
			} else {
				int key = it->first;
				it++;
				newmgDegeneratesMapping[key] = it->second;
			}
		}

		mgDegeneratesMapping = newmgDegeneratesMapping;


		delete[] pMesh->pVerts;
		pMesh->pVerts = NULL;
		pMesh->pVerts = newpoints;

		delete[] pMesh->wH;
		pMesh->wH = NULL;
		pMesh->wH = newwH;

		delete[] pMesh->wHorig;
		pMesh->wHorig = NULL;
		pMesh->wHorig = newwHorig;

		pMesh->E = NULL;
		pMesh->E = newE.copy();

		pMesh->numOfVertices = pMesh->numOfVertices - 1;*/
}

//---------------------------------------------------------------------------
/*inline bool isMeshTriangle(MeshGraph * pMesh, structure::t3DModel *pModel, int a,int b,int c){
	int offset = 0;
	for (int j = 0; j < pModel->numOfObjects; j++){
		structure::t3DObject *pObject = &pModel->pObject[j];
		for (int k = 0; k < pObject->numOfFaces; k++){
			int ind0 = pMesh->indices[offset + pObject->pFaces[k].vertIndex[0]];
			int ind1 = pMesh->indices[offset + pObject->pFaces[k].vertIndex[1]];
			int ind2 = pMesh->indices[offset + pObject->pFaces[k].vertIndex[2]];
			if ((a == ind0 && b == ind1 && c == ind2) ||
				(a == ind0 && c == ind1 && b == ind2) ||
				(b == ind0 && a == ind1 && c == ind2) ||
				(b == ind0 && c == ind1 && a == ind2) ||
				(c == ind0 && a == ind1 && b == ind2) ||
				(c == ind0 && b == ind1 && a == ind2))
				return true;
		}
		offset += pObject->numOfVerts;
	}
	return false;
}*/

//---------------------------------------------------------------------------
inline bool isMeshTriangle(MeshGraph * pMesh, int a,int b,int c){
	for (int j = 0; j < pMesh->numOfFaces; j++){
		int ind0 =  pMesh->triangleIndices[j * 3];
		int ind1 =  pMesh->triangleIndices[j * 3 + 1];
		int ind2 =  pMesh->triangleIndices[j * 3 + 2];

		if ((a == ind0 && b == ind1 && c == ind2) ||
			(a == ind0 && c == ind1 && b == ind2) ||
			(b == ind0 && a == ind1 && c == ind2) ||
			(b == ind0 && c == ind1 && a == ind2) ||
			(c == ind0 && a == ind1 && b == ind2) ||
			(c == ind0 && b == ind1 && a == ind2)){
				return true;
		}
	}
	return false;
}

//---------------------------------------------------------------------------
inline void createMeshGraph(structure::t3DModel *pModel,MeshGraph * pMesh, VertexJoiningVisualization * pJoining, float * joinings, float delta, float wL, float wH, int numOfComponents, std::vector<int> compMapping){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("createMeshGraph");

		timerlog.addStart("create vertex array");
	#endif
	
	boost::unordered_map<int, int> indicesBackup = pMesh->indices;

	//delete pMesh;
	//pMesh = NULL;
	//pMesh = new MeshGraph();

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createMeshGraph STARTED");
	#endif

	std::vector<CVector3> v;
	std::vector<CVector3> n;
	std::vector<int> componentIndices;
	int prevComponentVertexIndex = -1;
	int num = 0;
	float area = 0;
	int numOfFaces = 0;

    int offset = 0;
	int joinIdx = 0;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START createMeshGraph - vertices contruction");
	#endif

	std::vector<int> pindices;
	int maxIndex = 0;
	float joining = 0.0f;

	for (int i = 0; i < pModel->numOfObjects; i++){
		structure::t3DObject *pObject = &pModel->pObject[i];
		//pObject->pMeshGraphIndices = new int[pObject->numOfVerts];
		//for (int k = 0; k < pObject->numOfVerts; k++)
			//pObject->pMeshGraphIndices[k] = -1;
		for(int j = 0; j < pObject->numOfFaces; j++){
				for(int whichVertex = 0; whichVertex < 3; whichVertex++){
					
				   int index = pObject->pFaces[j].vertIndex[whichVertex];
				   CVector3 point = pObject->pVerts[index];
				   // there are multi vertices in 3ds file, we want same vertices only 1st time
				   bool add = true;
				   if (numOfComponents == -1)
					   joinIdx = i;
				   else
					   joinIdx = compMapping[indicesBackup[offset + index]];
				   for (int k = 0; k < v.size(); k++) {
					   float distance = Distance(v[k], point);
					   if (joinings[joinIdx] < 0)
						   distance = -distance;
					   if ((distance < delta) || (distance < joinings[joinIdx] && prevComponentVertexIndex != -1 && k < prevComponentVertexIndex)) {
						 add = false;
						 pMesh->indices[offset + index] = k;
						 pindices.push_back(k);
						 //pMesh->inverseIndices[k].push_back(index + offset);
						 //pObject->pMeshGraphIndices[index] = k;
						 break;
					   }
				   }

				   if (add) {
					   pMesh->indices[offset + index] = num;
					   pindices.push_back(num);
					   //pMesh->inverseIndices[num].push_back(index + offset);
					   //pObject->pMeshGraphIndices[index] = num;
					   num ++;
					   v.push_back(point);
					   CVector3 normal = pObject->pNormals[index];
					   n.push_back(normal);
					   componentIndices.push_back(joinIdx);
				   }

			}
			// avarage one ring area
			numOfFaces ++;
			CVector3 p1 = pObject->pVerts[pObject->pFaces[j].vertIndex[0]];
			CVector3 p2 = pObject->pVerts[pObject->pFaces[j].vertIndex[1]];
			CVector3 p3 = pObject->pVerts[pObject->pFaces[j].vertIndex[2]];
			float a = Magnitude(Cross(p2 - p1, p3 - p1)) / 2.0;
			area += a;
		}
		prevComponentVertexIndex = v.size() - 1;
	 	offset += pObject->numOfVerts;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "END createMeshGraph - vertices contruction");
	#endif

	// take only 100 closest neighbourhs

	//vector<CVector3> neigh = findNeighborhood(0, v, (pModel->modelbb.x_max - pModel->modelbb.x_min) / 2.5);

	//mean face area
	pMesh->numOfFaces = numOfFaces;
	pMesh->faceAreaSum = area;
	/*logg.log(2, "Avarage model Area: ", (pMesh->faceAreaSum / (float)pMesh->numOfFaces));
	float t = pMesh->faceAreaSum/(pMesh->faceAreaSum / (float)pMesh->numOfFaces);
	logg.log(LOG_LEVEL_C_PARAMS, "term t :", t);
	pMesh->wL = wL * t;*/

	pMesh->numOfVertices = num;
	delete[] pMesh->pVerts;
	pMesh->pVerts = NULL;
	pMesh->pVerts = new CVector3[num];
	delete[] pMesh->wH;
	pMesh->wH = NULL;
	pMesh->wH = new float[num];
	delete[] pMesh->wHorig;
	pMesh->wHorig = NULL;
	pMesh->wHorig = new float[num];
	delete[] pMesh->triangleIndices;
	pMesh->triangleIndices = NULL;

	for (int i = 0; i < num; i++){
		pMesh->pVerts[i] = v[i];
		//pMesh->wH[i] = 1.0f;
		//pMesh->wH[i] = 0.1f;
		//pMesh->wH[i] = 0.01f;
		//pMesh->wH[i] = 0.08f;
		//pMesh->wH[i] = 0.2f;
		pMesh->wH[i] = wH;
		pMesh->wHorig[i] = wH;
	}

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("create joinings");
	#endif

	pMesh->E = Array2D< bool >(num, num, false);

	int * numIndex = new int[3];

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START createMeshGraph - joining contruction");
	#endif

	std::vector<CVector3> joiningFrom;
	std::vector<CVector3> joiningTo;
	for (int i = 0; i < num; i++){
		// for each selected node find his neighbours
		offset = 0;
		for (int j = 0; j < pModel->numOfObjects; j++){
			structure::t3DObject *pObject = &pModel->pObject[j];
			for (int k = 0; k < pObject->numOfFaces; k++){
				for(int whichVertex = 0; whichVertex < 3; whichVertex++){
					int index = pObject->pFaces[k].vertIndex[whichVertex];
					CVector3 point = pObject->pVerts[index];
					// if the points are "the same"
					float dist = Distance(v[i], point);

				   if (numOfComponents == -1)
				    	joinIdx = j;
				   else
					   joinIdx = compMapping[indicesBackup[offset + index]];

					if (dist < joinings[joinIdx] && joinIdx != componentIndices[i]){
						if (dist > 0){
							// create vector pairs for visualization
							joiningFrom.push_back(v[i]);
							joiningTo.push_back(point);
                        }
					}
					if ((dist < delta) || (dist < joinings[joinIdx] && joinIdx != componentIndices[i])){
                    		// get corresponding index for each vertex in face in from selected nodes
							for (int m = 0; m < 3; m++) {
								numIndex[m] = pMesh->indices[offset + pObject->pFaces[k].vertIndex[m]];
								/*point = pObject->pVerts[pObject->pFaces[k].vertIndex[m]];
								for (int l = 0; l < num; l++){
									if (Distance(v[l], point) < 0.0000001f)
										numIndex[m] = l;
								}*/
							}
							if (whichVertex == 0) {
									pMesh->E[i][numIndex[1]] = true;
									pMesh->E[i][numIndex[2]] = true;

									pMesh->E[numIndex[1]][i] = true;
									pMesh->E[numIndex[2]][i] = true;
								} else if (whichVertex == 1) {
										pMesh->E[i][numIndex[0]] = true;
										pMesh->E[i][numIndex[2]] = true;

										pMesh->E[numIndex[0]][i] = true;
										pMesh->E[numIndex[2]][i] = true;
									} else if (whichVertex == 2) {
											pMesh->E[i][numIndex[0]] = true;
											pMesh->E[i][numIndex[1]] = true;

											pMesh->E[numIndex[0]][i] = true;
											pMesh->E[numIndex[1]][i] = true;
										}
					}
				}
			}
			offset += pObject->numOfVerts;
		}
	}

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("create joining visualziation");

		logg.log(LOG_LEVEL_METHODSTARTEND, "END createMeshGraph - joining contruction");
	#endif

	// CHECK THE E MATRIX, IF ALL EDGES ARE IN TRIANGLE
	/*for (int i=0; i < pMesh->numOfVertices; i++)
		for (int j=0; j < pMesh->numOfVertices; j++)
			if (pMesh->E[i][j]){
				bool notTriangle = true;
				for (int k=0; k < pMesh->numOfVertices; k++)
					if (k != i && k != j && pMesh->E[i][k] && pMesh->E[j][k])
						notTriangle = false;
				if (notTriangle){
					pMesh->E[i][j] = false;
					pMesh->E[j][i] = false;
				}
			}*/

	//create vertex joining visualisation data
	if (pJoining != NULL) {
		pJoining->numOfPairs = joiningFrom.size();
		delete[] pJoining->from;
		pJoining->from = NULL;
		delete[] pJoining->to;
		pJoining->to = NULL;
		pJoining->from = new CVector3[joiningFrom.size()];
		pJoining->to = new CVector3[joiningTo.size()];
		for (int i = 0; i < pJoining->numOfPairs; i++) {
			pJoining->from[i] = joiningFrom[i];
			pJoining->to[i]  = joiningTo[i];
		}
	}

	delete[] numIndex;
	numIndex = NULL;
	if (pMesh->origOneRingArea != NULL){
		delete[] pMesh->origOneRingArea;
		pMesh->origOneRingArea = NULL;
		pMesh->origOneRingArea = new float[pMesh->numOfVertices];
	}
	if (pMesh->origOneRingExtent != NULL){
		pMesh->origOneRingExtent = NULL;
		pMesh->origOneRingExtent = new float[pMesh->numOfVertices];
	}

	int aa = pindices.size();
	int bb = pMesh->numOfFaces;

	pMesh->triangleIndices = new int[pMesh->numOfFaces * 3];
	for (int i=0;i < pMesh->numOfFaces * 3; i++){
		pMesh->triangleIndices[i] = pindices[i];
	}

	calculateOneRingArea(pMesh, pModel, pMesh->origOneRingArea);

	/*pMesh->pColor = new CVector3[pMesh->numOfVertices];
	for (int i=0;i < pMesh->numOfVertices; i++){
		pMesh->pColor[i] = CVector3(1,1,1);
	}*/

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_DUMP, "Matica E");
		logg.log(LOG_LEVEL_DUMP, pMesh->E);

		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD createMeshGraph ENDED");

		timerlog.logExecutionTimes();
	#endif

	//pMesh->numOfVertices = neigh.size();

} 

inline void createMeshGraph(meshes::IndexedFace *mesh, MeshGraph * pMesh, float wL, float wH){

	pMesh->numOfVertices = mesh->vertices.size() / 3;
	delete[] pMesh->pVerts;
	pMesh->pVerts = NULL;
	pMesh->pVerts = new CVector3[pMesh->numOfVertices];

	delete[] pMesh->wH;
	pMesh->wH = NULL;
	pMesh->wH = new float[pMesh->numOfVertices];
	delete[] pMesh->wHorig;
	pMesh->wHorig = NULL;
	pMesh->wHorig = new float[pMesh->numOfVertices];

	if (mesh->indices.size() != 0){
		delete[] pMesh->triangleIndices;
		pMesh->triangleIndices = NULL;
	
		pMesh->triangleIndices = new int[mesh->indices.size()];
		pMesh->triangleIndices = &(mesh->indices[0]);

		pMesh->numOfFaces = mesh->indices.size() / 3;
	}

	for (int i=0; i < mesh->vertices.size() / 3; i++){
		pMesh->pVerts[i] = CVector3(mesh->vertices[i * 3], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]);
	}

	pMesh->E = Array2D<bool>(pMesh->numOfVertices,pMesh->numOfVertices, false);

	for (int i=0; i < mesh->indices.size() / 3; i++){
		int i1 = mesh->indices[i * 3];
		int i2 = mesh->indices[i * 3 + 1];
		int i3 = mesh->indices[i * 3 + 2];

		pMesh->E[i1][i2] = true;
		pMesh->E[i2][i3] = true;
		pMesh->E[i3][i1] = true;

		pMesh->E[i2][i1] = true;
		pMesh->E[i3][i2] = true;
		pMesh->E[i1][i3] = true;
	}

	pMesh->wL = wL;

	for (int i = 0; i < pMesh->numOfVertices; i++){
		pMesh->wH[i] = wH;
		pMesh->wHorig[i] = wH;
	}

}


inline void calculateOneRingExtent(MeshGraph * pMesh, float * p, std::vector<std::set<int>> globalNeighbourhoods){

	/*
	// Tu sa zistuje dlzka so sudmi cez hrany, nieje korektne pre point cloud

	for (int i = 0; i < pMesh->numOfVertices; i++) {
		float minedge = FLT_MAX;
		for (int j = 0; j < pMesh->numOfVertices; j++){
			if (pMesh->E[i][j]){
				float length = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[j]);
				if (length < minedge)
					minedge  = length;
				}
			p[i] = minedge;						
		}
	}*/

	// Tu sa rata ten one ring extent ako minimalna vzdialenost s k susedmi
	for (int i = 0; i < pMesh->numOfVertices; i++) {
		float minedge = FLT_MAX;
		std::set<int> neighs = globalNeighbourhoods[i];
		for (std::set<int>::iterator it1=neighs.begin(); it1!=neighs.end(); ++it1){
			float length = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[*it1]);
			if (length < minedge)
				minedge  = length;
		}
			p[i] = minedge;						
	}
}

/*boost::unordered_map<int, std::vector<int> > MeshGraph::returnIdentityDegeneratesMapping(MeshGraph * pMesh)
{
	boost::unordered_map<int, std::vector<int> > identitydm;
	for (int i=0; i<numOfVertices; i++){
		std::vector<int> vecmap;
		vecmap.push_back(i);
		identitydm[i] = vecmap;
	}
	return identitydm;
}*/

inline PCTMeshGraph * MG2PCTMG(MeshGraph * pMesh){

	PCTMeshGraph * pctMesh = new PCTMeshGraph();
	pctMesh->numOfVertices = pMesh->numOfVertices;
	pctMesh->E = pMesh->E.copy();

	pctMesh->pVerts = new PCTCVector3[pMesh->numOfVertices];
	for (int i=0; i < pMesh->numOfVertices; i++){
		pctMesh->pVerts[i].x = pMesh->pVerts[i].x;
		pctMesh->pVerts[i].y = pMesh->pVerts[i].y;
		pctMesh->pVerts[i].z = pMesh->pVerts[i].z;
	}


	return pctMesh;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------


inline void releaseStructure(t3DModel *pModel){
	for (int i = 0; i < pModel->numOfObjects; i++){
		 t3DObject *pObject = &pModel->pObject[i];

		/*for (int m=0; m < pModel->pMaterials.size(); m++){
			tMaterialInfo mat = pModel->pMaterials[m];
			delete[] mat.pTextredFaces;
			mat.pTextredFaces = NULL;
		}*/

		//delete[] (pObject->pIndices);
		//pObject->pIndices = NULL;
		delete[] (pObject->pVerts);
		pObject->pVerts = NULL;
		delete[] (pObject->pNormals);
		pObject->pNormals = NULL;
		//delete[] (pObject->pFaceNormals);
		//pObject->pFaceNormals = NULL;
		delete[] (pObject->pTangents);
		pObject->pTangents = NULL;
		delete[] (pObject->pTexVerts);
		pObject->pTexVerts = NULL;
		delete[] (pObject->pFaces);
		pObject->pFaces = NULL;
		//delete[] (pObject->pVertID);
		//pObject->pVertID = NULL;
	}

}

//---------------------------------------------------------------------------

// Def : Neighbours of vertex is union through all triangles of 2 vertices of triangle, where 1
// vertex of triangle has same coordinates as source vertex. Calculate for all triangles. O(n^2).

inline void createModelNeighbours(t3DModel *pModel, float delta){
  /* 	int offset = 0;
	for (int i = 0; i < pModel->numOfObjects; i++){
		t3DObject *pObject = &pModel->pObject[i];
			for (int j = 0; j < pObject->numOfVerts; j++){
			   CVector3 point = pObject->pVerts[j];
			   vector<int> nbrs;
//			   Application->ProcessMessages();

 			   for (int k = 0;  k< pModel->numOfObjects; k++){
					t3DObject *pObject2 = &pModel->pObject[k];
					for (int l = 0; l < pObject2->numOfFaces; l++){
						for(int whichVertex = 0; whichVertex < 3; whichVertex++)
						{
							int index = pObject2->pFaces[l].vertIndex[whichVertex];
							CVector3 p2 = pObject2->pVerts[index];

							if (Distance(p2, point) < delta){
								if (whichVertex == 0) {
									nbrs.push_back(pObject2->pFaces[l].vertIndex[1] + offset);
									nbrs.push_back(pObject2->pFaces[l].vertIndex[2] + offset);
								} else if (whichVertex == 1) {
									nbrs.push_back(pObject2->pFaces[l].vertIndex[0] + offset);
									nbrs.push_back(pObject2->pFaces[l].vertIndex[2] + offset);
									} else if (whichVertex == 2) {
										nbrs.push_back(pObject2->pFaces[l].vertIndex[1] + offset);
										nbrs.push_back(pObject2->pFaces[l].vertIndex[0] + offset);
                     					}
							}
						}
					}
				}


			pModel->neighbours.push_back(nbrs);
			}
			offset += pObject->numOfVerts;
	}
	dumpNeighbours(pModel);*/
}

//---------------------------------------------------------------------------

inline void calculateVerticeCount(t3DObject *pObject){

	int vertexCount = 0;
	
			for(int j = 0; j < pObject->numOfFaces; j++)
				for(int whichVertex = 0; whichVertex < 3; whichVertex++)
					vertexCount++;
	pObject->numOfVertices = vertexCount;
}

//---------------------------------------------------------------------------

inline void calculateVerticeCount(t3DModel *pModel){

	int vertexCount = 0;
	int vertCount = 0;

	for(int i = 0; i < pModel->numOfObjects; i++)
	{
		if(pModel->pObject.size() <= 0) break;
		t3DObject *pObject = &pModel->pObject[i];
		vertCount += pObject->numOfVerts;
			for(int j = 0; j < pObject->numOfFaces; j++)
			{
				for(int whichVertex = 0; whichVertex < 3; whichVertex++)
				{
					vertexCount++;
				}
			}
	}
	pModel->numOfVertices = vertexCount;
	pModel->numOfVerts = vertCount;
	#ifdef _LOG
		Log::log(LOG_LEVEL_C_PARAMS, "pocet vertexov ", vertexCount);
	#endif
}

//---------------------------------------------------------------------------

inline CVector3 getVertexWithID(t3DModel *pModel, int id){
	for (int i = 0; i < pModel->numOfObjects; i++){
		t3DObject *pObject = &pModel->pObject[i];
			for (int j = 0; j < pObject->numOfVerts; j++){
				if (pObject->pVertID[j] == id)
                	return pObject->pVerts[j];
			}
	}
	return CVector3(0,0,0);
}

//---------------------------------------------------------------------------

inline int  findClosestPointID(t3DModel *pModel, CVector3 sourcePoint){
	float minDist = FLT_MAX;
	int id = -1;
	for (int i = 0; i < pModel->numOfObjects; i++){
		t3DObject *pObject = &pModel->pObject[i];
			for (int j = 0; j < pObject->numOfVerts; j++){
			   CVector3 p = pObject->pVerts[j];
			   float dist = Distance(p, sourcePoint);
			   if (dist < minDist){
				   minDist = dist;
				   id = pObject->pVertID[j];
			   }
			}
	}
	return id;
}


//---------------------------------------------------------------------------

inline void dumpNeighbours(t3DModel *pModel){

	
}

//---------------------------------------------------------------------------

inline void dumpDistances(t3DModel *pModel){

	for(int i = 0; i < pModel->numOfObjects; i++)
	{
		if(pModel->pObject.size() <= 0) break;
		t3DObject *pObject = &pModel->pObject[i];
		//Log::log("Number of faces: ",System::Convert::ToString(pObject->numOfFaces));
		for(int j = 0; j < pObject->numOfFaces; j++)
		{
			for(int whichVertex = 0; whichVertex < 3; whichVertex++)
			{
				int index = pObject->pFaces[j].vertIndex[whichVertex];
				int vertID = pObject->pVertID[index];
				//Log::log((System::Convert::ToString(vertID)+"  "),System::Convert::ToString(pModel->pDistVert[vertID]));
				}
			}
		}
	  //Log::log("Max distance  :  ", System::Convert::ToString(pModel->maxDistance));
}

//---------------------------------------------------------------------------

inline int getTopPointID(t3DModel *pModel){
	float top = -FLT_MAX;
	int id = -1;
	for (int i = 0; i < pModel->numOfObjects; i++){
		t3DObject *pObject = &pModel->pObject[i];
			for (int j = 0; j < pObject->numOfVerts; j++){
			   CVector3 p = pObject->pVerts[j];
				   if(p.y > top){
					id = pObject->pVertID[j];
					top = p.y;
				   }
			   }
			}

	return id;
}

#pragma package(smart_init)


inline MeshSimplify::MeshSimplify(){
	g_pMesh = NULL;
	g_pProgMesh = NULL;
	g_edgemethod = PMesh::QUADRICTRI;
}	

inline void MeshSimplify::setInitMesh(MeshGraph * pMesh, t3DModel * pModel){
	delete g_pMesh;
	g_pMesh = NULL;

	g_pMesh = new Mesh();
	for (int i = 0; i < pMesh->numOfVertices; i++){

		vertex v(pMesh->pVerts[i].x, pMesh->pVerts[i].y, pMesh->pVerts[i].z);
		v.setIndex(i);

		g_pMesh->_vlist.push_back(v);
	}

	int offset = 0;
	for (int o = 0; o < pModel->numOfObjects; o++){
		t3DObject * pObject = &pModel->pObject[o];
		for (int i = 0; i < pObject->numOfFaces; i++){

			int v1, v2, v3;
			int nVerts;

			v1 = pMesh->indices[offset + pObject->pFaces[i].vertIndex[0]];
			v2 = pMesh->indices[offset + pObject->pFaces[i].vertIndex[1]];
			v3 = pMesh->indices[offset + pObject->pFaces[i].vertIndex[2]];

			triangle t(g_pMesh, v1, v2, v3);
			t.setIndex(i);

			g_pMesh->_plist.push_back(t); // push_back puts a *copy* of the element at the end of the list

			// update each vertex w/ its neighbors (vertrices & triangles)
			g_pMesh->_vlist[v1].addTriNeighbor(i);
			g_pMesh->_vlist[v1].addVertNeighbor(v2);
			g_pMesh->_vlist[v1].addVertNeighbor(v3);

			g_pMesh->_vlist[v2].addTriNeighbor(i);
			g_pMesh->_vlist[v2].addVertNeighbor(v1);
			g_pMesh->_vlist[v2].addVertNeighbor(v3);

			g_pMesh->_vlist[v3].addTriNeighbor(i);
			g_pMesh->_vlist[v3].addVertNeighbor(v1);
			g_pMesh->_vlist[v3].addVertNeighbor(v2);
		}
		
	}
	g_pMesh->setNumVerts(g_pMesh->_vlist.size());
	g_pMesh->setNumTriangles(g_pMesh->_plist.size());

	delete g_pProgMesh;
	g_pProgMesh = NULL;
	g_pProgMesh = new PMesh(g_pMesh, g_edgemethod);
}

inline void MeshSimplify::setMeshGraphVertices(MeshGraph * pMesh, MeshGraph * pMesh2){
	
	Mesh* g_pMeshNew = new Mesh();

	for (int i = 0; i < pMesh->numOfVertices; i++){

		vertex v(pMesh->pVerts[i].x, pMesh->pVerts[i].y, pMesh->pVerts[i].z);

		CVector3 v1 = pMesh->pVerts[i];
		CVector3 v2 = pMesh2->pVerts[i];

		if (Distance(v1, v2) > 1){
			int prob = 1;
		}

		v.setIndex(i);

		g_pMeshNew->_vlist.push_back(v);
	}

	/*if (pMesh->polygonIndices.size() == 0){
		for (int j=0; j<g_pProgMesh->numTris(); j++){

			triangle t;
			if (g_pProgMesh->getTri(j, t) && t.isActive()){
				const vertex& v1v = t.getVert1vertex();
				const vertex& v2v = t.getVert2vertex();
				const vertex& v3v = t.getVert3vertex();

				int v1 = v1v.getIndex();
				int v2 = v2v.getIndex();
				int v3 = v3v.getIndex();

				triangle t(g_pMeshNew, v1, v2, v3);
				t.setIndex(j);

				g_pMeshNew->_plist.push_back(t); // push_back puts a *copy* of the element at the end of the list

				// update each vertex w/ its neighbors (vertrices & triangles)
				g_pMeshNew->_vlist[v1].addTriNeighbor(j);
				g_pMeshNew->_vlist[v1].addVertNeighbor(v2);
				g_pMeshNew->_vlist[v1].addVertNeighbor(v3);

				g_pMeshNew->_vlist[v2].addTriNeighbor(j);
				g_pMeshNew->_vlist[v2].addVertNeighbor(v1);
				g_pMeshNew->_vlist[v2].addVertNeighbor(v3);

				g_pMeshNew->_vlist[v3].addTriNeighbor(j);
				g_pMeshNew->_vlist[v3].addVertNeighbor(v1);
				g_pMeshNew->_vlist[v3].addVertNeighbor(v2);

			}
		}
	} else {*/

		for (int j=0; j<pMesh->numOfFaces; j++){

			int v1 = pMesh->triangleIndices[j * 3];
			int v2 = pMesh->triangleIndices[j * 3 + 1];
			int v3 = pMesh->triangleIndices[j * 3 + 2];

			triangle t(g_pMeshNew, v1, v2, v3);
			t.setIndex(j);

			g_pMeshNew->_plist.push_back(t); // push_back puts a *copy* of the element at the end of the list

			// update each vertex w/ its neighbors (vertrices & triangles)
			g_pMeshNew->_vlist[v1].addTriNeighbor(j);
			g_pMeshNew->_vlist[v1].addVertNeighbor(v2);
			g_pMeshNew->_vlist[v1].addVertNeighbor(v3);

			g_pMeshNew->_vlist[v2].addTriNeighbor(j);
			g_pMeshNew->_vlist[v2].addVertNeighbor(v1);
			g_pMeshNew->_vlist[v2].addVertNeighbor(v3);

			g_pMeshNew->_vlist[v3].addTriNeighbor(j);
			g_pMeshNew->_vlist[v3].addVertNeighbor(v1);
			g_pMeshNew->_vlist[v3].addVertNeighbor(v2);

		}
	//}


	g_pMeshNew->setNumVerts(g_pMeshNew->_vlist.size());
	g_pMeshNew->setNumTriangles(g_pMeshNew->_plist.size());

	delete g_pMesh;
	g_pMesh = NULL;
	g_pMesh = g_pMeshNew;

	delete g_pProgMesh;
	g_pProgMesh = NULL;
	g_pProgMesh = new PMesh(g_pMesh, g_edgemethod);
}

inline void MeshSimplify::simplifyMesh(int numberOfCollapses){

	bool ret = true;
	for (int i = 0; i < numberOfCollapses; ++i) {
		ret = g_pProgMesh->collapseEdge();
	}
}

inline void MeshSimplify::getVerticesToMeshGraph(MeshGraph * pMeshOld, t3DModel * pModel){
	MeshGraph * pMesh = new MeshGraph();

	#ifdef _LOG
		Timerlog timerlog = Timerlog("createMeshGraph from simplified mesh");
		Log::log(LOG_LEVEL_METHODSTARTEND, "METHOD createMeshGraph STARTED");
	#endif

	vector<CVector3> v;
	vector<CVector3> vmulti;
	int num = 0;
	float area = 0;
	int numOfFaces = 0;
	float delta = 0.00001;

    int offset = 0;
	int joinIdx = 0;

	#ifdef _LOG
		log(LOG_LEVEL_METHODSTARTEND, "START createMeshGraph - vertices contruction");
	#endif

	int maxIndex = 0;
	float joining = 0.0f;

	for (int j=0; j<g_pProgMesh->numTris(); j++){
			
		triangle t;
		if (g_pProgMesh->getTri(j, t) && t.isActive()){
			const vertex& v1 = t.getVert1vertex();
			const vertex& v2 = t.getVert2vertex();
			const vertex& v3 = t.getVert3vertex();

			Vec3 v1pos = v1.getXYZ();
			Vec3 v2pos = v2.getXYZ();
			Vec3 v3pos = v3.getXYZ();

			CVector3 p1 = CVector3(v1pos.x, v1pos.y, v1pos.z);
			CVector3 p2 = CVector3(v2pos.x, v2pos.y, v2pos.z);
			CVector3 p3 = CVector3(v3pos.x, v3pos.y, v3pos.z);

			vmulti.push_back(p1);
			vmulti.push_back(p2);
			vmulti.push_back(p3);

			// avarage one ring area
			numOfFaces ++;
			float a = Magnitude(Cross(p2 - p1, p3 - p1)) / 2.0;
			area += a;
		}
	}			

	for (int i = 0; i < vmulti.size(); i++){
		bool add = true;
		for (int k = 0; k < v.size(); k++){
			if (Distance(vmulti[i], v[k]) < delta){
				add = false;
				pMesh->indices[i] = k;
				break;
			}
		}

		if (add){
			v.push_back(vmulti[i]);
			pMesh->indices[i] = v.size() - 1;
		}
	}

	//mean face area
	pMesh->numOfFaces = numOfFaces;
	pMesh->faceAreaSum = area;

	num = v.size();
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
	pMesh->wL = pMeshOld->wL;
	pMesh->wC = pMeshOld->wC;

	for (int i = 0; i < num; i++){
		pMesh->pVerts[i] = v[i];
		pMesh->wH[i] = pMeshOld->wH[i];
		pMesh->wHorig[i] = pMeshOld->wHorig[i];
	}

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	pMesh->E = Array2D< bool >(num, num, false);
	vmulti.clear();

	vector<int> pindices;
	for (int j=0; j<g_pProgMesh->numTris(); j++)
	{

		triangle t;
		if (g_pProgMesh->getTri(j, t) && t.isActive())
		{
			const vertex& v1 = t.getVert1vertex();
			const vertex& v2 = t.getVert2vertex();
			const vertex& v3 = t.getVert3vertex();

			Vec3 v1pos = v1.getXYZ();
			Vec3 v2pos = v2.getXYZ();
			Vec3 v3pos = v3.getXYZ();

			CVector3 p1 = CVector3(v1pos.x, v1pos.y, v1pos.z);
			CVector3 p2 = CVector3(v2pos.x, v2pos.y, v2pos.z);
			CVector3 p3 = CVector3(v3pos.x, v3pos.y, v3pos.z);

			vmulti.push_back(p1);
			int p1MeshIdx = pMesh->indices[vmulti.size() - 1];
			vmulti.push_back(p2);
			int p2MeshIdx = pMesh->indices[vmulti.size() - 1];
			vmulti.push_back(p3);
			int p3MeshIdx = pMesh->indices[vmulti.size() - 1];

			pindices.push_back(p1MeshIdx);
			pindices.push_back(p2MeshIdx);
			pindices.push_back(p3MeshIdx);

			pMesh->E[p1MeshIdx][p2MeshIdx] = true;
			pMesh->E[p1MeshIdx][p3MeshIdx] = true;

			pMesh->E[p2MeshIdx][p1MeshIdx] = true;
			pMesh->E[p2MeshIdx][p3MeshIdx] = true;

			pMesh->E[p3MeshIdx][p1MeshIdx] = true;
			pMesh->E[p3MeshIdx][p2MeshIdx] = true;
		}	
	}

	delete[] pMesh->triangleIndices;
	pMesh->triangleIndices = NULL;
	pMesh->triangleIndices = new int[pindices.size()];
	for (int i=0; i < pindices.size(); i++)
		pMesh->triangleIndices[i] = pindices[i];

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("create joining visualziation");
	#endif

	delete[] pMesh->origOneRingArea;
	pMesh->origOneRingArea = NULL;
	pMesh->origOneRingArea = new float[pMesh->numOfVertices];
	pMesh->origOneRingExtent = NULL;
	pMesh->origOneRingExtent = new float[pMesh->numOfVertices];

	calculateOneRingArea(pMesh, pModel, pMesh->origOneRingArea);
	
	#ifdef _LOG
		timerlog.addEnd();

		log(LOG_LEVEL_DUMP, "Matica E");
		log(LOG_LEVEL_DUMP, pMesh->E);

		log(LOG_LEVEL_METHODSTARTEND, "METHOD createMeshGraph ENDED");

		timerlog.logExecutionTimes();
	#endif

	copyMeshGraph(pMesh, pMeshOld);
}
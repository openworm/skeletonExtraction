//---------------------------------------------------------------------------
#include "lbse_extractor.h"

#define ISNAN(x) ((x) != (x))

lbse::Extractor::Extractor(){
	config_wA = 1.0f;
	config_wB = 0.1f;
	config_s_doBranchingSimplification = true;

	numOfIter = 3;
	mainComponentIndex = 0;
	stopContractionByIteration = true;
	stopContractionByRatio = false;
	groupingTolerance = 5;//config.GROUPING_TOLERANCE;
	sL = 3.0;//config.sL; 
	wL = 1.0;//config.wL; // 0.0025 before
	wH = 1.0;//config.wH;
	wC = 1.0;
	wA = 1.0;//config.wA;
	wB = 0.1;//config.wB;
	laplacianScheme = LaplacianScheme::LS_GLOBAL_JAMA_COTANGENT;
	groupingWithoutEdge = false;
	volumeTreshold = 1.0f / 100.0;//(float)config.VOLUME_THRESHOLD;
	maxBones = 24;//config.MAX_BONE_MAT;
	numOfBones = 24;//config.MAX_BONE_MAT;
	wantedNumOfBones = 24;//config.MAX_BONE_MAT;
	doBranchingSimplification = true;//config.s_doBranchingSimplification;
	doDisplacementShifting = true;//config.s_doDisplacementShifting;
	pSurgeryGraph = new SurgeryGraph();
	cyclicSkeleton = true;
	transferInteroperabilityMesh = true;
	isSDFvaluesComputed = false;
	useResolutionIndependencyTerm = true;
	doComputeSDF = false;
	useSDFBasedLaplacianWeights = false;
	useSDFBasedGroupingDistance = false;
	groupingToleranceSDFMulti = 1.0;
	doSimplifyMeshAfterIteration = false;
	doAddRandomNoiseAfterIteration = false;
	useSeparateWeigthsEachIterations = false;
	separateWeights = new Weights[5];
	pTriangulator = new PointCloudTriangulation::DeleunayTriangulator();
	pTriangulator->setKNeighParams(0.02, 8, 12);
	pTriangulator->setCenterFactorParams(0.2, 0.5, 2.0);
}

lbse::Extractor::~Extractor(){
	delete pSurgeryGraph;
	pSurgeryGraph = NULL;

	delete pTriangulator;
	pTriangulator = NULL;
}

//---------------------------------------------------------------------------
void lbse::Extractor::setsL(float _sL){
	sL = _sL;
}
void lbse::Extractor::setwL(float _wL){
	wL = _wL;
}
void lbse::Extractor::setwH(float _wH){
	wH = _wH;
	if (pMesh != NULL){
		for (int i = 0; i < pMesh->numOfVertices; i++)
			pMesh->wH[i]  = _wH;
	}
	if (originalMesh != NULL){
		for (int i = 0; i < originalMesh->numOfVertices; i++)
			originalMesh->wH[i]  = _wH;
	}
}
void lbse::Extractor::setwC(float _wC){
	wC = _wC;
}

//---------------------------------------------------------------------------
void lbse::Extractor::computeOptimalJoiningTolerance(int &numOfComponents, vector<int> &compMapping, float * thresholds){
	bool * mark = new bool[pMesh->numOfVertices];
	for (int i = 0; i < pMesh->numOfVertices; i++)
		mark[i] = false;
	vector< vector<int> > components;
	components.push_back(vector<int>());
	vector<int> queue;
	queue.push_back(0);	
	mark[0] = true;
	int componentIndex = 0;
	// get components
	while (queue.size() > 0){
		int index =  queue[queue.size() - 1];
		queue.pop_back();
		components[componentIndex].push_back(index);
		compMapping[index] = componentIndex;
		// add his neighbours
		for (int i = 0; i < pMesh->numOfVertices; i++){
			if (pMesh->E[index][i] && !mark[i]){
				queue.push_back(i);
				mark[i] = true;
			}
		}
		if (queue.size() == 0){
			// find first node in next component
			for (int i = 0; i < pMesh->numOfVertices; i++)
				if (!mark[i]){
					queue.push_back(i);
					componentIndex++;
					components.push_back(vector<int>());
					mark[i] = true;
					break;
				}
		}
	}

	numOfComponents = componentIndex + 1;

	if (componentIndex == 0){
		thresholds[0] = 0.0f;
	}

	for (int i = 0; i <componentIndex + 1; i++)
		thresholds[i] = FLT_MAX;
	// for each component find minimal joining to get him joined
	for (int i = 0; i <componentIndex + 1; i++)
		for (int j = i + 1; j <componentIndex + 1; j++){
			// find minmal joining between components i and j
			for (int x = 0; x < components[i].size(); x++)				
				for (int y = 0; y < components[j].size(); y++){
						float distance = Distance(pMesh->pVerts[components[i][x]], pMesh->pVerts[components[j][y]]);
						if(distance < thresholds[i])
							thresholds[i] = distance;
						if(distance < thresholds[j])
							thresholds[j] = distance;
				}
		}

	//for (int i = 0; i <compMapping.size(); i++){
	//	logg.log(LOG_LEVEL_DUMP, "compMapping: ", i);
	//	logg.log(LOG_LEVEL_DUMP, "value: ", compMapping[i]);
	//}

	for (int i = 0; i <componentIndex + 1; i++){
		#ifdef _LOG
			logg.log(LOG_LEVEL_C_PARAMS, "minimal joining for component: ", i);
			logg.log(LOG_LEVEL_C_PARAMS, "value: ", thresholds[i]);
		#endif
		thresholds[i] = thresholds[i] * 1.001f;
	}

}
//---------------------------------------------------------------------------
void lbse::Extractor::optimizeMeshGraphComponents(){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD optimizeMeshGraphComponents STARTED");
	#endif
	bool * mark = new bool[pMesh->numOfVertices];
	for (int i = 0; i < pMesh->numOfVertices; i++)
		mark[i] = false;
	vector< vector<int> > components;
	components.push_back(vector<int>());
	vector<int> queue;
	queue.push_back(0);	
	mark[0] = true;
	int componentIndex = 0;
	// get components
	while (queue.size() > 0){
		int index =  queue[0];
		queue.erase(queue.begin());
		components[componentIndex].push_back(index);
		// add his neighbours
		for (int i = 0; i < pMesh->numOfVertices; i++){
			if (pMesh->E[index][i] && !mark[i]){
				queue.push_back(i);
				mark[i] = true;
			}
		}
		if (queue.size() == 0){
			// find first node in next component
			for (int i = 0; i < pMesh->numOfVertices; i++)
				if (!mark[i]){
					queue.push_back(i);
					componentIndex++;
					components.push_back(vector<int>());
					mark[i] = true;
					break;
				}
		}
	}

	delete[] mark;

	// ak je iba 1 komponent, nic nemenime
	if (components.size() == 1)
		return;
	
	// zatial zobereme prvy component, casom by sa to malo dat nejako ovladat z gui alebo najvacsi brat
	int mainComponent = mainComponentIndex;
	if (mainComponent >= components.size())
		mainComponent = components.size() - 1;

	MeshGraph * optMesh = new MeshGraph();
	optMesh->numOfVertices = components[mainComponent].size();
	
	optMesh->wL = pMesh->wL;
	optMesh->wC = pMesh->wC;
	optMesh->faceAreaSum = pMesh->faceAreaSum;
	optMesh->numOfFaces = pMesh->numOfFaces;
	delete[] optMesh->wH;
	optMesh->wH = NULL;
	optMesh->wH = new float[optMesh->numOfVertices];
	delete[] optMesh->wHorig;
	optMesh->wHorig = NULL;
	optMesh->wHorig = new float[optMesh->numOfVertices];
	delete[] optMesh->origOneRingArea;
	optMesh->origOneRingArea = NULL;
	optMesh->origOneRingArea = new float[optMesh->numOfVertices];
	optMesh->origOneRingExtent = NULL;
	optMesh->origOneRingExtent = new float[optMesh->numOfVertices];
	delete[] optMesh->pVerts;
	optMesh->pVerts = NULL;
	optMesh->pVerts = new CVector3[optMesh->numOfVertices];
	optMesh->E = Array2D<bool>(optMesh->numOfVertices, optMesh->numOfVertices, false);

	for (int k = 0; k < pMesh->indices.size(); k++){
		optMesh->indices[k] = -1;
		for (int i = 0; i < optMesh->numOfVertices; i++)
			if (pMesh->indices[k] == components[mainComponent][i]){
				optMesh->indices[k] = i;
				//optMesh->inverseIndices[i] = pMesh->inverseIndices[i];
			}
		}

	for (int i = 0; i < optMesh->numOfVertices; i++) {
		for (int j = 0; j < optMesh->numOfVertices; j++) 
			optMesh->E[i][j] = pMesh->E[components[mainComponent][i]][components[mainComponent][j]];
		optMesh->wH[i] = pMesh->wH[components[mainComponent][i]];
		optMesh->wHorig[i] = pMesh->wHorig[components[mainComponent][i]];
		optMesh->origOneRingArea[i] = pMesh->origOneRingArea[components[mainComponent][i]];
		optMesh->origOneRingExtent[i] = pMesh->origOneRingExtent[components[mainComponent][i]];
		optMesh->pVerts[i] = pMesh->pVerts[components[mainComponent][i]];

	}

	// TODO: toto kopirovanie treba nejako solvnut pre viacej komponentov
	/*optMesh->triangleIndices = new int[pMesh->numOfFaces * 3];
	for (int i = 0; i < pMesh->numOfFaces * 3; i++) {
		optMesh->triangleIndices[i] = components[mainComponent][pMesh->triangleIndices[i]];
	}*/

	optMesh->numOfVBOlines = pMesh->numOfVBOlines;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "Mesh graph components have been optimized.");
		logg.log(LOG_LEVEL_METHODSTARTEND, "Old graph had vertices: ", pMesh->numOfVertices);
	#endif

	delete pMesh;
	pMesh = optMesh;

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "New graph has vertices: ", pMesh->numOfVertices);
		logg.log(LOG_LEVEL_DUMP, "Matica E");
		logg.log(LOG_LEVEL_DUMP, pMesh->E);
	#endif
}
//---------------------------------------------------------------------------
float lbse::Extractor::calculateVolumeFromMesh(MeshGraph * mesh, structure::t3DModel *pModel, int subdivision){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD calculateVolumeFromMesh STARTED");
	#endif

	float vol = 0.0f;
	/*float dx = (g_Load3DS.g_modelMax.x_max - g_Load3DS.g_modelMax.x_min) / (float)subdivision;
	float dy = (g_Load3DS.g_modelMax.y_max - g_Load3DS.g_modelMax.y_min) / (float)subdivision;
	float dz = (g_Load3DS.g_modelMax.z_max - g_Load3DS.g_modelMax.z_min) / (float)subdivision;
	float voxelVolume = dx * dy * dz;
	CVector3 * p = new CVector3[3];
	CVector3 * l = new CVector3[2];
	l[1] = CVector3(g_Load3DS.g_modelMax.x_max + 10.0f, g_Load3DS.g_modelMax.y_max + 10.0f, g_Load3DS.g_modelMax.z_max + 10.0f);
	for (int x = -subdivision/2; x < subdivision - subdivision/2; x++)
		for (int y = -subdivision/2; y < subdivision - subdivision/2; y++)
			for (int z = -subdivision/2; z < subdivision - subdivision/2; z++){
				int intersections = 0;
				//for every cell in octree structure, find center point
				l[0] = CVector3(dx * (x + 0.5f), dy * (y + 0.5f), dz * (z + 0.5f));

				int offset = 0;
				for (int i = 0; i < pModel->numOfObjects; i++){
					structure::t3DObject *pObject = &pModel->pObject[i];
					for(int j = 0; j < pObject->numOfFaces; j++){
						p[0] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[0]]];
						p[1] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[1]]];
						p[2] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[2]]];
						if (IntersectedPolygon(p, l, 3))
							intersections++;
					}
					offset += pObject->numOfVerts;
				}
					if (intersections % 2 == 1)
					vol += voxelVolume;
		}
    logg.log(LOG_LEVEL_METHODSTARTEND, "calculateVolumeFromMesh RETURNED ", vol);*/
	return vol;
}
//---------------------------------------------------------------------------
float lbse::Extractor::calculateVolumeFromMesh(MeshGraph * mesh, structure::t3DModel *pModel){
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD calculateVolumeFromMesh STARTED");
	#endif

	float vol = 0.0f;
	CVector3 * p = new CVector3[3];

	int offset = 0;
	for (int i = 0; i < pModel->numOfObjects; i++){
		structure::t3DObject *pObject = &pModel->pObject[i];
		for(int j = 0; j < pObject->numOfFaces; j++){
			p[0] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[0]]];
			p[1] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[1]]];
			p[2] = mesh->pVerts[mesh->indices[offset + pObject->pFaces[j].vertIndex[2]]];
			

			vol += Dot(p[0],Cross(p[1],p[2]));
		}
		offset += pObject->numOfVerts;
	}

	delete[] p;

	vol = vol / 6.0;

    #ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "calculateVolumeFromMesh RETURNED ", vol);
	#endif

	return vol;
}
//---------------------------------------------------------------------------
void lbse::Extractor::restoreMeshVolume(MeshGraph * mesh, structure::t3DModel *pModel){

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD calculateVolumeFromMesh STARTED");
	#endif

	float ratio = pow(origVolume / contractedVolume, (float)(1.0/3.0));
	CVector3 c = CVector3(pModel->modelbb.x_min, pModel->modelbb.y_min, pModel->modelbb.z_min);
	c = c + CVector3(pModel->modelbb.x_max - pModel->modelbb.x_min, pModel->modelbb.y_max - pModel->modelbb.y_min, pModel->modelbb.z_max - pModel->modelbb.z_min) / 2.0;

	int offset = 0;
	for (int i = 0; i < mesh->numOfVertices; i++){
			mesh->pVerts[i] = (mesh->pVerts[i] - c) * ratio + c;		
	}

}
//---------------------------------------------------------------------------
void lbse::Extractor::logContraction(MeshGraph * mesh1, MeshGraph * mesh2){
	#ifdef _LOG
	for (int i = 0; i < mesh1->numOfVertices; i++) {
		logg.log(LOG_LEVEL_C_PARAMS, "posun vertexu ", i);
		logg.log(LOG_LEVEL_C_PARAMS, "vektor: ", mesh1->pVerts[i] - mesh2->pVerts[i]);
	}
	#endif
}

int lbse::Extractor::calculateNumOfBones(SN::SkeletonNode * node){
	int sum = 1;
	for (int i=0; i < node->nodes.size(); i++)
		sum += calculateNumOfBones((SN::SkeletonNode*)node->nodes[i]);
	return sum;
}

bool lbse::Extractor::checkContractedMesh(MeshGraph * mesh){
	for (int i = 0; i < mesh->numOfVertices; i++) {
		if (ISNAN(mesh->pVerts[i].x) || mesh->pVerts[i].x > FLT_MAX || mesh->pVerts[i].x < -FLT_MAX)
			return false;
		if (ISNAN(mesh->pVerts[i].y) || mesh->pVerts[i].y > FLT_MAX || mesh->pVerts[i].y < -FLT_MAX)
			return false;
		if (ISNAN(mesh->pVerts[i].z) || mesh->pVerts[i].z > FLT_MAX || mesh->pVerts[i].z < -FLT_MAX)
			return false;
	}
	return true;
}

//---------------------------------------------------------------------------
void lbse::Extractor::computeSkeleton(structure::t3DModel *pModel, int sourcePointID, SN::SkeletonNode * node, int * ite, float modelMaxDim){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("computeSkeleton");
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD computeSkeleton STARTED");
	#endif

	numOfBones = maxBones;
	if (wantedNumOfBones < numOfBones)
		numOfBones = wantedNumOfBones;

	float * sdfValuesNormalized = NULL;
	float * sdfValues = NULL; 

	if (sdfSize > 0){	
		sdfValuesNormalized = new float[pMesh->numOfVertices];
		sdfValues = new float[pMesh->numOfVertices];
	}

	if (!isGeometryContracted){
		#ifdef _LOG
			timerlog.addStart("copy meshgraph");
		#endif

		copyMeshGraph(originalMesh, pMesh);

		//if (!isMeshGraphComputed)
		//	createMeshGraph(pModel, pMesh, NULL,  dmax /  joiningTolerance);
		//else

		for (int i=0; i<pMesh->numOfVertices; i++){
			vector<int> vecmap;
			vecmap.push_back(i);
			mgDegeneratesMapping[i] = vecmap;
		}

		#ifdef _LOG
			timerlog.addEnd();
		#endif
		//dumpDistances(pModel);
		MeshGraph * stepBack = new MeshGraph();

		origVolume = calculateVolumeFromMesh(pMesh, pModel);
		contractedVolume = origVolume;

		//float origVolume = 0.0f;
		if (stopContractionByRatio){
			#ifdef _LOG
				timerlog.addStart("calculate volume");
			#endif
			//origVolume = calculateVolumeFromMesh(pMesh, pModel, config.VOLUME_SUBDIVISION);
			#ifdef _LOG
				timerlog.addEnd();
			#endif
		}
		// now we have to contract the mesh
		//dumpMesh(pMesh);
		//float curVolume = origVolume;
		bool goBack = false;
		*ite = 1;

		vector<std::set<int>> globalNeighbourhoods = pTriangulator->computeGlobalNeighbourhood(MG2PCTMG(pMesh));


		if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_POINTCLOUD){
			calculateOneRingExtent(pMesh, pMesh->origOneRingExtent, globalNeighbourhoods);
		}

		float avarageCurOneRingArea = 0.0f;
		float avarageOrigOneRingArea = 0.0f;
		float avarageCurOneRingExtent = 0.0f;
		float avarageOrigOneRingExtent = 0.0f;

		float * curOneRingArea = new float[pMesh->numOfVertices];
		float * curOneRingExtent = new float[pMesh->numOfVertices];

		for (int i=0; i < pMesh->numOfVertices; i++){
			curOneRingArea[i] = pMesh->origOneRingArea[i];
			avarageCurOneRingArea += pMesh->origOneRingArea[i];
			avarageOrigOneRingArea += pMesh->origOneRingArea[i];

			curOneRingExtent[i] = pMesh->origOneRingExtent[i];
			avarageCurOneRingExtent += pMesh->origOneRingExtent[i];
			avarageOrigOneRingExtent += pMesh->origOneRingExtent[i];
		}
		avarageCurOneRingExtent /= pMesh->numOfVertices;
		avarageOrigOneRingExtent /= pMesh->numOfVertices;

		avarageCurOneRingArea /= pMesh->numOfVertices;
		avarageOrigOneRingArea /= pMesh->numOfVertices;

		bool isOverThreshold = true;

		if (useSeparateWeigthsEachIterations){
			wL = separateWeights[0].wL;
			wH = separateWeights[0].wH;
			wC = separateWeights[0].wC;
		}

		if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_POINTCLOUD){
			//pMesh->wL = wL * (1.0 / (5.0 * avarageCurOneRingExtent));
			pMesh->wL = wL * (0.002 * avarageCurOneRingExtent);
		}

		if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_COTANGENT || laplacianScheme == LaplacianScheme::LS_GLOBAL_VCL_COTANGENT ||
			laplacianScheme == LaplacianScheme::LS_LOCAL_JAMA_COTANGENT || laplacianScheme == LaplacianScheme::LS_LOCAL_OCL_COTANGENT || 
			laplacianScheme == LaplacianScheme::LS_GLOBAL_VCLLSM_COTANGENT || laplacianScheme == LaplacianScheme::LS_GLOBAL_CPU_COTANGENT_PCL ||
			laplacianScheme == LaplacianScheme::LS_LOCAL_OCL_QR_INTEROP_COTANGENT || laplacianScheme == LaplacianScheme::LS_GLOBAL_VCLLSMSDF_COTANGENT || LaplacianScheme::LS_LOCAL_JAMASDF_COTANGENT){
				pMesh->wL = wL * (0.001 * sqrt(avarageCurOneRingArea));
		}

		if (useResolutionIndependencyTerm)
			pMesh->wL = pMesh->wL * pMesh->numOfVertices;
			//pMesh->wL = pMesh->wL * pMesh->numOfFaces;

		float lastAvarageCurOneRingExtent = FLT_MAX;

		pMesh->wC = wC;

		int kneigh = pTriangulator->getKNeigh(MG2PCTMG(pMesh));

		//////////////////// SDF ///////////////

		if (sdfSize > 0){
			sdfHalfVectorsMG = new CVector3[pMesh->numOfVertices];
		}

		////////////////////////////////////////	
		//g_MeshSimplify.setInitMesh(pMesh, pModel);

		if (useSDFBasedLaplacianWeights){
			for (int i = 0; i < pMesh->numOfVertices; i++){
				float bias = 0.01;
				float changeRatio = sdfValuesNormalizedMG[i] * (1.0 - bias) + bias;
				pMesh->wH[i] = pMesh->wH[i] / changeRatio;
			}
		}


		if (laplacianScheme == LaplacianScheme::LS_LOCAL_OCL_QR_INTEROP_COTANGENT){
			// tato metoda riesi Laplacian iba vo one-rin areas a rata to paralelne na celom meshi, iteracie su vizualizaovane cez interoperability -> nekonverguje
			//contractMeshGraphParallelOpenCLInterop(ite, pMeshOpenCL, mgDegeneratesMapping, sL, curOneRingArea, numOfIter, openCLContext, openCLManager);
			contractedVolume = calculateVolumeFromMesh(pMeshOpenCL, pModel);
			//copyMeshGraph(pMesh, pMeshLastIteration);
		} else if (laplacianScheme == LaplacianScheme::LS_GLOBAL_OCL_JACOBI_INTEROP_COTANGENT){
			 // tato metoda paralelizuje jednu iteraciu kontrakcie na celom mesi, vie to paralelne zratat pre kazdy vertex, iteracie su vizualizaovane cez interoperability -> nekonverguje 
		  	 //contractMeshGraphParallelOpenCLJacobiInterop(ite, pMeshOpenCL, mgDegeneratesMapping, sL, curOneRingArea, numOfIter, openCLContext, openCLManager);
			} 
		else {
			while ((*ite <= numOfIter || !stopContractionByIteration) && (isOverThreshold || !stopContractionByRatio)){
				//if (curVolume > origVolume * 0.006)
				//copyMeshGraph(pMesh, stepBack);

				#ifdef _LOG
					char * s = new char[255];
					int iternum = (*ite);
					sprintf_s ( s, 255, "contraction %i", iternum);

					logg.log(LOG_LEVEL_NOTE, "Po Contraction ITERATION : ", iternum);
					timerlog.addStart(s);
				#endif


				// add random noise each iteration of local Laplacian
				if (laplacianScheme == LaplacianScheme::LS_LOCAL_OCL_COTANGENT && doAddRandomNoiseAfterIteration){
					addRandomNoiseToMeshGraph(pMesh, pModel, 0.01);
				}

				copyMeshGraph(pMesh, stepBack);

				float maxSDF = FLT_MIN;
				float minSDF = FLT_MAX;

				if (doComputeSDF){
					// calculate sdf for contracted mesh graph positions
					calculateSDFForMeshGraph(pMesh, pModel, sdfValuesNormalizedMG, &(sdfHalfVectors), sdfHalfVectorsMG);

					//gldAddModelData("test model", sdfValuesNormalizedMG);

				}

				contractMeshGraph(laplacianScheme, pMesh, mgDegeneratesMapping, *ite ,LImage, sL, curOneRingExtent, curOneRingArea, kneigh, globalNeighbourhoods, openCLContext, sdfHalfVectorsMG, pTriangulator, openCLManager); 

				
				if (doSimplifyMeshAfterIteration){
					//if (*ite == 1){
						
					//} else {
						g_MeshSimplify.setMeshGraphVertices(pMesh, stepBack);
					//}
					g_MeshSimplify.simplifyMesh(pMesh->numOfFaces / 8);
					g_MeshSimplify.getVerticesToMeshGraph(pMesh, pModel);
				}

				if (useSeparateWeigthsEachIterations && *ite <= 5){

					pMesh->wL = separateWeights[*ite - 1].wL * (0.001 * sqrt(avarageCurOneRingArea)) * pow((float)sL, (float)*ite);
					for (int i = 0; i < pMesh->numOfVertices; i++)
						pMesh->wH[i] = separateWeights[*ite - 1].wH * sqrt(pMesh->origOneRingArea[i] / curOneRingArea[i]);
					pMesh->wC = separateWeights[*ite - 1].wC;

				} else {

					if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_COTANGENT || laplacianScheme == LaplacianScheme::LS_GLOBAL_VCL_COTANGENT || 
						laplacianScheme == LaplacianScheme::LS_LOCAL_JAMA_COTANGENT || laplacianScheme == LaplacianScheme::LS_LOCAL_OCL_COTANGENT || 
						laplacianScheme == LaplacianScheme::LS_GLOBAL_VCLLSM_COTANGENT || laplacianScheme == LaplacianScheme::LS_GLOBAL_VCLLSMSDF_COTANGENT || LaplacianScheme::LS_LOCAL_JAMASDF_COTANGENT){

						calculateOneRingArea(pMesh, curOneRingArea);
						pMesh->wL = pMesh->wL * sL;

						if (useSDFBasedLaplacianWeights){
							for (int i = 0; i < pMesh->numOfVertices; i++){
								pMesh->wH[i] = pMesh->wHorig[i] * sqrt( pMesh->origOneRingArea[i] / curOneRingArea[i] );
								float bias = 0.01;
								float changeRatio = sdfValuesNormalizedMG[i] * (1.0 - bias) + bias;
								pMesh->wH[i] = pMesh->wH[i] / changeRatio;
							}
						} else {
							for (int i = 0; i < pMesh->numOfVertices; i++){
								if (curOneRingArea[i] > 0)
									pMesh->wH[i] = pMesh->wHorig[i] * sqrt( pMesh->origOneRingArea[i] / curOneRingArea[i] );
								else
									pMesh->wH[i] = FLT_MAX;
							}
						}
					}

					if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_POINTCLOUD){
						calculateOneRingExtent(pMesh, curOneRingExtent, globalNeighbourhoods);
							pMesh->wL = pMesh->wL * sL;
							for (int i = 0; i < pMesh->numOfVertices; i++){
								pMesh->wH[i] = pMesh->wHorig[i] * (pMesh->origOneRingExtent[i] / curOneRingExtent[i]);
							}
					}
				}

				//copyMeshGraph(pMesh, pMeshLastIteration);
				//char strBuffer[255] = {0};
				//sprintf(strBuffer, " %i", *ite);
				//serializeMeshGraph((char*)(config.projectDir + "\\MeshGraphs\\contractedMeshGraph_iteration_"+string(strBuffer)+".cmg").c_str());

				//dumpMesh(pMesh);
				//logContraction(pMesh, stepBack);
				if (stopContractionByRatio){
					lastAvarageCurOneRingExtent = avarageCurOneRingExtent;
					avarageCurOneRingExtent = 0.0f;
					for (int i=0; i < pMesh->numOfVertices; i++){
						avarageCurOneRingArea += curOneRingArea[i];
						avarageCurOneRingExtent += curOneRingExtent[i];
					}
					avarageCurOneRingArea /= pMesh->numOfVertices;
					avarageCurOneRingExtent /= pMesh->numOfVertices;

					if (laplacianScheme == LaplacianScheme::LS_GLOBAL_JAMA_POINTCLOUD){
						isOverThreshold = (lastAvarageCurOneRingExtent - avarageCurOneRingExtent) / avarageOrigOneRingExtent >= volumeTreshold;
					} else {
						isOverThreshold = avarageCurOneRingArea / avarageOrigOneRingArea >= volumeTreshold;
					}
				}
					//curVolume = calculateVolumeFromMesh(pMesh, pModel, config.VOLUME_SUBDIVISION);

				//if (!checkContractedMesh(pMesh)){
				//	goBack = true;
				//	break;
				//}
				(*ite)++;
				#ifdef _LOG
					timerlog.addEnd();
				#endif
			}
		}

		delete[] curOneRingArea;
		delete[] curOneRingExtent;

		/*if (goBack){
			logg.log(LOG_LEVEL_NOTE, "STEPING BACK THE MESH !!!");
			copyMeshGraph(stepBack, pMesh);
			dumpMesh(pMesh);
		}*/

		//delete stepBack;
		//stepBack = NULL;

		/*
		BoneInfo info = BoneInfo();
		BonesMatrices matrices = BonesMatrices();
		info.pBoneMatrices.push_back(matrices);

		//compute normals of contracted graph for visualization
		vector<CVector3> normals;
		CVector3 * vPoly = new CVector3[3];

		int offset = 0;

		for (int i = 0; i < pModel->numOfObjects; i++){
		structure::t3DObject * pObject = &pModel->pObject[i];
		for (int j = 0; j < pObject->numOfFaces; j++){

		vPoly[0] = pMesh->pVerts[pMesh->indices[offset + pObject->pFaces[j].vertIndex[0]]];
		vPoly[1] = pMesh->pVerts[pMesh->indices[offset + pObject->pFaces[j].vertIndex[1]]];
		vPoly[2] = pMesh->pVerts[pMesh->indices[offset + pObject->pFaces[j].vertIndex[2]]];

				CVector3 vVector1 = vPoly[0] - vPoly[2];
				CVector3 vVector2 = vPoly[2] - vPoly[1];

				CVector3 vNormal  = Cross(vVector1, vVector2);
				vNormal  = Normalize(vNormal);
				normals.push_back(vNormal);
			}
			offset += pObject->numOfVerts;
		}*/

		//pMesh->pNormals = new CVector3[normals.size()];
		//for (int i = 0; i < normals.size(); i++) {
		//	pMesh->pNormals[i] = normals[i];
		//}

		//logg.log(0, "Contracted mesh graph has been serialized into file contractedMeshGraph_last.cmg");
		//serializeMeshGraph((char*)(config.projectDir + "\\MeshGraphs\\contractedMeshGraph_last.cmg").c_str());

	}

	isGeometryContracted = true;
	#ifdef _LOG
		timerlog.addStart("connectivity surgery");
	#endif
	applyConnectivitySurgery(true, node, modelMaxDim);
	#ifdef _LOG
		timerlog.addEnd();
	#endif

	isSkeletonComputed = true;

	#ifdef _LOG
		timerlog.logExecutionTimes();
	#endif

	if (sdfSize > 0){
		delete[] sdfHalfVectorsMG;
		delete[] sdfValuesNormalized;
		delete[] sdfValues;
	}
}

void lbse::Extractor::meshgraphVertexPositionsToModel(MeshGraph * pMesh, structure::t3DModel * pModel){
	int offset=0;
	for (int i = 0; i < pModel->numOfObjects; i++){
		structure::t3DObject * pObject = &pModel->pObject[i];
		for (int j = 0; j < pObject->numOfFaces; j++){
			int idx0 = offset + pObject->pFaces[j].vertIndex[0];
			int idx1 = offset + pObject->pFaces[j].vertIndex[1];
			int idx2 = offset + pObject->pFaces[j].vertIndex[2];
			pObject->pVerts[pObject->pFaces[j].vertIndex[0]] = pMesh->pVerts[pMesh->indices[idx0]];
			pObject->pVerts[pObject->pFaces[j].vertIndex[1]] = pMesh->pVerts[pMesh->indices[idx1]];
			pObject->pVerts[pObject->pFaces[j].vertIndex[2]] = pMesh->pVerts[pMesh->indices[idx2]];
		}
		offset+= pObject->numOfVertices;
	}
}

void lbse::Extractor::applyConnectivitySurgery(bool applyLBSEPostprocessing, SN::SkeletonNode * node, float modelMaxDim)
{
	// connectivity surgery
	delete pSurgeryGraph;
	pSurgeryGraph = NULL;
	pSurgeryGraph = new SurgeryGraph();

	if (applyLBSEPostprocessing){
		createSurgeryGraphFromMeshGraph(pMesh, pSurgeryGraph);
	} else {
		createSurgeryGraphFromMeshGraph(polyMesh, pSurgeryGraph);
	}

	float threshold = modelMaxDim / (100.0f/groupingTolerance);
	createSkeletonFromSurgeryGraph(applyLBSEPostprocessing, pSurgeryGraph, originalMesh->pVerts, mgDegeneratesMapping, node, &numOfBones, threshold, groupingToleranceSDFMulti, doBranchingSimplification, doDisplacementShifting, groupingWithoutEdge, cyclicSkeleton, sdfHalfVectorsMG, useSDFBasedGroupingDistance);

    currentNumberOfSkeletonBones = generateIdForTree(node, skeletonMeshGraphIndices);

}

/*void lbse::Extractor::mergedMGToSkeleton(SN::SkeletonNode * node, float modelMaxDim){

	// connectivity surgery
	delete pSurgeryGraph;
	pSurgeryGraph = NULL;
	pSurgeryGraph = new SurgeryGraph();

	createSurgeryGraphFromMeshGraph(polyMesh, pSurgeryGraph);

	float threshold2 = modelMaxDim / (100.0f/groupingTolerance);
	createSkeletonFromSurgeryGraphSDFTest(pSurgeryGraph, originalMesh->pVerts, mgDegeneratesMapping, node, &numOfBones, threshold2, groupingToleranceSDFMulti, doBranchingSimplification, doDisplacementShifting, groupingWithoutEdge, cyclicSkeleton, sdfHalfVectorsMG, useSDFBasedGroupingDistance);

	currentNumberOfSkeletonBones = generateIdForTree(node, skeletonMeshGraphIndices);
}*/

void lbse::Extractor::calculateSDFForMeshGraph(MeshGraph * pMesh, structure::t3DModel * pModel, float * sdfValuesNormalizedMG, CVector3 ** sdfHalfVectors, CVector3 * sdfHalfVectorsMG){

	float * newpos = new float[pModel->numOfVerts * 3];

	getMGPositionVector(pMesh, pModel, newpos);

	sdfModelController->SetNewPositions(newpos);
	int size = 0;

	sdfModelController->ComputeSDF();
	float * sdf = sdfModelController->GetSDF(size, true); 
	float maxSDF = FLT_MIN;
	float minSDF = FLT_MAX;
	sdfSize = size;
	*(sdfHalfVectors) = new CVector3[size];
	float * sdfValuesNormalized = new float[size];

	for (int i=0; i < size; i++){
		(*sdfHalfVectors)[i] = pModel->pObject[0].pNormals[i] * sdf[i] / -2.0;

		if ((sdf[i]) > maxSDF){
			maxSDF = sdf[i];
		}
		if ((sdf[i]) < minSDF){
			minSDF = sdf[i];
		}
	}

	for (int i=0; i < size; i++){
		sdfValuesNormalized[i] =  (sdf[i] - minSDF) / (maxSDF - minSDF); 
	}

	vector<CVector3> mgsdfvec;

	recomputeModelValuesToMeshgraph<CVector3>(pMesh, pModel, *sdfHalfVectors, sdfHalfVectorsMG);

	recomputeModelValuesToMeshgraph<float>(pMesh, pModel, sdfValuesNormalized, sdfValuesNormalizedMG);

	for (int i=0; i < pMesh->numOfVertices; i++){
		mgsdfvec.push_back(sdfHalfVectorsMG[i]);
	}

	isSDFvaluesComputed = true;

	delete[] sdfValuesNormalized;
	delete[] sdf;
	delete[] newpos;
}

void lbse::Extractor::addRandomNoiseToMeshGraph(MeshGraph * pMesh, structure::t3DModel * pModel, float scale){

	srand (time(NULL));

	int offset=0;
	for (int i = 0; i < pModel->numOfObjects; i++){
		structure::t3DObject * pObject = &pModel->pObject[i];
		for (int j = 0; j < pObject->numOfFaces; j++){
			int idx0 = offset + pObject->pFaces[j].vertIndex[0];
			int idx1 = offset + pObject->pFaces[j].vertIndex[1];
			int idx2 = offset + pObject->pFaces[j].vertIndex[2];

			CVector3 rand0 = pObject->pNormals[idx0] * (((float)(100 - rand() % 200) / 100.0) * adaptVMdmax * scale);
			CVector3 rand1 = pObject->pNormals[idx1] * (((float)(100 - rand() % 200) / 100.0) * adaptVMdmax * scale);
			CVector3 rand2 = pObject->pNormals[idx2] * (((float)(100 - rand() % 200) / 100.0) * adaptVMdmax * scale);

			pMesh->pVerts[pMesh->indices[idx0]] = pMesh->pVerts[pMesh->indices[idx0]] + rand0;
			pMesh->pVerts[pMesh->indices[idx1]] = pMesh->pVerts[pMesh->indices[idx1]] + rand1;
			pMesh->pVerts[pMesh->indices[idx2]] = pMesh->pVerts[pMesh->indices[idx2]] + rand2;
		}
		offset+= pObject->numOfVertices;
	}
}

//---------------------------------------------------------------------------
template<typename T>
void lbse::Extractor::recomputeModelValuesToMeshgraph(MeshGraph * pMesh, structure::t3DModel *pModel, T * sdfValues, T * sdfvec){
	int maxMeshIdx = 0;
	int maxModelIdx = 0;

	int offset = 0;
	for(int i = 0; i < pModel->numOfObjects; i++)
	{
		if(pModel->pObject.size() <= 0) break;
		structure::t3DObject *pObject = &(pModel->pObject[i]);

		int length = pObject->numOfVertices;

		for(int j = 0; j < pObject->numOfFaces; j++){
			for(int whichVertex = 0; whichVertex < 3; whichVertex++){
				int modelIdx = pObject->pFaces[j].vertIndex[whichVertex] + offset;
				int meshIdx = pMesh->indices[modelIdx];

				if (modelIdx > maxModelIdx)
					maxModelIdx = modelIdx;
				if (meshIdx > maxMeshIdx)
					maxMeshIdx = meshIdx;

				sdfvec[meshIdx] = sdfValues[modelIdx];
			}
		}

		offset += pObject->numOfVerts;
	}

}

//---------------------------------------------------------------------------

void lbse::Extractor::getMGPositionVector(MeshGraph * pMesh, structure::t3DModel *pModel, float * newpos){

	int maxMeshIdx = 0;
	int maxModelIdx = 0;

	int offset = 0;
	for(int i = 0; i < pModel->numOfObjects; i++)
	{
		if(pModel->pObject.size() <= 0) break;
		structure::t3DObject *pObject = &(pModel->pObject[i]);

		int length = pObject->numOfVertices;

		for(int j = 0; j < pObject->numOfFaces; j++){
			for(int whichVertex = 0; whichVertex < 3; whichVertex++){
				int modelIdx = pObject->pFaces[j].vertIndex[whichVertex] + offset;
				int meshIdx = pMesh->indices[modelIdx];

				MathStructures::Vector4 original = MathStructures::Vector4(pMesh->pVerts[meshIdx].x,pMesh->pVerts[meshIdx].y, pMesh->pVerts[meshIdx].z, 1.0);
				MathStructures::Vector4 transformed = original * pModel->rootTrans;

				//Vector4 transformed = Vector4(pMesh->pVerts[meshIdx].x, pMesh->pVerts[meshIdx].z, -pMesh->pVerts[meshIdx].y, 1.0);
				//Vector4 transformed = Vector4(pMesh->pVerts[meshIdx].x, pMesh->pVerts[meshIdx].y, pMesh->pVerts[meshIdx].z, 1.0);

				newpos[modelIdx * 3] = transformed.X;
				newpos[modelIdx * 3 + 1] = transformed.Y;
				newpos[modelIdx * 3 + 2] = transformed.Z;

				if (modelIdx > maxModelIdx)
					maxModelIdx = modelIdx;
				if (meshIdx > maxMeshIdx)
					maxMeshIdx = meshIdx;
			}
		}

		offset += pObject->numOfVerts;
	}

}

SN::SkeletonNode * lbse::Extractor::rerootSkeleton(int newrootId, SN::SkeletonNode* pSkelet){
	vector<SN::SkeletonNode*> queue;
	queue.push_back(pSkelet);
	SN::SkeletonNode * imaginary = pSkelet->father;
	pSkelet->father = NULL;
	SN::SkeletonNode * newroot = NULL;

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		if (pNode->id == newrootId){
			newroot = pNode;
			queue.clear();
			if (pNode->father != NULL)
				queue.push_back(pNode);
			break;
		}
		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = pNode->nodes[i];
			queue.push_back(pSon);
		}
	}

	SN::SkeletonNode * lastSon = NULL;

	while (queue.size() > 0){

		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		// remove node from fathers nodes 
		for (int i=0; i < pNode->father->nodes.size(); i++){
			if (pNode->father->nodes[i] == pNode)
				pNode->father->nodes.erase(pNode->father->nodes.begin()+i);
		}

		// add father as child
		pNode->nodes.push_back(pNode->father);

		if (pNode->father->father != NULL){
			queue.push_back(pNode->father);
			lastSon = pNode;
		} else {
			// set updated father
			pNode->nodes[pNode->nodes.size() - 1]->father = pNode;
		}
		// set updated father
		if (lastSon != NULL)
			pNode->father = lastSon;
	}

	copySkeletonNode(newroot, pSkelet);
	pSkelet->father = imaginary;

	return pSkelet;
}


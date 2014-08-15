//---------------------------------------------------------------------------
#include "lbse_iterationSolver.h"

//---------------------------------------------------------------------------

void contractMeshGraph(int laplacianScheme, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingExtent, float * curOneRingArea, int kneigh, vector<std::set<int>> globalNeighbourhoods, OpenCLContext oclc, CVector3 * centerPoints, PointCloudTriangulation::DeleunayTriangulator * pTriangulator, OpenCLManager openCLManager){ //ModelController::CModel * sdfController
	switch(laplacianScheme){

	case LS_GLOBAL_JAMA_COTANGENT: 
		contractMeshGraphCPUCotangent(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea); 
		break;
	case LS_GLOBAL_JAMA_POINTCLOUD:
		contractMeshGraphCPUPointCloud(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingExtent, curOneRingArea, kneigh, globalNeighbourhoods, pTriangulator); 
		break;
	case LS_GLOBAL_VCL_COTANGENT: 
		contractMeshGraphGPUVCL(pMesh,mgDegeneratesMapping, it, LImage, sL, curOneRingArea); 
		break;
	case LS_LOCAL_JAMA_COTANGENT: 
		contractMeshGraphParallelCPU(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea); 
		break;
	case LS_LOCAL_OCL_COTANGENT:
		contractMeshGraphParallelOpenCL(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea, openCLManager); 
		break;
	case LS_GLOBAL_VCLLSM_COTANGENT: 
		contractMeshGraphGPUVCL_LSM(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea); 
		break;
	case LS_GLOBAL_CPU_COTANGENT_PCL: 
		contractMeshGraphCPUPCL(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea); 
		break;
	case LS_GLOBAL_VCLLSMSDF_COTANGENT:
		contractMeshGraphGPUVCL_LSM_SDF(pMesh, centerPoints, mgDegeneratesMapping, it, LImage, sL, curOneRingArea);				
		break;	
	case LS_LOCAL_JAMASDF_COTANGENT:
		contractMeshGraphParallelCPU_SDF(pMesh, centerPoints, mgDegeneratesMapping, it, LImage, sL, curOneRingArea);				
		break;	
	case LS_GLOBAL_OCL_JACOBI_COTANGENT:
		contractMeshGraphParallelOpenCLJacobi(pMesh, mgDegeneratesMapping, sL, curOneRingArea, oclc, openCLManager);
		break;
	default: contractMeshGraphGPUVCL_LSM(pMesh, mgDegeneratesMapping, it, LImage, sL, curOneRingArea);
	}
}

//---------------------------------------------------------------------------

void collapseDegenerates(MeshGraph * pMesh,  boost::unordered_map<int, vector<int> > &mgDegeneratesMapping){
	float *a = new float[2];
	int *idcs = new int[2];

	bool degenerates = false;

	do {
		degenerates = false;
		for (int i = 0; i < pMesh->numOfVertices && !degenerates; i++)
			for (int j = 0; j < pMesh->numOfVertices && !degenerates; j++)
				if (pMesh->E[i][j] && i != j){

					int ind = 0;
					for (int k = 0; k < pMesh->numOfVertices; k++){
						if (pMesh->E[i][k] && pMesh->E[j][k] && i != k && j != k && isMeshTriangle(pMesh, i,j,k)){
							CVector3 v1 = pMesh->pVerts[k] - pMesh->pVerts[j];
							CVector3 v2 = pMesh->pVerts[k] - pMesh->pVerts[i];

							if (ind > 1) {
								#ifdef _LOG
									logg.log(LOG_LEVEL_WARNING, "Vrchol narusajuci 2D manifold pri DEGENERATES: " ,k);
									logg.log(LOG_LEVEL_WARNING, "ind je: " ,ind);
								#endif
							} else {
								idcs[ind] = k;
								a[ind] = AngleBetweenVectors(v1, v2);
								ind++;
							}
						}
					}
					if (ind == 2){
						CVector3 center;
						if (cotan(a[0]) > FLT_MAX || cotan(a[0])< -FLT_MAX){
							//pMesh->pVerts[idcs[0]] = (pMesh->pVerts[idcs[0]] + pMesh->pVerts[i] + pMesh->pVerts[j]) / 3;
							//collapseTriangle(pMesh, mgDegeneratesMapping, i, j, idcs[0]);
							float a = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[j]);
							float b = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[idcs[0]]);
							float c = Magnitude(pMesh->pVerts[j] - pMesh->pVerts[idcs[0]]);

							if (a < b)
								if (a < c)
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, i, j);
								else
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, j, idcs[0]);
							else
								if (b < c)
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, i, idcs[0]);
								else
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, j, idcs[0]);

							degenerates = true;
							break;
						}
						if (cotan(a[1]) > FLT_MAX || cotan(a[1])< -FLT_MAX){
							//pMesh->pVerts[idcs[1]] = (pMesh->pVerts[idcs[1]] + pMesh->pVerts[i] + pMesh->pVerts[j]) / 3;
							//collapseTriangle(pMesh, mgDegeneratesMapping, i, j, idcs[1]);
							float a = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[j]);
							float b = Magnitude(pMesh->pVerts[i] - pMesh->pVerts[idcs[1]]);
							float c = Magnitude(pMesh->pVerts[j] - pMesh->pVerts[idcs[1]]);

							if (a < b)
								if (a < c)
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, i, j);
								else
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, j, idcs[1]);
							else
								if (b < c)
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, i, idcs[1]);
								else
									halfEdgeCollapse(pMesh, mgDegeneratesMapping, j, idcs[1]);

							degenerates = true;
							break;
						}

					} else {
						//logg.log(LOG_LEVEL_WARNING, "ind nieje 2 pri DEGENERATES : ", ind);
					}
				}

	} while (degenerates && pMesh->numOfVertices > 2);

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(0, "METHOD contractMeshGraph RETURNED: pMesh->numOfVertices < 3");
		#endif

		return;
	}

	delete[] a;
	delete[] idcs;
};

//---------------------------------------------------------------------------

Array2D< float > calculateLaplacianMatrix(MeshGraph * pMesh){

	float *a = new float[2];

	Array2D< float >  L = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	int pocetTroj = 0;
	int pocetL = 0;

	for (int i = 0; i < pMesh->numOfVertices; i++)
		for (int j = 0; j < pMesh->numOfVertices; j++)
			if (pMesh->E[i][j] && i != j){

				int ind = 0;
				for (int k = 0; k < pMesh->numOfVertices; k++){
					if (pMesh->E[i][k] && pMesh->E[j][k] && i != k && j != k && isMeshTriangle(pMesh, i,j,k)){
						pocetTroj++;
						CVector3 v1 = pMesh->pVerts[k] - pMesh->pVerts[j];
						CVector3 v2 = pMesh->pVerts[k] - pMesh->pVerts[i];

						if (ind > 1) {
							#ifdef _LOG
								logg.log(LOG_LEVEL_WARNING, "Vrchol narusajuci 2 manifold: " ,k);
							#endif
						} else
							a[ind++] = AngleBetweenVectors(v1, v2);
					}
				}
				if (ind == 2){
					//L[i][j] = 1.0f;
					pocetL++;
					float val = cotan(a[0]) + cotan(a[1]);
					L[i][j] = val;

					if (L[i][j] < -FLT_MAX) {
						L[i][j] = -FLT_MAX;

						#ifdef _LOG
							logg.log(LOG_LEVEL_WARNING, "L bolo mensie ako -float");
							logg.log(LOG_LEVEL_WARNING, "alfa : " ,a[0]);
							logg.log(LOG_LEVEL_WARNING, "beta : " ,a[1]);
							logg.log(LOG_LEVEL_WARNING, "i: " ,i);
							logg.log(LOG_LEVEL_WARNING, "j: " ,j);
						#endif
					} else if (L[i][j] > FLT_MAX) {
						L[i][j] = FLT_MAX;
						#ifdef _LOG
							logg.log(LOG_LEVEL_WARNING, "L bolo vacsie ako float");
							logg.log(LOG_LEVEL_WARNING, "alfa : " ,a[0]);
							logg.log(LOG_LEVEL_WARNING, "beta : " ,a[1]);
							logg.log(LOG_LEVEL_WARNING, "i: " ,i);
							logg.log(LOG_LEVEL_WARNING, "j: " ,j);
						#endif
					}
				} 
			}

			delete[] a;
			a = NULL;

			return L;
}

//---------------------------------------------------------------------------

void contractMeshGraphParallelOpenCL(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea, OpenCLManager openCLManager){
	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelOpenCL");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");
	#endif

	#ifdef _LOG
		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);


	boost::numeric::ublas::matrix<float> Ll(pMesh->numOfVertices,pMesh->numOfVertices);


	for (int i=0; i < pMesh->numOfVertices; i++)
		for (int j=0; j < pMesh->numOfVertices; j++)
			Ll(i,j) = L[i][j];	

	/*float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }*/

	/*int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}

	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		int num = 0;
		for (int j=0; j < numOfVertices; j++)
			if (Lm(i,j) != 0)
				num++;
		neighSize[i] = num + 1;
		if (num > maxNeigh){
			maxNeigh = num;
		}
	}

	// plus one for the -sum
	maxNeigh ++;*/

	/*int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;
		for (int j=0; j < numOfVertices; j++)
			if (Lm(i,j) != 0){
				neighbourhoods[i * (maxNeigh+1) + idx] = j;
				idx++;
			}
	}*/

	int neighbourhoodLevel = 1; // 0 for single point, 1 for one-ring area ...

	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}

	std::set<int> * neighbourIndices = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Lm(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = neighbourIndices[i].size();
		if (neighbourIndices[i].size() > maxNeigh){
			maxNeigh = neighbourIndices[i].size();
		}
	}

	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
			neighbourhoods[i * (maxNeigh+1) + idx] = *(it++);
			idx++;
		}
	}

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;
	}

	// call opencl computation of laplace contraction
	openCLManager.openCL_LaplaceContraction(pMesh->pVerts, inputVertices, numOfVertices, &Lm(0,0), maxNeigh,  neighbourhoods, pMesh->wL, pMesh->wH);

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphParallelOpenCLInterop(int * ite, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, int numOfIte, OpenCLContext oclc, OpenCLManager openCLManager){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelOpenCLInterop");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct E");
	#endif



	int neighbourhoodMinorLevel = 1; // 0 for single point, 1 for one-ring area ...
	int neighbourhoodMajorLevel = neighbourhoodMinorLevel + 1;

	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Em(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			if(pMesh->E[i][j])
				Em(i,j) = 1.0;
			else
				Em(i,j) = 0.0;
		}
	}

	// neighbourhood minor

	std::set<int> * neighbourIndices = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodMinorLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Em(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = neighbourIndices[i].size();
		if (neighbourIndices[i].size() > maxNeigh){
			maxNeigh = neighbourIndices[i].size();
		}
	}

	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
		neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
			neighbourhoods[i * (maxNeigh+1) + idx] = *(it++);
			idx++;
		}
	}

	// neighbourhood major

	std::set<int> * neighbourIndices2 = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices2[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodMajorLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices2[i].begin(); it != neighbourIndices2[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Em(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices2[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize2 = new int[numOfVertices];
	int maxNeigh2 = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize2[i] = neighbourIndices2[i].size();
		if (neighbourIndices2[i].size() > maxNeigh2){
			maxNeigh2 = neighbourIndices2[i].size();
		}
	}

	int * neighbourhoods2 = new int[numOfVertices * (maxNeigh2+1)];
	for (int i=0; i < numOfVertices; i++){
		neighbourhoods2[i * (maxNeigh2+1)] = neighSize2[i];
		for (int j=1; j < maxNeigh2+1; j++){
			neighbourhoods2[i * (maxNeigh2+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices2[i].begin(); it != neighbourIndices2[i].end();){
			neighbourhoods2[i * (maxNeigh2+1) + idx] = *(it++);
			idx++;
		}
	}

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;
	}


	#ifdef _LOG
		timerlog.addEnd();
	#endif

	// call opencl computation of laplace contraction
	openCLManager.openCL_LaplaceContractionInterop(ite, pMesh->pVerts, inputVertices, numOfVertices, &Em(0,0) ,maxNeigh,  neighbourhoods, maxNeigh2,  neighbourhoods2, pMesh->wL, pMesh->wH, sL, numOfIte, pMesh->gMeshgraphPositionsVB, oclc);


	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphParallelOpenCL2ring(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea, OpenCLManager openCLManager){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelOpenCL");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");
	#endif

	#ifdef _LOG
		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif
	
	L = calculateLaplacianMatrix(pMesh);

	boost::numeric::ublas::matrix<float> Ll(pMesh->numOfVertices,pMesh->numOfVertices);


	for (int i=0; i < pMesh->numOfVertices; i++)
		for (int j=0; j < pMesh->numOfVertices; j++)
			Ll(i,j) = L[i][j];	

	ofstream fileQR6 ("L-OCL2ring.mat", ofstream::binary);
	fileQR6 << Ll;
	fileQR6.close();

	/*float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }*/
	/*
	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}



	
	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		int num = 0;
		for (int j=0; j < numOfVertices; j++)
			if (Lm(i,j) != 0)
				num++;
		neighSize[i] = num + 1;
		if (num > maxNeigh){
			maxNeigh = num;
		}
	}

	// plus one for the -sum
	maxNeigh ++;*/

	/*int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;
		for (int j=0; j < numOfVertices; j++)
			if (Lm(i,j) != 0){
				neighbourhoods[i * (maxNeigh+1) + idx] = j;
				idx++;
			}
	}*/

	int neighbourhoodMinorLevel = 2; // 0 for single point, 1 for one-ring area ...
	int neighbourhoodMajorLevel = neighbourhoodMinorLevel + 1;

	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}

	// neighbourhood minor

	std::set<int> * neighbourIndices = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodMinorLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Lm(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = neighbourIndices[i].size();
		if (neighbourIndices[i].size() > maxNeigh){
			maxNeigh = neighbourIndices[i].size();
		}
	}

	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
			neighbourhoods[i * (maxNeigh+1) + idx] = *(it++);
			idx++;
		}
	}

	// neighbourhood major

	std::set<int> * neighbourIndices2 = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices2[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodMajorLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices2[i].begin(); it != neighbourIndices2[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Lm(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices2[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize2 = new int[numOfVertices];
	int maxNeigh2 = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize2[i] = neighbourIndices2[i].size();
		if (neighbourIndices2[i].size() > maxNeigh2){
			maxNeigh2 = neighbourIndices2[i].size();
		}
	}

	int * neighbourhoods2 = new int[numOfVertices * (maxNeigh2+1)];
	for (int i=0; i < numOfVertices; i++){
		neighbourhoods2[i * (maxNeigh2+1)] = neighSize2[i];
		for (int j=1; j < maxNeigh2+1; j++){
			neighbourhoods2[i * (maxNeigh2+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices2[i].begin(); it != neighbourIndices2[i].end();){
			neighbourhoods2[i * (maxNeigh2+1) + idx] = *(it++);
			idx++;
		}
	}

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;
	}

	// call opencl computation of laplace contraction
	openCLManager.openCL_LaplaceContraction2ring(pMesh->pVerts, inputVertices, numOfVertices, &Lm(0,0), maxNeigh,  neighbourhoods, maxNeigh2,  neighbourhoods2, pMesh->wL, pMesh->wH);

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphParallelOpenCLJacobi(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, OpenCLContext oclc, OpenCLManager openCLManager){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelOpenCLJacobi");
	#endif

	/*

	CO TREBA NABUDUCE RIESIT ::

	mam globalne L, ale ratam si jednotlive dimenzie samostatne
	potrebujem mat v X, ine B ako v Y a Z
	B si musim rekonstruovat na shadri, ale mat 3 rozne B predspracovane

	*/

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	
	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }

	 boost::numeric::ublas::matrix<float> A(2 * pMesh->numOfVertices,pMesh->numOfVertices);

	 int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> B(2 * numOfVertices, 3);

	WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("compute onering areas");

		timerlog.addEnd();

		timerlog.addStart("prepair submatrices");
	#endif

		for (int i = 0; i < pMesh->numOfVertices; i++) {
			WL[i][i] = pMesh->wL;
			WH[i][i] = pMesh->wH[i];
		}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica wL");
		logg.log(LOG_LEVEL_DUMP, WL);

		logg.log(LOG_LEVEL_DUMP, "Matica wH");
		logg.log(LOG_LEVEL_DUMP, WH);
	#endif

		UPA = WL * L;

		for (int m = 0; m < UPA.dim1(); m++)
			for (int n = 0; n < UPA.dim2(); n++)
				if (UPA[m][n] < -FLT_MAX) {
					UPA[m][n] = -FLT_MAX;
				} else if (UPA[m][n] > FLT_MAX) {
					UPA[m][n] = FLT_MAX;
				}

				#ifdef _LOG
					logg.log(LOG_LEVEL_DUMP, "Matica UPA");
					logg.log(LOG_LEVEL_DUMP, UPA);
				#endif

				// now solve the system using TNT and JAMA with QR decomposition

				A.clear();

				for (int j = 0; j < pMesh->numOfVertices; j++){
					A(pMesh->numOfVertices + j,j) = (double)WH[j][j];
					for (int i = 0; i < pMesh->numOfVertices; i++)
						A(i,j) = (double)UPA[i][j];
				}
				
				boost::numeric::ublas::matrix<float> Atrans = boost::numeric::ublas::trans(A);
				boost::numeric::ublas::matrix<float> ASquared =  boost::numeric::ublas::prod(Atrans, A);

				B.clear();

				for (int i = 0; i < pMesh->numOfVertices; i++){
					B(pMesh->numOfVertices + i,0) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
					B(pMesh->numOfVertices + i,1) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
					B(pMesh->numOfVertices + i,2) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
				}

				ofstream fileA ("A.mat", ofstream::binary);
				fileA << A;
				fileA.close();

				ofstream fileAtrans ("Atrans.mat", ofstream::binary);
				fileAtrans << Atrans;
				fileAtrans.close();

				ofstream fileAsquared ("Asquared.mat", ofstream::binary);
				fileAsquared << ASquared;
				fileAsquared.close();

				ofstream fileB ("B.mat", ofstream::binary);
				fileB << B;
				fileB.close();

				B =  boost::numeric::ublas::prod(Atrans, B);

				ofstream fileATB ("ATB.mat", ofstream::binary);
				fileATB << B;
				fileATB.close();

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i * 3] = pMesh->pVerts[i].x;
		inputVertices[i * 3 + 1] = pMesh->pVerts[i].y;
		inputVertices[i * 3 + 2] = pMesh->pVerts[i].z;
	}

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	// call opencl computation of laplace contraction
	openCLManager.openCL_LaplaceContractionJacobi(pMesh->pVerts, inputVertices, numOfVertices, &ASquared(0,0), &B(0,0), pMesh->gMeshgraphPositionsVB, oclc);


	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphParallelOpenCLJacobiInterop(int * ite, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, int numOfIte, OpenCLContext oclc, OpenCLManager openCLManager){
	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelOpenCLJacobiInterop");
	#endif

	/*

	CO TREBA NABUDUCE RIESIT ::

	mam globalne L, ale ratam si jednotlive dimenzie samostatne
	potrebujem mat v X, ine B ako v Y a Z
	B si musim rekonstruovat na shadri, ale mat 3 rozne B predspracovane

	*/

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);


	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }

	 boost::numeric::ublas::matrix<float> A(2 * pMesh->numOfVertices,pMesh->numOfVertices);

	 int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> B(2 * numOfVertices, 3);

	WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("compute onering areas");

		timerlog.addEnd();

		timerlog.addStart("prepair submatrices");
	#endif

		for (int i = 0; i < pMesh->numOfVertices; i++) {
			WL[i][i] = pMesh->wL;
			WH[i][i] = pMesh->wH[i];
		}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica wL");
		logg.log(LOG_LEVEL_DUMP, WL);

		logg.log(LOG_LEVEL_DUMP, "Matica wH");
		logg.log(LOG_LEVEL_DUMP, WH);
	#endif

		UPA = WL * L;

		for (int m = 0; m < UPA.dim1(); m++)
			for (int n = 0; n < UPA.dim2(); n++)
				if (UPA[m][n] < -FLT_MAX) {
					UPA[m][n] = -FLT_MAX;
				} else if (UPA[m][n] > FLT_MAX) {
					UPA[m][n] = FLT_MAX;
				}

				#ifdef _LOG
					logg.log(LOG_LEVEL_DUMP, "Matica UPA");
					logg.log(LOG_LEVEL_DUMP, UPA);
				#endif

				// now solve the system using TNT and JAMA with QR decomposition

				for (int j = 0; j < pMesh->numOfVertices; j++){
					A(pMesh->numOfVertices + j,j) = (double)WH[j][j];
					for (int i = 0; i < pMesh->numOfVertices; i++)
						A(i,j) = (double)UPA[i][j];
				}
				
				boost::numeric::ublas::matrix<float> Atrans = boost::numeric::ublas::trans(A);
				boost::numeric::ublas::matrix<float> ASquared =  boost::numeric::ublas::prod(Atrans, A);

				for (int i = 0; i < pMesh->numOfVertices; i++){
					B(pMesh->numOfVertices + i,0) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
					B(pMesh->numOfVertices + i,1) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
					B(pMesh->numOfVertices + i,2) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
				}

				B =  boost::numeric::ublas::prod(Atrans, B);

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;
	}

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	// call opencl computation of laplace contraction
	openCLManager.openCL_LaplaceContractionJacobi(pMesh->pVerts, inputVertices, numOfVertices, &ASquared(0,0), &B(0,0), pMesh->gMeshgraphPositionsVB, oclc);


	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

float solveParallelWithJama(int idx, Array2D< float >  L_global, float* input, unsigned int numOfVertices,  MeshGraph * pMesh,  unsigned int maxNeigh,  int* neighbourhoods){


	int idxmod = idx % numOfVertices;

	const unsigned int neighSize = neighbourhoods[idxmod * (maxNeigh + 1)];

	// nema susedov, neriesime
	if (neighSize == 0)
		return 0;

	// M > N !!!

	int m = 2 * neighSize;
	int n = neighSize;

	int idxneigh = -1;

	for (int i=0; i < n; i++){
		if (neighbourhoods[idxmod * (maxNeigh + 1) + i + 1] == idxmod)
			idxneigh = i;
	}

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< float > QR, A;
	Array1D< float > B, V;

	 WL = Array2D< float >(n, n, 0.0f);
	 WH = Array2D< float >(n, n, 0.0f);
	 UPA = Array2D< float >(n, n, 0.0f);

	 for (int i = 0; i < n; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
	 }

	A = Array2D<float>(m, n, 0.0f);

	L = Array2D<float>(n, n, 0.0f);

	V = Array1D<float>(n, 0.0f);
	for (int i=0; i < n; i++){
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		V[i] = input[curNeigh];
		for (int j=0; j < n; j++){
			int curNeigh2 = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
			L[i][j] = L_global[curNeigh][curNeigh2];
		}
	}
	
	for (int i = 0; i <n; i++){
		float sum = 0.0;
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		for (int j = 0; j <n; j++){
			int neigh1 = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
			if (pMesh->E[neigh1][curNeigh])
				sum -= L[i][j];
		}
		L[i][i] = sum;
	}

	//LV = L * V;
	
	UPA = WL * L;

	for (int j = 0; j <n; j++){
		for (int i = 0; i < n; i++)
			A[i][j] = (float)UPA[i][j];

		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
		A[n + j][j] = (float)WH[j][j];
	}

	B = Array1D<float>(m, 0.0f);

	// X
	for (int i = 0; i < n; i++){
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		B[n + i] = (float)(input[curNeigh + (idx - idxmod)] * WH[i][i]);
	}

	JAMA::QR<float> qr(A);

	/*		if (idx == 60){

	boost::numeric::ublas::matrix<float> Vold(n,1);


			for (int j=0; j < n; j++){
				Vold(j,0) = V[j];
			}

	ofstream fileV ("Vold.mat", ofstream::binary);
	fileV << Vold;
	fileV.close();

			
	boost::numeric::ublas::matrix<float> LVv(n,1);


			for (int j=0; j < n; j++){
				LVv(j,0) = LV[j][0];
			}

	ofstream fileQR9 ("LV.mat", ofstream::binary);
	fileQR9 << LVv;
	fileQR9.close();
	
	boost::numeric::ublas::matrix<float> NB(1,maxNeigh + 1);


			for (int j=0; j < maxNeigh + 1; j++)
				NB(0,j) = neighbourhoods[idxmod * (maxNeigh + 1) + j];	

	ofstream fileQR8 ("NB.mat", ofstream::binary);
	fileQR8 << NB;
	fileQR8.close();

	boost::numeric::ublas::matrix<float> Ll(n,n);


		for (int i=0; i < n; i++)
			for (int j=0; j < n; j++)
				Ll(i,j) = L[i][j];	

	ofstream fileQR6 ("Ll.mat", ofstream::binary);
	fileQR6 << Ll;
	fileQR6.close();

	boost::numeric::ublas::matrix<float> Aa(m,n);


		for (int i=0; i < m; i++)
			for (int j=0; j < n; j++)
				Aa(i,j) = A[i][j];	

	ofstream fileAa ("Aa.mat", ofstream::binary);
	fileAa << Aa;
	fileAa.close();

	boost::numeric::ublas::matrix<float> Bb(m,1);


		for (int i=0; i < m; i++)
				Bb(i,0) = B[i];	

	ofstream fileBb ("Bb.mat", ofstream::binary);
	fileBb << Bb;
	fileBb.close();

	}*/

	V = qr.solve(B);
	/*
	if (idx == 60){
		boost::numeric::ublas::matrix<float> Vnew(n,1);

	
			for (int j=0; j < n; j++){
				Vnew(j,0) = V[j];
			}

	ofstream fileV1 ("Vnew.mat", ofstream::binary);
	fileV1 << Vnew;
	fileV1.close();
	}*/

	return V[idxneigh];
}

//---------------------------------------------------------------------------

float solveParallelWithJama_SDF(int idx, float * centerPoints, Array2D< float >  L_global, float* input, unsigned int numOfVertices,  MeshGraph * pMesh,  unsigned int maxNeigh,  int* neighbourhoods){

	int idxmod = idx % numOfVertices;

	const unsigned int neighSize = neighbourhoods[idxmod * (maxNeigh + 1)];

	// nema susedov, neriesime
	if (neighSize == 0)
		return 0;

	// M > N !!!

	int m = 3 * neighSize;
	int n = neighSize;

	int idxneigh = -1;

	for (int i=0; i < n; i++){
		if (neighbourhoods[idxmod * (maxNeigh + 1) + i + 1] == idxmod)
			idxneigh = i;
	}

	Array2D< float >  L, WL, WH, WC, UPA, LV, O;
	Array2D< float > QR, A;
	Array1D< float > B, V;

	 WL = Array2D< float >(n, n, 0.0f);
	 WH = Array2D< float >(n, n, 0.0f);
	 WC = Array2D< float >(n, n, 0.0f);
	 UPA = Array2D< float >(n, n, 0.0f);

	 for (int i = 0; i < n; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
		WC[i][i] = pMesh->wC;
	 }

	A = Array2D<float>(m, n, 0.0f);

	L = Array2D<float>(n, n, 0.0f);

	V = Array1D<float>(n, 0.0f);
	for (int i=0; i < n; i++){
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		V[i] = input[curNeigh];
		for (int j=0; j < n; j++){
			int curNeigh2 = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
			L[i][j] = L_global[curNeigh][curNeigh2];
		}
	}
	
	for (int i = 0; i <n; i++){
		float sum = 0.0;
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		for (int j = 0; j <n; j++){
			int neigh1 = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
			if (pMesh->E[neigh1][curNeigh])
				sum -= L[i][j];
		}
		L[i][i] = sum;
	}

	//LV = L * V;
	
	UPA = WL * L;

	for (int j = 0; j <n; j++){
		for (int i = 0; i < n; i++)
			A[i][j] = (float)UPA[i][j];

		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + j + 1];
		A[n + j][j] = (float)WH[j][j];
		A[2 * n + j][j] = (float)WC[j][j];
	}

	B = Array1D<float>(m, 0.0f);

	// X
	for (int i = 0; i < n; i++){
		int curNeigh = neighbourhoods[idxmod * (maxNeigh + 1) + i + 1];
		B[n + i] = (float)(input[curNeigh + (idx - idxmod)] * WH[i][i]);
		B[2 * n + i] = (float)(centerPoints[curNeigh + (idx - idxmod)] * WC[i][i]);
	}

	JAMA::QR<float> qr(A);

	V = qr.solve(B);

	return V[idxneigh];
}

//---------------------------------------------------------------------------

void contractMeshGraphParallelCPU(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelCPU");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);

	/*float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }*/

	 //logg.log(0, "Matica L cotangent");
	 //logg.log(0, L);
	 /*
	 WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	 for (int i=0; i < pMesh->numOfVertices; i++)
		 for (int j=0; j < pMesh->numOfVertices; j++){
			 GLubyte val = ((L[i][j] - minL) / (maxL - minL)) * 255.0;
			 //GLubyte val = (float)j / (float)pMesh->numOfVertices * 255.0;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3] = val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 1] = 255 - val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 2] = 0;
		 }

		 recreateOperator = true;

	 timerlog.addEnd();

	 timerlog.addStart("compute onering areas");

	 timerlog.addEnd();

	 timerlog.addStart("prepair submatrices");

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
	 }

	 logg.log(LOG_LEVEL_DUMP, "Matica wL");
	 logg.log(LOG_LEVEL_DUMP, WL);

	 logg.log(LOG_LEVEL_DUMP, "Matica wH");
	 logg.log(LOG_LEVEL_DUMP, WH);

	 UPA = WL * L;

	 for (int m = 0; m < UPA.dim1(); m++)
		 for (int n = 0; n < UPA.dim2(); n++)
			if (UPA[m][n] < -FLT_MAX) {
							UPA[m][n] = -FLT_MAX;
							logg.log(LOG_LEVEL_WARNING,"UPA[m][n] bolo mensie ako -float v sume");
							logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
						 } else if (UPA[m][n] > FLT_MAX) {
								UPA[m][n] = FLT_MAX;
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] bolo vacsie ako float v sume");
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
			 					}

	 logg.log(LOG_LEVEL_DUMP, "Matica UPA");
	 logg.log(LOG_LEVEL_DUMP, UPA);

	// now solve the system using TNT and JAMA with QR decomposition

	A = Array2D<double>(2 * pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[pMesh->numOfVertices + i][j] = (double)WH[i][j];
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[i][j] = (double)UPA[i][j];
	}

	 logg.log(LOG_LEVEL_DUMP, "Matica A");
	 logg.log(LOG_LEVEL_DUMP, A);

	 B = Array2D<double>(2 * pMesh->numOfVertices, 3, 0.0f);

	// X
	for (int i = 0; i < pMesh->numOfVertices; i++){
		B[pMesh->numOfVertices + i][0] = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][1] = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][2] = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}*/

	 #ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepare and solve parallel on cpu");
	#endif

	int neighbourhoodLevel = 1; // 0 for single point, 1 for one-ring area ...

	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}

	std::set<int> * neighbourIndices = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Lm(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices[i].insert(newadded.begin(), newadded.end());
		}
	}

	/*int * neighSize = new int[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = numOfVertices;
	}
	int maxNeigh = numOfVertices;
	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
		neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=0; j < maxNeigh; j++){	
			neighbourhoods[i * (maxNeigh+1) + j + 1] = j;
		}
	}*/


	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = neighbourIndices[i].size();
		if (neighbourIndices[i].size() > maxNeigh){
			maxNeigh = neighbourIndices[i].size();
		}
	}

	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
			neighbourhoods[i * (maxNeigh+1) + idx] = *(it++);
			idx++;
		}
	}

	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;
	}

	float * results = new float[3 * numOfVertices];

	for (int i = 0; i<3 * numOfVertices; i++){
		results[i] = solveParallelWithJama(i, L, inputVertices, numOfVertices, pMesh, maxNeigh, neighbourhoods);
	}

	for (int i = 0; i<numOfVertices; i++){
		pMesh->pVerts[i] = CVector3((float)results[i], (float)results[numOfVertices + i], (float)results[2 * numOfVertices + i]);
	}

	/*for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)V[i][0], (float)V[i][1], (float)V[i][2]);*/

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif

}

void contractMeshGraphParallelCPU_SDF(MeshGraph * pMesh, CVector3 * centerPoints, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){
	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphParallelCPU");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");
	#endif

	#ifdef _LOG
		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("prepare and solve parallel on cpu");
	#endif

	int neighbourhoodLevel = 1; // 0 for single point, 1 for one-ring area ...

	int numOfVertices = pMesh->numOfVertices;

	boost::numeric::ublas::matrix<float> Lm(numOfVertices, numOfVertices);

	for (int i=0; i < numOfVertices; i++){
		for (int j=0; j < numOfVertices; j++){
			Lm(i,j) = L[i][j];
		}
	}

	std::set<int> * neighbourIndices = new std::set<int>[numOfVertices];
	for (int i=0; i < numOfVertices; i++){
		neighbourIndices[i].insert(i);
	}

	for (int i=0; i < numOfVertices; i++){ // pre kazdy vertex samostatny set
		for (int l=0; l < neighbourhoodLevel; l++){ // kolko mame levelov tolko krat iterujeme
			std::set<int> newadded = std::set<int>();
			for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
				for (int k=0; k < numOfVertices; k++){ // najdeme k nemu susedov (test so vsetkymi)
					if (Lm(*it,k) != 0)
						newadded.insert((k)); // ak je sused, pridame do nasho setu
				}
				++it;
			}
			neighbourIndices[i].insert(newadded.begin(), newadded.end());
		}
	}

	int * neighSize = new int[numOfVertices];
	int maxNeigh = 0;
	for (int i=0; i < numOfVertices; i++){
		neighSize[i] = neighbourIndices[i].size();
		if (neighbourIndices[i].size() > maxNeigh){
			maxNeigh = neighbourIndices[i].size();
		}
	}

	int * neighbourhoods = new int[numOfVertices * (maxNeigh+1)];
	for (int i=0; i < numOfVertices; i++){
			neighbourhoods[i * (maxNeigh+1)] = neighSize[i];
		for (int j=1; j < maxNeigh+1; j++){
			neighbourhoods[i * (maxNeigh+1) + j] = -1;
		}
		int idx = 1;

		for(std::set<int>::iterator it = neighbourIndices[i].begin(); it != neighbourIndices[i].end();){
			neighbourhoods[i * (maxNeigh+1) + idx] = *(it++);
			idx++;
		}
	}

	float * center = new float[3 * numOfVertices];
	float * inputVertices = new float[3 * numOfVertices];
	for (int i=0; i<numOfVertices; i++){
		inputVertices[i] = pMesh->pVerts[i].x;
		inputVertices[numOfVertices + i] = pMesh->pVerts[i].y;
		inputVertices[2 * numOfVertices + i] = pMesh->pVerts[i].z;

		center[i] = centerPoints[i].x;
		center[numOfVertices + i] = centerPoints[i].y;
		center[2 * numOfVertices + i] = centerPoints[i].z;
	}

	float * results = new float[3 * numOfVertices];

	if (pMesh->wC < 0.01){
		for (int i = 0; i<3 * numOfVertices; i++){
			results[i] = solveParallelWithJama(i, L, inputVertices, numOfVertices, pMesh, maxNeigh, neighbourhoods);
		}
	}
	else {
		for (int i = 0; i<3 * numOfVertices; i++){
			results[i] = solveParallelWithJama_SDF(i, center, L, inputVertices, numOfVertices, pMesh, maxNeigh, neighbourhoods);
		}
	}

	for (int i = 0; i<numOfVertices; i++){
		pMesh->pVerts[i] = CVector3((float)results[i], (float)results[numOfVertices + i], (float)results[2 * numOfVertices + i]);
	}

	/*for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)V[i][0], (float)V[i][1], (float)V[i][2]);*/

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");
		timerlog.logExecutionTimes();
	#endif



}

//---------------------------------------------------------------------------

void contractMeshGraphCPUCotangent(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){
	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphCPU1");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< float > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 double sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }

	 boost::numeric::ublas::matrix<float> Ll(pMesh->numOfVertices,pMesh->numOfVertices);


	 for (int i=0; i < pMesh->numOfVertices; i++)
		 for (int j=0; j < pMesh->numOfVertices; j++)
			 Ll(i,j) = L[i][j];	

	 //logg.log(0, "Matica L cotangent");
	 //logg.log(0, L);
	 
	 WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	 /*for (int i=0; i < pMesh->numOfVertices; i++)
		 for (int j=0; j < pMesh->numOfVertices; j++){
			 BYTE val = ((L[i][j] - minL) / (maxL - minL)) * 255.0;
			 //GLubyte val = (float)j / (float)pMesh->numOfVertices * 255.0;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3] = val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 1] = 255 - val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 2] = 0;
		 }

		 recreateOperator = true;*/

	#ifdef _LOG
		 timerlog.addEnd();

		timerlog.addStart("compute onering areas");

		timerlog.addEnd();

		timerlog.addStart("prepair submatrices");
	#endif

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
	 }

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica wL");
		 logg.log(LOG_LEVEL_DUMP, WL);

		 logg.log(LOG_LEVEL_DUMP, "Matica wH");
		 logg.log(LOG_LEVEL_DUMP, WH);
	#endif

	 UPA = WL * L;

	 for (int m = 0; m < UPA.dim1(); m++)
		 for (int n = 0; n < UPA.dim2(); n++)
			if (UPA[m][n] < -FLT_MAX) {
							UPA[m][n] = -FLT_MAX;
							//logg.log(LOG_LEVEL_WARNING,"UPA[m][n] bolo mensie ako -float v sume");
							//logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
						 } else if (UPA[m][n] > FLT_MAX) {
								UPA[m][n] = FLT_MAX;
								//logg.log(LOG_LEVEL_WARNING, "UPA[m][n] bolo vacsie ako float v sume");
								//logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
			 					}

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica UPA");
		 logg.log(LOG_LEVEL_DUMP, UPA);
	#endif

	// now solve the system using TNT and JAMA with QR decomposition

	A = Array2D<float>(2 * pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[pMesh->numOfVertices + i][j] = (float)WH[i][j];
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[i][j] = (float)UPA[i][j];
	}

	 //logg.log(0, "Matica A");
	 //logg.log(0, A);

	 B = Array2D<float>(2 * pMesh->numOfVertices, 3, 0.0f);

	// X
	for (int i = 0; i < pMesh->numOfVertices; i++){
		B[pMesh->numOfVertices + i][0] = (float)(pMesh->pVerts[i].x * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][1] = (float)(pMesh->pVerts[i].y * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][2] = (float)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}

	//logg.log(0, "Matica B");
	//logg.log(0, B);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition");
	#endif

	JAMA::QR<float> qr(A);
	V = qr.solve(B);

	//logg.log(0, "Nove vrcholy");
	//logg.log(0, V);

	//delete[] curOneRingArea;
	//curOneRingArea = NULL;

	
	for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)V[i][0], (float)V[i][1], (float)V[i][2]);

}

//---------------------------------------------------------------------------

void contractMeshGraphCPUPCL(MeshGraph * origMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){
	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphCPU1");
	#endif

	MeshGraph * pMesh = new MeshGraph();
	// zakomentovane koli tomu, ze misovi nejde PCL includovat
	//greedyProjection(origMesh, pMesh);

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenarates");
	#endif

	Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	L = calculateLaplacianMatrix(pMesh);

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L[i][j];

			 if (pMesh->E[i][j] > maxL)
				 maxL = pMesh->E[i][j];

 			 if (pMesh->E[i][j] < minL)
				 minL = pMesh->E[i][j];

		 }

		 L[i][i] = sum;
	 }

	 //logg.log(0, "Matica L cotangent");
	 //logg.log(0, L);
	 
	 WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	 UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	 /*for (int i=0; i < pMesh->numOfVertices; i++)
		 for (int j=0; j < pMesh->numOfVertices; j++){
			 BYTE val = ((L[i][j] - minL) / (maxL - minL)) * 255.0;
			 //GLubyte val = (float)j / (float)pMesh->numOfVertices * 255.0;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3] = val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 1] = 255 - val;
			 LImage[i * pMesh->numOfVertices * 3 + j * 3 + 2] = 0;
		 }

		 recreateOperator = true;*/

	#ifdef _LOG
		 timerlog.addEnd();

		timerlog.addStart("compute onering areas");

		timerlog.addEnd();

		timerlog.addStart("prepair submatrices");
	#endif

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
	 }

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica wL");
		 logg.log(LOG_LEVEL_DUMP, WL);

		 logg.log(LOG_LEVEL_DUMP, "Matica wH");
		 logg.log(LOG_LEVEL_DUMP, WH);
	#endif

	 UPA = WL * L;

	 for (int m = 0; m < UPA.dim1(); m++)
		 for (int n = 0; n < UPA.dim2(); n++)
			if (UPA[m][n] < -FLT_MAX) {
							UPA[m][n] = -FLT_MAX;
							#ifdef _LOG
								logg.log(LOG_LEVEL_WARNING,"UPA[m][n] bolo mensie ako -float v sume");
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
							#endif
						 } else if (UPA[m][n] > FLT_MAX) {
								UPA[m][n] = FLT_MAX;
								#ifdef _LOG
									logg.log(LOG_LEVEL_WARNING, "UPA[m][n] bolo vacsie ako float v sume");
									logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
								#endif
			 					}

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica UPA");
		 logg.log(LOG_LEVEL_DUMP, UPA);
	#endif

	// now solve the system using TNT and JAMA with QR decomposition

	A = Array2D<double>(2 * pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[pMesh->numOfVertices + i][j] = (double)WH[i][j];
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[i][j] = (double)UPA[i][j];
	}

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica A");
		 logg.log(LOG_LEVEL_DUMP, A);
	#endif

	 B = Array2D<double>(2 * pMesh->numOfVertices, 3, 0.0f);

	// X
	for (int i = 0; i < pMesh->numOfVertices; i++){
		B[pMesh->numOfVertices + i][0] = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][1] = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][2] = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica B");
		logg.log(LOG_LEVEL_DUMP, B);

		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition");
	#endif

	JAMA::QR<double> qr(A);
	V = qr.solve(B);


	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Nove vrcholy");
		logg.log(LOG_LEVEL_DUMP, V);
	#endif

	//delete[] curOneRingArea;
	//curOneRingArea = NULL;

	for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)V[i][0], (float)V[i][1], (float)V[i][2]);

}

/*float GLaplace(float h, CVector3 v1, CVector3 v2){
	float exp1 = -((pow(v1.x - v2.x, 2.0f) + pow(v1.y - v2.y, 2.0f) + pow(v1.z - v2.z, 2.0f)) / (4.0 * h));
	return pow((float)boost::math::constants::e<float>(), exp1) / (4 * PI * pow(h, 2.0f));
}*/

//---------------------------------------------------------------------------

void findNearestKNeighbours(MeshGraph * pMesh, int idx, int kneigh, int * neighbours){
		float * distances = new float[kneigh];
		for (int i=0; i < kneigh; i++)
			distances[i] = FLT_MAX;
		float maxdist = FLT_MAX;
		int maxdistIndex = 0;

		for (int j = 0; j < pMesh->numOfVertices; j++){
			if (idx != j){
				float length = Magnitude(pMesh->pVerts[idx] - pMesh->pVerts[j]);
				if (length < maxdist){
					// zarad do zoznamu
					neighbours[maxdistIndex] = j;
					distances[maxdistIndex] = length;
					maxdist = 0;
					// aktualizuj max dist
					for (int k=0; k < kneigh; k++){
						if (distances[k] > maxdist){
							maxdist = distances[k];
							maxdistIndex = k;
						}
					}

				}
					
			}

		}
}

//---------------------------------------------------------------------------

float getNeighbourhoodKernelWidth(MeshGraph * pMesh, int idx, int kneigh, int * neighbours){
	float maxdist = 0;
	for (int k=0; k < kneigh; k++){
		float length = Magnitude(pMesh->pVerts[idx] - pMesh->pVerts[neighbours[k]]);
		if (length > maxdist){
			maxdist = length;
		}
	}
	return maxdist;
}

//---------------------------------------------------------------------------

void contractMeshGraphCPUPointCloud(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingExtent, float * curOneRingArea, int kneigh, vector<std::set<int>> globalNeighbourhoods, PointCloudTriangulation::DeleunayTriangulator * pTriangulator){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphCPU2");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");
	#endif

    Array2D< float >  L, WL, WH, UPA, LV, O;
	Array2D< double > QR, A, B, V;

	// compute Laplace operator from new points

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - construction of Laplacian operator");

		timerlog.addStart("construct L");
	#endif

	float *a = new float[2];
	int *idcs = new int[2];

	bool degenerates = false;

	L = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	//float kernelWidth = getNeighbourhoodKernelWidth(pMesh, i, kneigh, globalNeighbourhoods[i]);

	/*float kernelWidth = 0;
	for (int i = 0; i < pMesh->numOfVertices; i++){
		kernelWidth += pMesh->origOneRingExtent[i];
	}
	kernelWidth /= pMesh->numOfVertices;
	kernelWidth *= 3.0;*/

	/*float xx = pModel->modelbb.x_max - pModel->modelbb.x_min;
	float yy = pModel->modelbb.y_max - pModel->modelbb.y_min;
	float zz = pModel->modelbb.z_max - pModel->modelbb.z_min;
	float kernelWidth = 0.005 * ((xx + yy + zz) / 3.0);*/

	/*for (int i = 0; i < pMesh->numOfVertices; i++){

	 //logg.log(0, "Vertex i:", i);
	 //logg.log(0, "Maxedge for i:", maxedge);
		for (int j = 0; j < pMesh->numOfVertices; j++){
				L[i][j] = GLaplace(kernelWidth, pMesh->pVerts[i], pMesh->pVerts[j]) * (curOneRingArea[j] / 3.0);
		}
		//for (int k = 0; k < kneigh; k++)
			//	L[i][globalNeighbourhoods[i][k]] = 1.0f;
				//L[i][globalNeighbourhoods[i][k]] = GLaplace(kernelWidth, pMesh->pVerts[i], pMesh->pVerts[globalNeighbourhoods[i][k]]) * (curOneRingArea[globalNeighbourhoods[i][k]] / 3.0);

	}*/


	/*
	 
	For each point in for cycle:
	- find k nearest neighbours
	- PCA and plane normal
	- project them into a plane
	- compute 2D Delanay triangulation
	- compute local Li
	- copy local Li into global matrix L

	*/

	float * verts = new float[pMesh->numOfVertices * 3];
	for (int j=0; j < pMesh->numOfVertices; j++){
		verts[j * 3] = pMesh->pVerts[j].x;
		verts[j * 3 + 1] = pMesh->pVerts[j].y;
		verts[j * 3 + 2] = pMesh->pVerts[j].z;
	}

	int numOfTrians = 0;
	int * indices = NULL;

	pTriangulator->neighVis = new PCTNeighVisualization[pMesh->numOfVertices];
	for (int i=0; i<pMesh->numOfVertices; i++){
		pTriangulator->neighVis[i].isE_local_visualize = false;
	}

	pTriangulator->neighVisSize = pMesh->numOfVertices;

	for (int i=0; i<pMesh->numOfVertices; i++){

			Array2D<bool> E_local(pMesh->numOfVertices, pMesh->numOfVertices, false);

			float * nor = new float[3];

			// UNCOMMENT pTriangulator->computeLocalTriangulationFromPoints(i, pMesh->numOfVertices, verts, numOfTrians, &indices, globalNeighbourhoods, nor, true);

			for (int j=0; j<numOfTrians; j++){
				E_local[indices[j * 3]][indices[j * 3 + 1]] = true;
				E_local[indices[j * 3 + 1]][indices[j * 3]] = true;
				E_local[indices[j * 3 + 1]][indices[j * 3 + 2]] = true;
				E_local[indices[j * 3 + 2]][indices[j * 3 + 1]] = true;
				E_local[indices[j * 3]][indices[j * 3 + 2]] = true;
				E_local[indices[j * 3 + 2]][indices[j * 3]] = true;
			}

		// calculate Laplacian L from one ring area

		float *a = new float[2];

		for (int j = 0; j < pMesh->numOfVertices; j++)
			if (E_local[i][j] && i != j){

				int ind = 0;
				for (int k = 0; k < pMesh->numOfVertices; k++){
					if (E_local[i][k] && E_local[j][k] && i != k && j != k){
						CVector3 v1 = pMesh->pVerts[k] - pMesh->pVerts[j];
						CVector3 v2 = pMesh->pVerts[k] - pMesh->pVerts[i];

						if (ind > 1) {
							#ifdef _LOG
								logg.log(LOG_LEVEL_WARNING, "Vrchol narusajuci 2 manifold: " ,k);
							#endif
						} else
							a[ind++] = AngleBetweenVectors(v1, v2);
					}
				}
				if (ind == 2){
					//L[i][j] = 1.0f;
					L[i][j] = cotan(a[0]) + cotan(a[1]);

					if (L[i][j] < -FLT_MAX) {
						L[i][j] = -FLT_MAX;
						#ifdef _LOG
							logg.log(LOG_LEVEL_WARNING, "L bolo mensie ako -float");
							logg.log(LOG_LEVEL_WARNING, "alfa : " ,a[0]);
							logg.log(LOG_LEVEL_WARNING, "beta : " ,a[1]);
							logg.log(LOG_LEVEL_WARNING, "i: " ,i);
							logg.log(LOG_LEVEL_WARNING, "j: " ,j);
						#endif
					} else if (L[i][j] > FLT_MAX) {
						L[i][j] = FLT_MAX;
						#ifdef _LOG
							logg.log(LOG_LEVEL_WARNING, "L bolo vacsie ako float");
							logg.log(LOG_LEVEL_WARNING, "alfa : " ,a[0]);
							logg.log(LOG_LEVEL_WARNING, "beta : " ,a[1]);
							logg.log(LOG_LEVEL_WARNING, "i: " ,i);
							logg.log(LOG_LEVEL_WARNING, "j: " ,j);
						#endif
					}
				} 
			}

			delete[] a;
			a = NULL;

		  /////////////////////////////////
	}

					
	WL = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	WH = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);
	UPA = Array2D< float >(pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);	
			
	delete[] a;
	a = NULL;

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (i != j)
				sum -= L[i][j];

			 if (L[i][j] > maxL)
				 maxL = L[i][j];

 			 if (L[i][j] < minL)
				 minL = L[i][j];

		 }
		 L[i][i] = sum;
	 }
	 //logg.log(0, "Matica L pre pointcloud");
	 //logg.log(0, L);

	 /*for (int i=0; i < pMesh->numOfVertices; i++)
		 for (int j=0; j < pMesh->numOfVertices; j++){
			BYTE val = ((L[i][j] - minL) / (maxL - minL)) * 255.0;
			//GLubyte val = (float)j / (float)pMesh->numOfVertices * 255.0;
			LImage[i * pMesh->numOfVertices * 3 + j * 3] = val;
			LImage[i * pMesh->numOfVertices * 3 + j * 3 + 1] = 255 - val;
			LImage[i * pMesh->numOfVertices * 3 + j * 3 + 2] = 0;
		 }

	 recreateOperator = true;*/

	#ifdef _LOG
		 timerlog.addEnd();

		 timerlog.addStart("compute onering areas");

		 timerlog.addEnd();

		 logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - construction of Laplacian operator");

		 logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - preparing of weights matrices");

		 timerlog.addStart("prepare submatrices");
	#endif

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL[i][i] = pMesh->wL;
		WH[i][i] = pMesh->wH[i];
	 }

	 //logg.log(0, "Matica wL");
	 //logg.log(0, WL);

	 //logg.log(0, "Matica wH");
	 //logg.log(0, WH);

	 UPA = WL * L;

	 for (int m = 0; m < UPA.dim1(); m++)
		 for (int n = 0; n < UPA.dim2(); n++)
			if (UPA[m][n] < -FLT_MAX) {
							UPA[m][n] = -FLT_MAX;
							#ifdef _LOG
								logg.log(LOG_LEVEL_WARNING,"UPA[m][n] bolo mensie ako -float v sume");
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
							#endif
						 } else if (UPA[m][n] > FLT_MAX) {
								UPA[m][n] = FLT_MAX;
								#ifdef _LOG
									logg.log(LOG_LEVEL_WARNING, "UPA[m][n] bolo vacsie ako float v sume");
									logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : " ,UPA[m][n]);
								#endif
			 					}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica UPA");
		logg.log(LOG_LEVEL_DUMP, UPA);
	#endif

	// now solve the system using TNT and JAMA with QR decomposition

	A = Array2D<double>(2 * pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[pMesh->numOfVertices + i][j] = (double)WH[i][j];
		for (int i = 0; i < pMesh->numOfVertices; i++)
			A[i][j] = (double)UPA[i][j];
	}

	 //logg.log(0, "Matica A");
	 //logg.log(0, A);


	B = Array2D<double>(2 * pMesh->numOfVertices, 3, 0.0f);

	// X
	for (int i = 0; i < pMesh->numOfVertices; i++){
		B[pMesh->numOfVertices + i][0] = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][1] = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
		B[pMesh->numOfVertices + i][2] = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	//logg.log(0, "Matica B");
	//logg.log(0, B);

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - preparing of weights matrices");

		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - solve LV system");

		timerlog.addStart("solve using QR decomposition");
	#endif

	JAMA::QR<double> qr(A);
	V = qr.solve(B);

	#ifdef _LOG
		timerlog.addEnd();

		logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - solve LV system");

		logg.log(0, "Nove vrcholy");
		logg.log(0, V);
	#endif

	for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)V[i][0], (float)V[i][1], (float)V[i][2]);

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");

		timerlog.logExecutionTimes();
	#endif
}

//---------------------------------------------------------------------------

void contractMeshGraphGPUVCL(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphGPU");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenerates");
	#endif

	using namespace boost::numeric::ublas;

	matrix<float> L(pMesh->numOfVertices, pMesh->numOfVertices);
	L.clear();
	matrix<float> WL(pMesh->numOfVertices, pMesh->numOfVertices);
	WL.clear();
	matrix<float> WH(pMesh->numOfVertices, pMesh->numOfVertices);
	WH.clear();
	matrix<float> UPA(pMesh->numOfVertices, pMesh->numOfVertices);
	UPA.clear();
	matrix<float> LV(pMesh->numOfVertices, pMesh->numOfVertices);
	LV.clear();
	matrix<float> O(pMesh->numOfVertices, pMesh->numOfVertices);
	O.clear();

	// compute Laplace operator from new points

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - construction of Laplacian operator");
	#endif

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG

		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	Array2D< float > Ltnt = calculateLaplacianMatrix(pMesh);

	for (int i = 0; i < pMesh->numOfVertices; i++){
		for (int j = 0; j < pMesh->numOfVertices; j++){
			L(i,j) = Ltnt[i][j];
		}
	}

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L(i,j);

			 }

		 L(i,i) = sum;
	 }

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica L");

		 // logg.log(LOG_LEVEL_DUMP, L);

		 timerlog.addEnd();

		 timerlog.addStart("compute onering areas");

		 timerlog.addEnd();

		 logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - construction of Laplacian operator");

		 logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - preparing of weights matrices");

		 timerlog.addStart("prepare submatrices - WH");
	#endif

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL(i,i) = pMesh->wL;

		#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "one ring area pre vrchol ", i);
				logg.log(LOG_LEVEL_DUMP, "cur ", curOneRingArea[i]);
				logg.log(LOG_LEVEL_DUMP, "orig ", pMesh->origOneRingArea[i]);
		#endif

		WH(i,i) = pMesh->wH[i];
	 }
	 //delete[] curOneRingArea;

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica wL");
	//	 logg.log(LOG_LEVEL_DUMP, WL);

		 logg.log(LOG_LEVEL_DUMP, "Matica wH");
		// logg.log(LOG_LEVEL_DUMP, WH);

		 timerlog.addEnd();
		 timerlog.addStart("prepare submatrices - UPA");
	#endif

	 UPA = prod (WL, L);

	/* for (int m = 0; m < UPA.size1(); m++)
		 for (int n = 0; n < UPA.size2(); n++)
			if (UPA(m,n) < -FLT_MAX) {
							UPA(m,n) = -FLT_MAX;
							logg.log(LOG_LEVEL_WARNING,"UPA[m][n] bolo mensie ako -float v sume");
							logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : ", UPA(m,n));
						 } else if (UPA(m,n) > FLT_MAX) {
								UPA(m,n) = FLT_MAX;
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] bolo vacsie ako float v sume");
								logg.log(LOG_LEVEL_WARNING, "UPA[m][n] : ", UPA(m,n));
			 					}*/

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica UPA");

		 timerlog.addEnd();
		 timerlog.addStart("prepare submatrices - A");

	//	 logg.log(LOG_LEVEL_DUMP, UPA);
	#endif

	 matrix<float> A(2 * pMesh->numOfVertices, pMesh->numOfVertices);
	 //matrix<float> B(2 * pMesh->numOfVertices, 3);
	 boost::numeric::ublas::vector<float> Bvec_x(2 * pMesh->numOfVertices);
	 boost::numeric::ublas::vector<float> Bvec_y(2 * pMesh->numOfVertices);
	 boost::numeric::ublas::vector<float> Bvec_z(2 * pMesh->numOfVertices);
	 Bvec_x.clear();
	 Bvec_y.clear();
	 Bvec_z.clear();

	// now solve the system using TNT and JAMA with QR decomposition

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++){
			A(i,j) = (double)UPA(i,j);
			A(pMesh->numOfVertices + i,j) = (double)WH(i,j);
		}
	}

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica A");
		 //logg.log(LOG_LEVEL_DUMP, A);

		 timerlog.addEnd();
		 timerlog.addStart("prepare submatrices - B");
	#endif

	for (int i = 0; i < pMesh->numOfVertices; i++){
		Bvec_x(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
		Bvec_y(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
		Bvec_z(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica B");
		//logg.log(LOG_LEVEL_DUMP, B);
	#endif

	matrix<float> Q(2 * pMesh->numOfVertices, 2 * pMesh->numOfVertices);
	matrix<float> R(2 * pMesh->numOfVertices, pMesh->numOfVertices);
	//matrix<float> V(2 * pMesh->numOfVertices, 3);
	boost::numeric::ublas::vector<float> Vvec_x(pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Vvec_y(pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Vvec_z(pMesh->numOfVertices);

	boost::numeric::ublas::vector<float> Yvec_x(2 * pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Yvec_y(2 * pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Yvec_z(2 * pMesh->numOfVertices);

	// write matrices to stream file

#ifdef _LOG
	logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - preparing of weights matrices");

	logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - solve LV system");

	timerlog.addEnd();

	timerlog.addStart("solve using QR decomposition - get QR on GPU - prepare matrices copy to GPU");
#endif

	//set up a sparse ViennaCL matrix:
	viennacl::matrix<float, viennacl::column_major> A_GPU(2 * pMesh->numOfVertices, pMesh->numOfVertices);
	viennacl::vector<float> Bvec_x_GPU(2 * pMesh->numOfVertices);
	viennacl::vector<float> Bvec_y_GPU(2 * pMesh->numOfVertices);
	viennacl::vector<float> Bvec_z_GPU(2 * pMesh->numOfVertices);
	viennacl::compressed_matrix<float> B_GPU(2 * pMesh->numOfVertices, 3);
	//viennacl::compressed_matrix<float> V_GPU(pMesh->numOfVertices, 3);
	viennacl::vector<float> Vvec_x_GPU(pMesh->numOfVertices);
	viennacl::vector<float> Vvec_y_GPU(pMesh->numOfVertices);
	viennacl::vector<float> Vvec_z_GPU(pMesh->numOfVertices);

	viennacl::vector<float> Yvec_x_GPU(2 * pMesh->numOfVertices);
	viennacl::vector<float> Yvec_y_GPU(2 * pMesh->numOfVertices);
	viennacl::vector<float> Yvec_z_GPU(2 * pMesh->numOfVertices);

	//copy to OpenCL device:
	//viennacl::fast_copy(&(A[0]), &(A[0]) + A.size(), A_GPU);
	//viennacl::fast_copy(&(B[0]), &(B[0]) + B.size(), B_GPU);
	viennacl::copy(A, A_GPU);
	//viennacl::copy(A.begin(), A.end() ,A_GPU);
	//viennacl::copy(Bvec_x, Bvec_x_GPU);
	//viennacl::copy(Bvec_y, Bvec_y_GPU);
	//viennacl::copy(Bvec_z, Bvec_z_GPU);
	//viennacl::copy(Bvec, Bvec_GPU);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition - get QR on GPU - get hybrid betas");
	#endif

	std::vector<float> betas = viennacl::linalg::inplace_qr(A_GPU);
	//std::vector<float> betas = viennacl::linalg::inplace_qr(A);
	//viennacl::compressed_matrix<float> Q_GPU(2 * pMesh->numOfVertices, 2 * pMesh->numOfVertices);
	//viennacl::compressed_matrix<float> R_GPU(2 * pMesh->numOfVertices, pMesh->numOfVertices);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition - get QR on GPU - recover Q and R");
	#endif

	viennacl::copy(A_GPU, A);
	viennacl::linalg::recoverQ(A, betas, Q, R);

	/*Array2D< float > Q_DEBUG = Array2D< float >(2*pMesh->numOfVertices, 2*pMesh->numOfVertices, 0.0f);
	Array2D< float > R_DEBUG = Array2D< float >(2*pMesh->numOfVertices, pMesh->numOfVertices, 0.0f);

	for (int i=0; i < 2*pMesh->numOfVertices; i++){
		for (int j=0; j < 2*pMesh->numOfVertices; j++)
			Q_DEBUG[i][j] = Q(i,j);
		for (int j=0; j < pMesh->numOfVertices; j++)
			R_DEBUG[i][j] = Q(i,j);
	}

	logg.log(0, "Matica Q:");
	logg.log(0, Q_DEBUG);
	
	logg.log(0, "Matica R:");
	logg.log(0, R_DEBUG);*/

	//viennacl::compressed_matrix<float> Y_GPU(2 * pMesh->numOfVertices, 2 * pMesh->numOfVertices);

	//viennacl::copy(R, R_GPU);
	//viennacl::copy(trans(Q), Q_GPU);
	//Yvec_x_GPU = viennacl::linalg::prod(Q_GPU, Bvec_x_GPU);
	//Yvec_y_GPU = viennacl::linalg::prod(Q_GPU, Bvec_y_GPU);
	//Yvec_z_GPU = viennacl::linalg::prod(Q_GPU, Bvec_z_GPU);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition - get Y on CPU");
	#endif

	Yvec_x = prod(trans(Q), Bvec_x);
	Yvec_y = prod(trans(Q), Bvec_y);
	Yvec_z = prod(trans(Q), Bvec_z);

	//viennacl::copy(Y, Y_GPU);

	//viennacl::ocl::get_queue().finish();
	//timer.start();

	Yvec_x.resize(pMesh->numOfVertices, true);
	Yvec_y.resize(pMesh->numOfVertices, true);
	Yvec_z.resize(pMesh->numOfVertices, true);
	R.resize(pMesh->numOfVertices, pMesh->numOfVertices, true);

	//Vvec_x_GPU = viennacl::linalg::solve(R_GPU, Yvec_x_GPU, viennacl::linalg::cg_tag());
	//Vvec_y_GPU = viennacl::linalg::solve(R_GPU, Yvec_y_GPU, viennacl::linalg::cg_tag());
	//Vvec_z_GPU = viennacl::linalg::solve(R_GPU, Yvec_z_GPU, viennacl::linalg::cg_tag());

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("solve using QR decomposition - triangular solver on CPU");
	#endif

	Vvec_x = boost::numeric::ublas::solve(R, Yvec_x, upper_tag());
	Vvec_y = boost::numeric::ublas::solve(R, Yvec_y, upper_tag());
	Vvec_z = boost::numeric::ublas::solve(R, Yvec_z, upper_tag());

	#ifdef _LOG
		timerlog.addEnd();
	#endif

	//copy back to CPU:
	//viennacl::copy(Vvec_x_GPU, Vvec_x);
	//viennacl::copy(Vvec_y_GPU, Vvec_y);
	//viennacl::copy(Vvec_z_GPU, Vvec_z);
	//viennacl::copy(Vvec_GPU, Vvec);

	//JAMA::QR<double> qr(A);
	//V = qr.solve(B);

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - solve LV system");

		logg.log(LOG_LEVEL_DUMP, "Nove vrcholy");
		//logg.log(LOG_LEVEL_DUMP, V);
	#endif


	for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)Vvec_x(i), (float)Vvec_y(i), (float)Vvec_z(i));

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");

		timerlog.logExecutionTimes();
	#endif


}

//---------------------------------------------------------------------------

void contractMeshGraphGPUVCL_LSM(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphGPU");
		#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenerates");
	#endif
	
	using namespace boost::numeric::ublas;

	// setutp ViennaCL context and device
	viennacl::ocl::set_context_device_type(0, CL_DEVICE_TYPE_GPU);

	matrix<float> L(pMesh->numOfVertices, pMesh->numOfVertices);
	L.clear();
	matrix<float> WL(pMesh->numOfVertices, pMesh->numOfVertices);
	WL.clear();
	matrix<float> WH(pMesh->numOfVertices, pMesh->numOfVertices);
	WH.clear();
	matrix<float> UPA(pMesh->numOfVertices, pMesh->numOfVertices);
	UPA.clear();
	matrix<float> LV(pMesh->numOfVertices, pMesh->numOfVertices);
	LV.clear();
	matrix<float> O(pMesh->numOfVertices, pMesh->numOfVertices);
	O.clear();

	// compute Laplace operator from new points

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - construction of Laplacian operator");
	#endif

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	Array2D< float > Ltnt = calculateLaplacianMatrix(pMesh);

	for (int i = 0; i < pMesh->numOfVertices; i++){
		for (int j = 0; j < pMesh->numOfVertices; j++){
			L(i,j) = Ltnt[i][j];
		}
	}

	float maxL = -FLT_MAX;
	float minL = FLT_MAX;

	 for (int i = 0; i < pMesh->numOfVertices; i++){
		 float sum = 0.0;
		 for (int j = 0; j < pMesh->numOfVertices; j++){
			 if (pMesh->E[i][j])
				sum -= L(i,j);
		 }

		 L(i,i) = sum;
	 }

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica L");

		 timerlog.addEnd();

		 timerlog.addStart("compute onering areas");

		 timerlog.addEnd();

		 logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - construction of Laplacian operator");

		 logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - preparing of weights matrices");

		 timerlog.addStart("prepare submatrices - WH");
	#endif

	 for (int i = 0; i < pMesh->numOfVertices; i++) {
		WL(i,i) = pMesh->wL;

		#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "one ring area pre vrchol ", i);
				logg.log(LOG_LEVEL_DUMP, "cur ", curOneRingArea[i]);
				logg.log(LOG_LEVEL_DUMP, "orig ", pMesh->origOneRingArea[i]);
		#endif

		WH(i,i) = pMesh->wH[i];
	 }
	 //delete[] curOneRingArea;

	#ifdef _LOG
		 logg.log(LOG_LEVEL_DUMP, "Matica wL");
	//	 logg.log(LOG_LEVEL_DUMP, WL);

		 logg.log(LOG_LEVEL_DUMP, "Matica wH");
		// logg.log(LOG_LEVEL_DUMP, WH);

	 
		 timerlog.addEnd();
		 timerlog.addStart("prepare submatrices - A");
	#endif

	 UPA = prod (WL, L);

//	 logg.log(LOG_LEVEL_DUMP, UPA);

	 matrix<float> A(2 * pMesh->numOfVertices, pMesh->numOfVertices);
	 //matrix<float> B(2 * pMesh->numOfVertices, 3);
	 boost::numeric::ublas::vector<float> Bvec_x(2 * pMesh->numOfVertices);
	 boost::numeric::ublas::vector<float> Bvec_y(2 * pMesh->numOfVertices);
	 boost::numeric::ublas::vector<float> Bvec_z(2 * pMesh->numOfVertices);
	 Bvec_x.clear();
	 Bvec_y.clear();
	 Bvec_z.clear();

	// now solve the system using TNT and JAMA with QR decomposition

	for (int j = 0; j < pMesh->numOfVertices; j++){
		for (int i = 0; i < pMesh->numOfVertices; i++){
			A(i,j) = (double)UPA(i,j);
			A(pMesh->numOfVertices + i,j) = (double)WH(i,j);
		}
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_DUMP, "Matica A");
		//logg.log(LOG_LEVEL_DUMP, A);
		 timerlog.addEnd();
		 timerlog.addStart("prepare submatrices - B");
	#endif

	// X
	for (int i = 0; i < pMesh->numOfVertices; i++){
		Bvec_x(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
		Bvec_y(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
		Bvec_z(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
	}


	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - preparing of weights matrices");

		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - solve LV system");
	#endif

	boost::numeric::ublas::vector<float> Vvec_x(pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Vvec_y(pMesh->numOfVertices);
	boost::numeric::ublas::vector<float> Vvec_z(pMesh->numOfVertices);

	#ifdef _LOG
		timerlog.addEnd();
		timerlog.addStart("viennaCL - inplace QR - get betas");
	#endif

	typedef float               ScalarType; 
	typedef boost::numeric::ublas::matrix<ScalarType>              MatrixType;
	typedef boost::numeric::ublas::vector<ScalarType>              VectorType;
	typedef viennacl::matrix<ScalarType, viennacl::column_major>   VCLMatrixType;
	typedef viennacl::vector<ScalarType>                           VCLVectorType;

	VectorType ublas_b(2 * pMesh->numOfVertices); 
	MatrixType ublas_A(2 * pMesh->numOfVertices, pMesh->numOfVertices); 

		VCLMatrixType A_GPU(A.size1(), A.size2()); 

		viennacl::copy(A, A_GPU);

		std::vector<ScalarType> ublas_betas = viennacl::linalg::inplace_qr(A_GPU);  

		viennacl::copy(A_GPU, A);

		boost::numeric::ublas::range ublas_range(0, pMesh->numOfVertices);
		boost::numeric::ublas::matrix_range<MatrixType> ublas_R(ublas_A, ublas_range, ublas_range);
		boost::numeric::ublas::vector_range<VectorType> ublas_b2(ublas_b, ublas_range);

		// X //

		#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - QR - get X");
		#endif

		std::copy(Bvec_x.begin(), Bvec_x.end(), ublas_b.begin());

		for (int i=0; i < A.size1(); i++)
			for (int j=0; j < A.size2(); j++){
				ublas_A(i,j) = A(i,j);
			}

		viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);
		

		boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

		std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_x.begin());

		#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - QR - get Y");
		#endif
		
		// Y //

		std::copy(Bvec_y.begin(), Bvec_y.end(), ublas_b.begin());
		
		for (int i=0; i < A.size1(); i++)
			for (int j=0; j < A.size2(); j++){
				ublas_A(i,j) = A(i,j);
			}

		viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


		boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

		std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_y.begin());

		#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - QR - get Z");
		#endif

		// Z //

		std::copy(Bvec_z.begin(), Bvec_z.end(), ublas_b.begin());
		
		for (int i=0; i < A.size1(); i++)
			for (int j=0; j < A.size2(); j++){
				ublas_A(i,j) = A(i,j);
			}

 
		viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


		boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

		std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_z.begin());

	
		#ifdef _LOG
			timerlog.addEnd();
			logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - solve LV system");
		#endif


	for (int i = 0; i < pMesh->numOfVertices; i++)
		pMesh->pVerts[i] = CVector3((float)Vvec_x(i), (float)Vvec_y(i), (float)Vvec_z(i));

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");

		timerlog.logExecutionTimes();
	#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphGPUVCL_LSM_SDF(MeshGraph * pMesh, CVector3* centerPoints, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){

	/*
	Zakladna pointa tejto metody je, ze okrem zakladnych 2ch rovnic kje jedna nuluje laplacian a druha holduje vrcholy pribudne tretia:
	X'[i] = X[i] + SDF[i]
	sdf hodnota sa ma rovnat rozdielu novovypocitaneho vrcholu - povodneho
	*/

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphGPU");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenerates");
	#endif

	using namespace boost::numeric::ublas;

	// setup ViennaCL context and device
	viennacl::ocl::set_context_device_type(0, CL_DEVICE_TYPE_GPU);

	matrix<float> L(pMesh->numOfVertices, pMesh->numOfVertices);
	L.clear();
	matrix<float> WL(pMesh->numOfVertices, pMesh->numOfVertices);
	WL.clear();
	matrix<float> WH(pMesh->numOfVertices, pMesh->numOfVertices);
	WH.clear();
	matrix<float> WC(pMesh->numOfVertices, pMesh->numOfVertices);
	WC.clear();
	matrix<float> UPA(pMesh->numOfVertices, pMesh->numOfVertices);
	UPA.clear();
	matrix<float> LV(pMesh->numOfVertices, pMesh->numOfVertices);
	LV.clear();
	matrix<float> O(pMesh->numOfVertices, pMesh->numOfVertices);
	O.clear();

	// compute Laplace operator from new points

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - construction of Laplacian operator");
	#endif

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	Array2D< float > Ltnt = calculateLaplacianMatrix(pMesh);

	for (int i = 0; i < pMesh->numOfVertices; i++){
		for (int j = 0; j < pMesh->numOfVertices; j++){
			L(i,j) = Ltnt[i][j];
		}
	}

			float maxL = -FLT_MAX;
			float minL = FLT_MAX;

			for (int i = 0; i < pMesh->numOfVertices; i++){
				float sum = 0.0;
				for (int j = 0; j < pMesh->numOfVertices; j++){
					if (pMesh->E[i][j])
						sum -= L(i,j);
				}

				L(i,i) = sum;
			}
		#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica L");

				timerlog.addEnd();

				timerlog.addStart("compute onering areas");

				timerlog.addEnd();

				logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - construction of Laplacian operator");

				logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - preparing of weights matrices");

				timerlog.addStart("prepare submatrices - WH");
		#endif

			for (int i = 0; i < pMesh->numOfVertices; i++) {
				WL(i,i) = pMesh->wL;

				#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "one ring area pre vrchol ", i);
				logg.log(LOG_LEVEL_DUMP, "cur ", curOneRingArea[i]);
				logg.log(LOG_LEVEL_DUMP, "orig ", pMesh->origOneRingArea[i]);
				#endif

				WH(i,i) = pMesh->wH[i];
				WC(i,i) = pMesh->wC;
			}
			//delete[] curOneRingArea;

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica wL");
				//	 logg.log(LOG_LEVEL_DUMP, WL);

				logg.log(LOG_LEVEL_DUMP, "Matica wH");
				// logg.log(LOG_LEVEL_DUMP, WH);


				timerlog.addEnd();
				timerlog.addStart("prepare submatrices - A");

				//	 logg.log(LOG_LEVEL_DUMP, UPA);
			#endif

			matrix<float> A(3 * pMesh->numOfVertices, pMesh->numOfVertices);
			//matrix<float> B(2 * pMesh->numOfVertices, 3);
			boost::numeric::ublas::vector<float> Bvec_x(3 * pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_y(3 * pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_z(3 * pMesh->numOfVertices);
			Bvec_x.clear();
			Bvec_y.clear();
			Bvec_z.clear();

			// now solve the system using TNT and JAMA with QR decomposition

			for (int j = 0; j < pMesh->numOfVertices; j++){
				for (int i = 0; i < pMesh->numOfVertices; i++){
					A(i,j) = (double)L(i,j);
					A(pMesh->numOfVertices + i,j) = (double)WH(i,j);
					A(2 * pMesh->numOfVertices + i,j) = (double)WC(i,j);
				}
			}

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica A");
				//logg.log(LOG_LEVEL_DUMP, A);

				timerlog.addEnd();
				timerlog.addStart("prepare submatrices - B");
			#endif

			
			for (int i = 0; i < pMesh->numOfVertices; i++){
				Bvec_x(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
				Bvec_y(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
				Bvec_z(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
			}

			for (int i = 0; i < pMesh->numOfVertices; i++){
				Bvec_x(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].x * pMesh->wC;
				Bvec_y(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].y * pMesh->wC;
				Bvec_z(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].z * pMesh->wC;
			}
			// X

			#ifdef _LOG
				logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - preparing of weights matrices");

				logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - solve LV system");
			#endif

			boost::numeric::ublas::vector<float> Vvec_x(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Vvec_y(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Vvec_z(pMesh->numOfVertices);

			#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - inplace QR - get betas");
			#endif

			typedef float               ScalarType; 
			typedef boost::numeric::ublas::matrix<ScalarType>              MatrixType;
			typedef boost::numeric::ublas::vector<ScalarType>              VectorType;
			typedef viennacl::matrix<ScalarType, viennacl::column_major>   VCLMatrixType;
			typedef viennacl::vector<ScalarType>                           VCLVectorType;

			VectorType ublas_b(3 * pMesh->numOfVertices); 
			MatrixType ublas_A(3 * pMesh->numOfVertices, pMesh->numOfVertices); 
			
			VCLMatrixType A_GPU(A.size1(), A.size2()); 

			viennacl::copy(A, A_GPU);

			std::vector<ScalarType> ublas_betas = viennacl::linalg::inplace_qr(A_GPU);  

			viennacl::copy(A_GPU, A);

			boost::numeric::ublas::range ublas_range(0, pMesh->numOfVertices);
			boost::numeric::ublas::matrix_range<MatrixType> ublas_R(ublas_A, ublas_range, ublas_range);
			boost::numeric::ublas::vector_range<VectorType> ublas_b2(ublas_b, ublas_range);

			// X //

			#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - QR - get X");
			#endif

			std::copy(Bvec_x.begin(), Bvec_x.end(), ublas_b.begin());

			for (int i=0; i < A.size1(); i++)
				for (int j=0; j < A.size2(); j++){
					ublas_A(i,j) = A(i,j);
				}

				viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


				boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

				std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_x.begin());

				#ifdef _LOG
					timerlog.addEnd();
					timerlog.addStart("viennaCL - QR - get Y");
				#endif

				// Y //

				std::copy(Bvec_y.begin(), Bvec_y.end(), ublas_b.begin());

				for (int i=0; i < A.size1(); i++)
					for (int j=0; j < A.size2(); j++){
						ublas_A(i,j) = A(i,j);
					}

					viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


					boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

					std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_y.begin());

					#ifdef _LOG
						timerlog.addEnd();
						timerlog.addStart("viennaCL - QR - get Z");
					#endif

					// Z //

					std::copy(Bvec_z.begin(), Bvec_z.end(), ublas_b.begin());

					for (int i=0; i < A.size1(); i++)
						for (int j=0; j < A.size2(); j++){
							ublas_A(i,j) = A(i,j);
						}


						viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


						boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

						std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_z.begin());


						#ifdef _LOG
							timerlog.addEnd();
							logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - solve LV system");
						#endif

						// compose new positions

						for (int i = 0; i < pMesh->numOfVertices; i++)
							pMesh->pVerts[i] = CVector3((float)Vvec_x(i), (float)Vvec_y(i), (float)Vvec_z(i));

						#ifdef _LOG
							logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");

							timerlog.logExecutionTimes();
						#endif

}

//---------------------------------------------------------------------------

void contractMeshGraphGPUVCL_LSM_SDF_C(MeshGraph * pMesh, CVector3* centerPoints, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea){

	// vertices are controcted only to SDF center, Laplacian smoothing is noot used...

	/*
	Zakladna pointa tejto metody je, ze okrem zakladnych 2ch rovnic kje jedna nuluje laplacian a druha holduje vrcholy pribudne tretia:
	X'[i] = X_0[i] + SDF[i] / 2
	sdf hodnota sa ma rovnat rozdielu novovypocitaneho vrcholu - povodneho
	*/

	#ifdef _LOG
		Timerlog timerlog = Timerlog("contractMeshGraphGPU");
	#endif

	if (pMesh->numOfVertices < 3){
		#ifdef _LOG
			logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph SKIPPED: pMesh->numOfVertices < 3");
		#endif
		return;
	}

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph STARTED");

		timerlog.addStart("check degenerates");
	#endif

	using namespace boost::numeric::ublas;

	// setup ViennaCL context and device
	viennacl::ocl::set_context_device_type(0, CL_DEVICE_TYPE_GPU);

	matrix<float> L(pMesh->numOfVertices, pMesh->numOfVertices);
	L.clear();
	matrix<float> WL(pMesh->numOfVertices, pMesh->numOfVertices);
	WL.clear();
	matrix<float> WH(pMesh->numOfVertices, pMesh->numOfVertices);
	WH.clear();
	matrix<float> WC(pMesh->numOfVertices, pMesh->numOfVertices);
	WC.clear();
	matrix<float> UPA(pMesh->numOfVertices, pMesh->numOfVertices);
	UPA.clear();
	matrix<float> LV(pMesh->numOfVertices, pMesh->numOfVertices);
	LV.clear();
	matrix<float> O(pMesh->numOfVertices, pMesh->numOfVertices);
	O.clear();

	// compute Laplace operator from new points

	#ifdef _LOG
		logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - construction of Laplacian operator");
	#endif

	collapseDegenerates(pMesh, mgDegeneratesMapping);

	#ifdef _LOG
		timerlog.addEnd();

		timerlog.addStart("construct L");
	#endif

	Array2D< float > Ltnt = calculateLaplacianMatrix(pMesh);

	for (int i = 0; i < pMesh->numOfVertices; i++){
		for (int j = 0; j < pMesh->numOfVertices; j++){
			L(i,j) = Ltnt[i][j];
		}
	}

			float maxL = -FLT_MAX;
			float minL = FLT_MAX;

			for (int i = 0; i < pMesh->numOfVertices; i++){
				float sum = 0.0;
				for (int j = 0; j < pMesh->numOfVertices; j++){
					if (pMesh->E[i][j])
						sum -= L(i,j);
				}

				L(i,i) = sum;
			}

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica L");

				timerlog.addEnd();

				timerlog.addStart("compute onering areas");

				timerlog.addEnd();

				logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - construction of Laplacian operator");

				logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - preparing of weights matrices");

				timerlog.addStart("prepare submatrices - WH");
			#endif

			for (int i = 0; i < pMesh->numOfVertices; i++) {
				//WL(i,i) = pMesh->wL;

				#ifdef _LOG
					logg.log(LOG_LEVEL_DUMP, "one ring area pre vrchol ", i);
					logg.log(LOG_LEVEL_DUMP, "cur ", curOneRingArea[i]);
					logg.log(LOG_LEVEL_DUMP, "orig ", pMesh->origOneRingArea[i]);
				#endif

				WH(i,i) = pMesh->wH[i];
				WC(i,i) = pMesh->wC;
			}
			//delete[] curOneRingArea;

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica wL");
				//	 logg.log(LOG_LEVEL_DUMP, WL);

				logg.log(LOG_LEVEL_DUMP, "Matica wH");
				// logg.log(LOG_LEVEL_DUMP, WH);


				timerlog.addEnd();
				timerlog.addStart("prepare submatrices - A");

				//	 logg.log(LOG_LEVEL_DUMP, UPA);
			#endif

			matrix<float> A(pMesh->numOfVertices, pMesh->numOfVertices);//matrix<float> A(3 * pMesh->numOfVertices, pMesh->numOfVertices);
			//matrix<float> B(2 * pMesh->numOfVertices, 3);
			/*boost::numeric::ublas::vector<float> Bvec_x(3 * pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_y(3 * pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_z(3 * pMesh->numOfVertices);*/
			boost::numeric::ublas::vector<float> Bvec_x(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_y(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Bvec_z(pMesh->numOfVertices);
			Bvec_x.clear();
			Bvec_y.clear();
			Bvec_z.clear();

			// now solve the system using TNT and JAMA with QR decomposition

			/*for (int j = 0; j < pMesh->numOfVertices; j++){
				for (int i = 0; i < pMesh->numOfVertices; i++)
					A(i,j) = (double)L(i,j);
				for (int i = 0; i < pMesh->numOfVertices; i++)
					A(pMesh->numOfVertices + i,j) = (double)WH(i,j);
				for (int i = 0; i < pMesh->numOfVertices; i++)
					A(2 * pMesh->numOfVertices + i,j) = (double)WC(i,j);
			}*/
			for (int j = 0; j < pMesh->numOfVertices; j++){
				for (int i = 0; i < pMesh->numOfVertices; i++)
					A(i,j) = (double)WC(i,j);
			}

			#ifdef _LOG
				logg.log(LOG_LEVEL_DUMP, "Matica A");
				//logg.log(LOG_LEVEL_DUMP, A);

				timerlog.addEnd();
				timerlog.addStart("prepare submatrices - B");
			#endif

			
			/*for (int i = 0; i < pMesh->numOfVertices; i++){
				Bvec_x(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].x * pMesh->wH[i]);
				Bvec_y(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].y * pMesh->wH[i]);
				Bvec_z(pMesh->numOfVertices + i) = (double)(pMesh->pVerts[i].z * pMesh->wH[i]);
			}

			for (int i = 0; i < pMesh->numOfVertices; i++){
				Bvec_x(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].x * pMesh->wC;
				Bvec_y(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].y * pMesh->wC;
				Bvec_z(2 * pMesh->numOfVertices + i) = (double)centerPoints[i].z * pMesh->wC;
			}*/
			for (int i = 0; i < pMesh->numOfVertices; i++){
				Bvec_x(i) = (double)centerPoints[i].x * pMesh->wC;
				Bvec_y(i) = (double)centerPoints[i].y * pMesh->wC;
				Bvec_z(i) = (double)centerPoints[i].z * pMesh->wC;
			}
			// X

			#ifdef _LOG
				logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - preparing of weights matrices");

				logg.log(LOG_LEVEL_METHODSTARTEND, "START contractMeshGraph - solve LV system");
			#endif

			boost::numeric::ublas::vector<float> Vvec_x(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Vvec_y(pMesh->numOfVertices);
			boost::numeric::ublas::vector<float> Vvec_z(pMesh->numOfVertices);

			#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - inplace QR - get betas");
			#endif

			typedef float               ScalarType; 
			typedef boost::numeric::ublas::matrix<ScalarType>              MatrixType;
			typedef boost::numeric::ublas::vector<ScalarType>              VectorType;
			typedef viennacl::matrix<ScalarType, viennacl::column_major>   VCLMatrixType;
			typedef viennacl::vector<ScalarType>                           VCLVectorType;

			//VectorType ublas_b(3 * pMesh->numOfVertices); 
			//MatrixType ublas_A(3 * pMesh->numOfVertices, pMesh->numOfVertices); 
			VectorType ublas_b(pMesh->numOfVertices); 
			MatrixType ublas_A(pMesh->numOfVertices, pMesh->numOfVertices); 

			VCLMatrixType A_GPU(A.size1(), A.size2()); 

			viennacl::copy(A, A_GPU);

			std::vector<ScalarType> ublas_betas = viennacl::linalg::inplace_qr(A_GPU);  

			viennacl::copy(A_GPU, A);

			boost::numeric::ublas::range ublas_range(0, pMesh->numOfVertices);
			boost::numeric::ublas::matrix_range<MatrixType> ublas_R(ublas_A, ublas_range, ublas_range);
			boost::numeric::ublas::vector_range<VectorType> ublas_b2(ublas_b, ublas_range);

			// X //

			#ifdef _LOG
				timerlog.addEnd();
				timerlog.addStart("viennaCL - QR - get X");
			#endif

			std::copy(Bvec_x.begin(), Bvec_x.end(), ublas_b.begin());

			for (int i=0; i < A.size1(); i++)
				for (int j=0; j < A.size2(); j++){
					ublas_A(i,j) = A(i,j);
				}

				viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


				boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

				std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_x.begin());

				#ifdef _LOG
					timerlog.addEnd();
					timerlog.addStart("viennaCL - QR - get Y");
				#endif

				// Y //

				std::copy(Bvec_y.begin(), Bvec_y.end(), ublas_b.begin());

				for (int i=0; i < A.size1(); i++)
					for (int j=0; j < A.size2(); j++){
						ublas_A(i,j) = A(i,j);
					}

					viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


					boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

					std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_y.begin());

					#ifdef _LOG
						timerlog.addEnd();
						timerlog.addStart("viennaCL - QR - get Z");
					#endif

					// Z //

					std::copy(Bvec_z.begin(), Bvec_z.end(), ublas_b.begin());

					for (int i=0; i < A.size1(); i++)
						for (int j=0; j < A.size2(); j++){
							ublas_A(i,j) = A(i,j);
						}


						viennacl::linalg::inplace_qr_apply_trans_Q(ublas_A, ublas_betas, ublas_b);


						boost::numeric::ublas::inplace_solve(ublas_R, ublas_b2, boost::numeric::ublas::upper_tag()); 

						std::copy(ublas_b2.begin(), ublas_b2.end(), Vvec_z.begin());


						#ifdef _LOG
							timerlog.addEnd();
						#endif

						// compostion of new positions


						#ifdef _LOG
							logg.log(LOG_LEVEL_METHODSTARTEND, "END contractMeshGraph - solve LV system");
						#endif


						for (int i = 0; i < pMesh->numOfVertices; i++)
							pMesh->pVerts[i] = CVector3((float)Vvec_x(i), (float)Vvec_y(i), (float)Vvec_z(i));

						#ifdef _LOG
							logg.log(LOG_LEVEL_METHODSTARTEND, "METHOD contractMeshGraph ENDED");

							timerlog.logExecutionTimes();
						#endif
}

#pragma package(smart_init)

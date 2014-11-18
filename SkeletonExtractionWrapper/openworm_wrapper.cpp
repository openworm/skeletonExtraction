#include "openworm_wrapper.h"

#define NUM_OF_CTRL_BONES 2
#define MAX_BONE_MAT 24

#define MAX_WORM_ITERATIONS 4000

// main thread calls

Openworm_Wrapper::Openworm_Wrapper(){
	wantedNumOfSkeletonNodes = MAX_BONE_MAT;
}

void Openworm_Wrapper::prepareGLDDataForSkinningPreview(){
	//prepareGLDDataForSimulationPreview has to be called before this

	// prepare data


	// load skinning weights and indices


	// send skinning data to GPU

}

void Openworm_Wrapper::prepareGLDDataForSimulationPreview(string positionsFile, string membraneFile){
	wormLoader.SetWormFile(positionsFile);
	wormLoader.SetWormMeshFile(membraneFile);

	#ifdef _GLD
		glGenBuffers(1, &meshToAddGLDVerticesId);
	#endif

	mesh = new meshes::IndexedFace();

	wormLoader.ReadWormIteration(mesh, 0);
	wormLoader.ReadWormMesh(mesh);

	int NumOfV = mesh->vertices.size()/3;
	if (NumOfV>0)
	{
		GLfloat* GLvertices = &mesh->vertices[0];

		#ifdef _GLD
			glGenBuffers(1, &meshToAddGLDVerticesId);
			glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDVerticesId);
			glBufferData(GL_ARRAY_BUFFER, NumOfV * 3 * sizeof(float), GLvertices, GL_STATIC_DRAW);
		#endif

		#ifdef _GLD
			gldAddModel("ComparisionSimulationModel", NumOfV, meshToAddGLDVerticesId);
		#endif

		/*GLint * GLindices = &mesh->indices[0];

		glGenBuffers(1, &meshToAddGLDIndicesId);
		glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDIndicesId);
		glBufferData(GL_ARRAY_BUFFER, mesh->indices.size() * sizeof(int), GLindices, GL_STATIC_DRAW);

		gldAddModelEdges("ComparisionSimulationModel", GL_TRIANGLES, mesh->indices.size(), meshToAddGLDIndicesId, GL_UNSIGNED_INT);*/
	}

	animTimestep = 0;
	timeStep = 10;
	simulationDirection = 1;

	#ifdef _GLD
		gldAddLine("simulation iteration %i: ", &animTimestep);
	#endif

	//prepareGLDDataForSkinningPreview has tobe called after this
}

void Openworm_Wrapper::playSimulationAndSkinningPreview(){
	previewPaused = false;
}

void Openworm_Wrapper::pauseSimulationAndSkinningPreview(){
	previewPaused = true;
}

void Openworm_Wrapper::updateSimulationModelFromMainThread(){

	int NumOfV = meshToAdd.vertices.size()/3;
	if (NumOfV>0)
	{
		#ifdef _GLD
			glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDVerticesId);
			glBufferData(GL_ARRAY_BUFFER, NumOfV * 3 * sizeof(float), &meshToAdd.vertices[0], GL_STATIC_DRAW);
		#endif
	}

}

// animation thread calls

void Openworm_Wrapper::calculateAndExportSkeletonTransformations(string filePath, Export::ColladaExporter * exporter){

	// go through all timesteps, compute skeletons and export transformations

	int bindPoseTimeStep = 0;
	timeStep = 0;

	oLBSExtractor.sdfHalfVectors = new CVector3[0];
	oLBSExtractor.sdfHalfVectorsMG = new CVector3[0];

	pSkeletonRootBindPose = new skl::SkeletonNode();

	calculateAndExportSkeletonTransformationsForTimestep(bindPoseTimeStep, filePath, exporter, pSkeletonRootBindPose);

	while (timeStep < MAX_WORM_ITERATIONS){

		calculateAndExportSkeletonTransformationsForTimestep(timeStep, filePath, exporter, NULL);

		//timeStep = timeStep + 100;
		timeStep++;
	}

	delete[] oLBSExtractor.sdfHalfVectors;
	delete[] oLBSExtractor.sdfHalfVectorsMG;
	
}

void Openworm_Wrapper::skeletonExtractionSDF(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput, meshes::Mesh  * meshImport){

	oLBSExtractor.sdfModelController = new ModelController::CModel();
	oLBSExtractor.pMesh = new MeshGraph();
	skl::SkeletonNode * loadSkelet = new skl::SkeletonNode();

	std::string pFile = std::string(filePath + "collada\\wormCollada_" + std::to_string(timeStep) + ".dae");

	ImportModel(meshImport, NULL, (char*)pFile.c_str(), oLBSExtractor.sdfModelController, loadSkelet);

	//delete pSkeletonRootForTimestep;
	//pSkeletonRootForTimestep = new skl::SkeletonNode();

	//delete oLBSExtractor.pMesh;

	oLBSExtractor.polyMesh = new MeshGraph();
	oLBSExtractor.pSurgeryGraph = new SurgeryGraph();
	boost::unordered_map<int, vector<int> > emptymgDegeneratesMapping;
	oLBSExtractor.mgDegeneratesMapping = emptymgDegeneratesMapping;

	createMeshGraph(meshImport, oLBSExtractor.pMesh, oLBSExtractor.wL, oLBSExtractor.wH);

	//calculateModelMaxDim(oLBSExtractor.pMesh, &adaptVMdmax);

	// calculate skeleton from meshToAdd
	SN::SkeletonNode * extractedSkeleton = new SN::SkeletonNode();

	skeletonExtractionSDF_calculateSDF(meshImport, &oLBSExtractor);
	skeletonExtractionSDF_extractSkeleton(extractedSkeleton, &oLBSExtractor, &oSDFExtractor);

	copySNSkeletonNodeToSkl(extractedSkeleton, skeletonOutput);

	delete extractedSkeleton;
	extractedSkeleton = NULL;

	delete oLBSExtractor.polyMesh;
	oLBSExtractor.polyMesh = NULL;
	delete oLBSExtractor.pSurgeryGraph;
	oLBSExtractor.pSurgeryGraph = NULL;

	delete oLBSExtractor.pMesh;
	oLBSExtractor.pMesh = NULL;
	delete oLBSExtractor.sdfModelController;
	oLBSExtractor.sdfModelController = NULL;	

	delete loadSkelet;
	loadSkelet = NULL;
}

void Openworm_Wrapper::calculateAndExportSkeletonTransformationsForTimestep(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput){

	meshes::Mesh  * meshImport = new meshes::Mesh();
	pSkeletonRootForTimestep = new skl::SkeletonNode();

	skeletonExtractionSDF(timeStep, filePath, exporter, pSkeletonRootForTimestep, meshImport);

	if (skeletonOutput != NULL){
		copySNSkeletonNodeToSkl(pSkeletonRootForTimestep, skeletonOutput);
	}

	// serialize skeleton nodes

	ofstream ofs(filePath + "animation\\skeleton_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + std::to_string(timeStep) + ".skl");
	assert(ofs.good());
	boost::archive::xml_oarchive oa(ofs);
	oa << BOOST_SERIALIZATION_NVP(pSkeletonRootForTimestep);


	// compare differences with bind pose - timeStep 0
	SM::CalculateWormTransformationMatrices(pSkeletonRootBindPose, pSkeletonRootForTimestep);

	// send new skeleton rotations to shader
	Export::SaveQuaternionsToFile(pSkeletonRootForTimestep, filePath + "animation\\quaternion_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + std::to_string(timeStep) + ".qua" );
	Export::SaveAxisAngleToFile(pSkeletonRootForTimestep, filePath + "animation\\quaternion_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + std::to_string(timeStep) + ".exa" );
	Export::SaveMatricesToFile(pSkeletonRootForTimestep, filePath + "animation\\matrix_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + std::to_string(timeStep) + ".mat");

	exporter->Export(meshImport, pSkeletonRootForTimestep, filePath + "animation\\model_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + std::to_string(timeStep) + ".dae");

	delete pSkeletonRootForTimestep;
	pSkeletonRootForTimestep = NULL;
	delete meshImport;
	meshImport = NULL;
}

void Openworm_Wrapper::renderSkinningPreview(){


}

void Openworm_Wrapper::renderSimulationPreview(){
	wormLoader.ReadWormIteration(mesh, animTimestep);
	//wormLoader.ReadWormMesh(mesh);

	addSimulationModel(mesh);
}

void Openworm_Wrapper::renderSimulationAndSkinningPreview(bool * finished, bool force){

	mesh = new meshes::IndexedFace();

	if (animTimestep >= OW_MAX_WORM_ITERATIONS || animTimestep <= 0 && (!previewPaused || force)){
		//animTimestep = 0;
		if (animTimestep <= 0)
			simulationDirection = 1;
		else
			simulationDirection = -1;
	}

	if (!previewPaused || force){

		renderSimulationPreview();
		renderSkinningPreview();

		animTimestep += timeStep * simulationDirection;
	}

	*finished = true;

	delete mesh;
}

void Openworm_Wrapper::addSimulationModel(meshes::IndexedFace * mesh){
	b_addSimulationModel = true;
	meshToAdd.vertices = mesh->vertices;
	meshToAdd.indices = mesh->indices;
	meshToAdd.normals = mesh->normals;
}


void Openworm_Wrapper::skeletonExtractionSDF_calculateSDF(meshes::Mesh * _mesh, lbse::Extractor * pLBSExtractor){
	//calculate sdf half vectors for mesh graph

	int size = 0;

	float * newpos = new float[_mesh->vertices.size()];

	for (int i = 0; i < _mesh->vertices.size(); i++){
		newpos[i] = _mesh->vertices[i];
	}

	pLBSExtractor->sdfModelController->SetNewPositions(newpos);

	pLBSExtractor->sdfModelController->ComputeSDF();

	float * sdf = pLBSExtractor->sdfModelController->GetSDF(size, true); 

	maxSDF = FLT_MIN;
	minSDF = FLT_MAX;

	pLBSExtractor->sdfSize = size;

	vector<float> normalsFromMesh;

	delete[] pLBSExtractor->sdfHalfVectors;
	delete[] pLBSExtractor->sdfHalfVectorsMG;
	pLBSExtractor->sdfHalfVectors = new CVector3[size];
	pLBSExtractor->sdfHalfVectorsMG = new CVector3[pLBSExtractor->pMesh->numOfVertices];

	for (int i=0; i < size; i++){
		float sdfVal = sdf[i];

		pLBSExtractor->sdfHalfVectors[i] = CVector3(_mesh->normals[i * 3], _mesh->normals[i * 3 + 1], _mesh->normals[i * 3 + 2]) * (float)(sdf[i] / -2.0);
		pLBSExtractor->sdfHalfVectorsMG[i] = pLBSExtractor->sdfHalfVectors[i];

		if (sdfVal > maxSDF){
			maxSDF = sdfVal;
		}

		if (sdfVal < minSDF){
			minSDF = sdfVal;
		}
	}

	pLBSExtractor->isSDFvaluesComputed = true;


	/*vector<CVector3> sdfMGHFdebug;
	vector<float> sdfdebug;
	vector<CVector3> normalsdebug;

	for (int i=0; i < size; i++){
		sdfdebug.push_back(sdf[i]);
		normalsdebug.push_back(CVector3(_mesh->normals[i * 3], _mesh->normals[i * 3 + 1], _mesh->normals[i * 3 + 2]));
		sdfMGHFdebug.push_back(pLBSExtractor->sdfHalfVectorsMG[i]);
	}*/

	pLBSExtractor->originalMesh = new MeshGraph();

	copyMeshGraph(pLBSExtractor->pMesh, pLBSExtractor->originalMesh);

	delete[] newpos;

}

void Openworm_Wrapper::skeletonExtractionSDF_extractSkeleton(SN::SkeletonNode * pSkeletonRoot, lbse::Extractor * pLBSExtractor, sdf::Extractor * pSDFExtractor){

	for (int i=0; i < pLBSExtractor->pMesh->numOfVertices; i++){
		pLBSExtractor->pMesh->pVerts[i] = pLBSExtractor->originalMesh->pVerts[i] + pLBSExtractor->sdfHalfVectorsMG[i];

		#ifdef _LOG
			logg.log(0, "shiftnuty vertex", pLBSExtractor->pMesh->pVerts[i]);
		#endif
	}

	if (pLBSExtractor->polyMesh != NULL){
		delete pLBSExtractor->polyMesh;
		pLBSExtractor->polyMesh = NULL;
	}

	float interpolationParam = 0.5;
	float maxIP = 1.0;
	float minIP = 0.0;

	int polyMeshNumOfBones = 0;

	int iteThreshold = 100;
	int ite=0;

	// binary search to find optimal threshold for extraction of desired number of skeleton nodes

	while ((polyMeshNumOfBones != wantedNumOfSkeletonNodes) & (ite < iteThreshold)){
		pLBSExtractor->polyMesh = new MeshGraph();

		float threshold = (minSDF * (1.0 - interpolationParam) + maxSDF * (interpolationParam));
		pSDFExtractor->applyMedianMerge(pLBSExtractor->polyMesh, pLBSExtractor->pMesh, threshold);

		polyMeshNumOfBones = pLBSExtractor->polyMesh->numOfVertices;

		if ((polyMeshNumOfBones != wantedNumOfSkeletonNodes) && (ite < iteThreshold)){

			if (polyMeshNumOfBones > wantedNumOfSkeletonNodes){
				minIP = interpolationParam;
				interpolationParam = (interpolationParam + maxIP) / 2.0;
			}
			if (polyMeshNumOfBones < wantedNumOfSkeletonNodes){
				maxIP = interpolationParam;
				interpolationParam = (interpolationParam + minIP) / 2.0;
			}

			delete pLBSExtractor->polyMesh;
		}

		ite++;
	}
	
	pLBSExtractor->applyConnectivitySurgery(false, pSkeletonRoot, adaptVMdmax);

	pSDFExtractor->subdivideSkeletonByCrossSectionMedians(pSkeletonRoot, pLBSExtractor->pMesh, pSDFExtractor->skeletonTesselationFactor);
}


void Openworm_Wrapper::loadModelToStructures(lbse::Extractor * pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance){
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

void Openworm_Wrapper::calculateModelMaxDim(MeshGraph * pMesh, float * adaptVMdmax){

	BoundingBox modelbb;

	for (int i=0; i < pMesh->numOfVertices; i++){
		CVector3 * vert = &(pMesh->pVerts[i]);
		if (vert->x > modelbb.x_max)
			modelbb.x_max = vert->x;
		if (vert->x < modelbb.x_min)
			modelbb.x_min = vert->x;
		if (vert->y > modelbb.y_max)
			modelbb.y_max = vert->y;
		if (vert->y < modelbb.y_min)
			modelbb.y_min = vert->y;
		if (vert->z > modelbb.z_max)
			modelbb.z_max = vert->z;
		if (vert->z < modelbb.z_min)
			modelbb.z_min = vert->z;
	}

	float dx = modelbb.x_max - modelbb.x_min;
	float dy = modelbb.y_max - modelbb.y_min;
	float dz = modelbb.z_max - modelbb.z_min;

	// Maximal size
	*adaptVMdmax = dx > dy ? dx : dy;
	*adaptVMdmax = *adaptVMdmax > dz ? *adaptVMdmax : dz;
}

void Openworm_Wrapper::calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat){
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

void Openworm_Wrapper::extractMuscleSkeleton(meshes::IndexedFace * muscle, skl::SkeletonNode * pSkelet){

	int numOfVerts = muscle->vertices.size() / 3;
	CVector3 n;
	CVector3 ev_1;
	CVector3 ev_2;
	CVector3 ev_3;

	CVector3 * points = new CVector3[numOfVerts];
	CVector3 * projectedPoints = new CVector3[numOfVerts];
	float * lengths = new float[numOfVerts];

	for (int i=0; i < numOfVerts; i++){
		points[i] = CVector3(muscle->vertices[i * 3], muscle->vertices[i * 3 + 1], muscle->vertices[i * 3 + 2]);
	}

	getTangentPlanePCA_orig(numOfVerts, points, &n, &ev_1, &ev_2, &ev_3);

	// get center of mass

	CVector3 com;

	for (int i=0; i < numOfVerts; i++){
		com = com + points[i];
	}

	com = com / numOfVerts;


	// project points onto the ev_1

	CVector3 s = ev_1;

	for (int i=0; i < numOfVerts; i++){
		CVector3 p = CVector3(muscle->vertices[i * 3], muscle->vertices[i * 3 + 1], muscle->vertices[i * 3 + 2]) - com;
		lengths[i] = (Dot(p,s));
		projectedPoints[i] = com + s * lengths[i];		
	}

	float minLength = FLT_MAX;
	float maxLength = FLT_MIN;
	int minIdx = -1;
	int maxIdx = -1;

	for (int i=0; i < numOfVerts; i++){
		if (lengths[i] < minLength){
			minLength = lengths[i];
			minIdx = i;
		}
		if (lengths[i] > maxLength){
			maxLength = lengths[i];
			maxIdx = i;
		}
	}

	// set root

	pSkelet->point = com;
	pSkelet->id = 0;

	// set two childs

	if (minIdx != -1){
		skl::SkeletonNode * node1 = new skl::SkeletonNode();;
		node1->point = com + s * minLength;
		node1->id = 1;
		pSkelet->nodes.push_back(node1);
		node1->father = pSkelet;
	}

	if (maxIdx != -1){
		skl::SkeletonNode  * node2 = new skl::SkeletonNode;
		node2->point = com + s * maxLength;
		node2->id = 2;
		pSkelet->nodes.push_back(node2);
		node2->father = pSkelet;
	}

	delete[] points;
	delete[] projectedPoints;
	delete[] lengths;

}

void Openworm_Wrapper::calculateWormSimulationSkeletonsFromMeshes(string dir){
/*******************************************************************************************************************************/
//                                    PATHS
/*******************************************************************************************************************************/

	string positionsFile;
	string membraneFile;
	string colladaMeshFile;
	string colladaMeshAndSkeletonFile;
	string colladaAnimationFile;

	string transformationsMatrixFile;
	string transformationsQuaternionFile;

/*******************************************************************************************************************************/
//                                    BASIC OBJECTS
/*******************************************************************************************************************************/

	meshes::IndexedFace * mesh = new meshes::IndexedFace();

	Import::WormLoader wormLoader;
	Export::ColladaExporter exporter;

/*******************************************************************************************************************************/
//                                    SETTINGS, PARAMS ...
/*******************************************************************************************************************************/

	float joiningTolerance = 0.0;
	//oLBSExtractor.laplacianScheme = LS_GLOBAL_JAMA_COTANGENT;
	//oLBSExtractor.numOfIter = 4;
	oLBSExtractor.groupingTolerance = 2.5;
/*	oLBSExtractor.wA = 1.0;
	oLBSExtractor.wB = 0.1;
	oLBSExtractor.wL = 1.0;
	oLBSExtractor.wC = 1.0;
	oLBSExtractor.wH = 1.0;
	oLBSExtractor.sL = 3.0;*/
	oLBSExtractor.wantedNumOfBones = 24;

	bool calculateSkinningWeights = false;
	bool exportBindMeshWithWeights = false;
	bool exportSkeletonsAndTransformation = true;

/*******************************************************************************************************************************/
	// [1] Read the worm particles w/wo triangulation and transform it into meshgraph stucture (using importer)
/*******************************************************************************************************************************/

	// - call Loader_lib, load t3DModel

	wormLoader.SetWormFile(positionsFile);
	wormLoader.SetWormMeshFile(membraneFile);

	wormLoader.ReadWormIteration(mesh, OW_ITE_TO_LOAD);
	wormLoader.ReadWormMesh(mesh);

	createMeshGraph(mesh, oLBSExtractor.pMesh, oLBSExtractor.wL, oLBSExtractor.wH);

	structure::t3DModel g_3DModel;
	int adaptVMdmax = 0;

	// load model from Worm file, timestep 1

	//loadModelToStructures(&oLBSExtractor, &g_3DModel, &adaptVMdmax, joiningTolerance);


/*******************************************************************************************************************************/
	// [2] Perform SDF median skeleton extraction on the meshgraph
/*******************************************************************************************************************************/

	//skeletonExtractionSDF(pSkeletonRootBindPose, &oLBSExtractor, &oSDFExtractor);
	
/*******************************************************************************************************************************/
	// [3] Calculate skinning weights using inverse geodesic distance
/*******************************************************************************************************************************/

	if (calculateSkinningWeights){

		// - calculate weights using GDSW_lib

		GraphAlgorithms ga;
		Array2D<float> distanceMatrix;

		ga.FloydWarshall(oLBSExtractor.pMesh, distanceMatrix);

		ObjectSkeletonShaderData wormData;

		calculateSkinningData(&ga, &wormData, &g_3DModel, pSkeletonRootBindPose, &distanceMatrix, &oLBSExtractor, NUM_OF_CTRL_BONES, MAX_BONE_MAT);

/*******************************************************************************************************************************/
	// [3] Export mesh and mesh with extracted skeleton and weights into Collada (using Collada exporter)
/*******************************************************************************************************************************/

		if (exportBindMeshWithWeights){

			// - call Exporter_lib to store mesh and skinning data in .dae

			// export mesh only
			exporter.Export(mesh, colladaMeshFile);

			// export mesh and skinning data
			meshes::MeshSkin * meshSkin = new meshes::MeshSkin(mesh);

			for (int i=0; i < meshSkin->indices.size() / 3; i++){
				meshSkin->influences.push_back(NUM_OF_CTRL_BONES);
			}

			for (int i=0; i < (meshSkin->indices.size() / 3) * NUM_OF_CTRL_BONES; i++){
				meshSkin->jointIDs.push_back(wormData.indices[i]);
			}

			for (int i=0; i < (meshSkin->indices.size() / 3) * NUM_OF_CTRL_BONES; i++){
				meshSkin->skinWeights.push_back(wormData.weights[i]);
			}

			exporter.Export(meshSkin, pSkeletonRootBindPose, colladaMeshAndSkeletonFile);
		}

		delete[] wormData.indices;
		delete[] wormData.weights;
	}

/*******************************************************************************************************************************/
//	Loop through all theE timesteps of the simulation - extract skeletons and compute skeleton transformations
/*******************************************************************************************************************************/
	if (exportSkeletonsAndTransformation){
		calculateAndExportSkeletonTransformations(dir, &exporter);
	}

}
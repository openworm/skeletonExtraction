#include "openworm_wrapper.h"
#include <glm\custom\aditions.h>

#define NUM_OF_CTRL_BONES 2
#define MAX_BONE_MAT 24

#define MAX_WORM_ITERATIONS 4000

#define WORM_GRAVITATION_CONSTANT 0.012

// main thread calls

Openworm_Wrapper::Openworm_Wrapper(){
	wantedNumOfSkeletonNodes = OW_NUMNODES_BEFORETES;
	initSkinning = false;
	skinningGPU = true;

	b_muscleReading = true;
	b_cuticleReading = true;
	b_addSkinningSkeleton = false;
	b_addSimulationModel = false;
	b_addSimulationMuscleModel = false;

	mesh = new meshes::IndexedFace();
	muscleMesh = new meshes::MeshSkin();
}

Openworm_Wrapper::~Openworm_Wrapper(){
	delete mesh;
	delete muscleMesh;
}

void Openworm_Wrapper::loadAndPrepareSkinningData(meshes::SerializableMeshSkin * serMesh, ObjectSkeletonShaderData * skeletonData){

	int idx = 0;
	int idxFile = 0;
	for (int i=0; i < skeletonData->numOfVertices; i++){
		int inf = serMesh->influences[i];
		for (int j=0; j < 4; j++){
			if (j == 0 || j == 1){
				skeletonData->indices[idx] = serMesh->jointIDs[idxFile];
				skeletonData->weights[idx] = serMesh->skinWeights[idxFile];
				idxFile++;
			} else {
				skeletonData->weights[idx] = 0.0;
			}
			idx++;
		}
	}

}

void Openworm_Wrapper::loadCuticleModelIntoBuffer(meshes::MeshSkin * model){

#ifdef _GLD

	glGenBuffers(1, &(verticesBO));
	glBindBuffer(GL_ARRAY_BUFFER, verticesBO);
	glBufferData(GL_ARRAY_BUFFER,model->vertices.size() * sizeof(float), &(model->vertices[0]), GL_STATIC_DRAW);

	mergedVertices.insert(mergedVertices.end(), model->vertices.begin(), model->vertices.end());

	gldAddModel("Skinning cuticle", model->vertices.size() / 3, verticesBO);

	if (model->indices.size() > 0){
		glGenBuffers(1, &trianglesBO);
		glBindBuffer(GL_ARRAY_BUFFER, trianglesBO);
		glBufferData(GL_ARRAY_BUFFER, model->indices.size() * sizeof(int), &(model->indices[0]), GL_STATIC_DRAW);

		mergedTriangles.insert(mergedTriangles.end(), model->indices.begin(), model->indices.end());

		gldAddModelEdges("Skinning cuticle", GL_TRIANGLES, model->indices.size(), trianglesBO, GL_UNSIGNED_INT);
	}

	if (model->jointIDs.size() > 0 && model->skinWeights.size() > 0){

		vector<int> floatIndices;

		for (int i=0; i<model->jointIDs.size(); i++){
			floatIndices.push_back(model->jointIDs[i]);		
		}

		glGenBuffers(1, &indicesBO);
		glBindBuffer(GL_ARRAY_BUFFER, indicesBO);
		glBufferData(GL_ARRAY_BUFFER, model->jointIDs.size() * sizeof(int),  &(floatIndices[0]), GL_STATIC_DRAW);

		glGenBuffers(1, &weightsBO);
		glBindBuffer(GL_ARRAY_BUFFER, weightsBO);
		glBufferData(GL_ARRAY_BUFFER, model->skinWeights.size() * sizeof(float), &(model->skinWeights[0]), GL_STATIC_DRAW);

		mergedJointIDs.insert(mergedJointIDs.end(), model->jointIDs.begin(), model->jointIDs.end());
		mergedWeights.insert(mergedWeights.end(), model->skinWeights.begin(), model->skinWeights.end());

		if (skinningGPU){
			gldAddModelSkinningData("Skinning cuticle", indicesBO, weightsBO);
		} else {
			gldAddModelSkinningData("Skinning cuticle", 0, 0);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);	

#endif
}

void Openworm_Wrapper::loadMuscleModelIntoBuffer(meshes::MeshSkin * model){

#ifdef _GLD

	glGenBuffers(1, &(verticesMuscleBO));
	glBindBuffer(GL_ARRAY_BUFFER, verticesMuscleBO);
	glBufferData(GL_ARRAY_BUFFER, model->vertices.size() * sizeof(float), &(model->vertices[0]), GL_STATIC_DRAW);

	mergedVertices.insert(mergedVertices.end(), model->vertices.begin(), model->vertices.end());

	gldAddModel("Skinning muscles", model->vertices.size() / 3, verticesMuscleBO);

	if (model->indices.size() > 0){
		glGenBuffers(1, &trianglesMuscleBO);
		glBindBuffer(GL_ARRAY_BUFFER, trianglesMuscleBO);
		glBufferData(GL_ARRAY_BUFFER, model->indices.size() * sizeof(int), &(model->indices[0]), GL_STATIC_DRAW);

		mergedTriangles.insert(mergedTriangles.end(), model->indices.begin(), model->indices.end());

		gldAddModelEdges("Skinning muscles", GL_TRIANGLES, model->indices.size(), trianglesMuscleBO, GL_UNSIGNED_INT);
	}

	if (model->jointIDs.size() > 0 && model->skinWeights.size() > 0){

		vector<int> floatIndices;

		for (int i=0; i<model->jointIDs.size(); i++){
			floatIndices.push_back(model->jointIDs[i]);		
		}

		glGenBuffers(1, &indicesMuscleBO);
		glBindBuffer(GL_ARRAY_BUFFER, indicesMuscleBO);
		glBufferData(GL_ARRAY_BUFFER, model->jointIDs.size() * sizeof(int),  &(floatIndices[0]), GL_STATIC_DRAW);

		glGenBuffers(1, &weightsMuscleBO);
		glBindBuffer(GL_ARRAY_BUFFER, weightsMuscleBO);
		glBufferData(GL_ARRAY_BUFFER, model->skinWeights.size() * sizeof(float), &(model->skinWeights[0]), GL_STATIC_DRAW);

		mergedJointIDs.insert(mergedJointIDs.end(), model->jointIDs.begin(), model->jointIDs.end());
		mergedWeights.insert(mergedWeights.end(), model->skinWeights.begin(), model->skinWeights.end());

		if (skinningGPU){
			gldAddModelSkinningData("Skinning muscles", indicesMuscleBO, weightsMuscleBO);
			//gldAddModelSkinningData("Stretching only skinning muscle", indicesMuscleBO, weightsMuscleBO);
		} else {
			gldAddModelSkinningData("Skinning muscles", 0, 0);
			//gldAddModelSkinningData("Stretching only skinning muscle", 0, 0);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);

#endif
}

void Openworm_Wrapper::prepareMergedVertices(){
#ifdef _GLD

	glGenBuffers(1, &(verticesMergedBO));
	glBindBuffer(GL_ARRAY_BUFFER, verticesMergedBO);
	glBufferData(GL_ARRAY_BUFFER, mergedVertices.size() * sizeof(float), &(mergedVertices[0]), GL_STATIC_DRAW);

	gldAddModel("Skinning merged", mergedVertices.size() / 3, verticesMergedBO);

	if (mergedTriangles.size() > 0){
		glGenBuffers(1, &trianglesMergedBO);
		glBindBuffer(GL_ARRAY_BUFFER, trianglesMergedBO);
		glBufferData(GL_ARRAY_BUFFER,mergedTriangles.size() * sizeof(int), &(mergedTriangles[0]), GL_STATIC_DRAW);

		gldAddModelEdges("Skinning merged", GL_TRIANGLES, mergedTriangles.size(), trianglesMergedBO, GL_UNSIGNED_INT);
	}

	if (mergedJointIDs.size() > 0 && mergedWeights.size() > 0){

		vector<int> floatIndices;

		for (int i=0; i<mergedJointIDs.size(); i++){
			floatIndices.push_back(mergedJointIDs[i]);		
		}

		glGenBuffers(1, &indicesMergedBO);
		glBindBuffer(GL_ARRAY_BUFFER, indicesMergedBO);
		glBufferData(GL_ARRAY_BUFFER, mergedJointIDs.size() * sizeof(int),  &(floatIndices[0]), GL_STATIC_DRAW);

		glGenBuffers(1, &weightsMergedBO);
		glBindBuffer(GL_ARRAY_BUFFER, weightsMergedBO);
		glBufferData(GL_ARRAY_BUFFER, mergedWeights.size() * sizeof(float), &(mergedWeights[0]), GL_STATIC_DRAW);

		if (skinningGPU){
			gldAddModelSkinningData("Skinning merged", indicesMergedBO, weightsMergedBO);
		} else {
			gldAddModelSkinningData("Skinning merged", 0, 0);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (skinningGPU){
		gldAddModelShaderProgram("Skinning merged", skinningProgram->program);
		//gldAddModelShaderProgram("Stretching only skinning muscle", skinningProgram->program);
		gldAddModelShaderProgram("Skinning merged", skinningProgram->program);
	} else {
		gldAddModelShaderProgram("Skinning merged", 0);
		//gldAddModelShaderProgram("Stretching only skinning muscle", 0);
		gldAddModelShaderProgram("Skinning merged", 0);
	}

#endif
}

#ifdef _GLD
void Openworm_Wrapper::prepareGLDDataForSkinningPreview(string projectPath){
	//prepareGLDDataForSimulationPreview has to be called before this

	// send skinning data to GPU

	GLW::Shaders * skinningShaders = new GLW::Shaders();
	skinningShaders->vert = new GLW::Shader(GL_VERTEX_SHADER);
	skinningShaders->vert->Load(projectPath + "Resources\\skinning.vert");
	skinningShaders->frag = new GLW::Shader(GL_FRAGMENT_SHADER);
	skinningShaders->frag->Load(projectPath + "Resources\\skinning.frag");
	skinningShaders->vert->Compile();
	skinningShaders->frag->Compile();
	skinningProgram = new GLW::Program();
	skinningProgram->AttachShaders(skinningShaders);
	skinningProgram->Link();
	skinningProgram->SaveProgramLog();

	glm::mat4 identityMatrix(1);

	glUseProgram(skinningProgram->program);

	idShaderBones = skinningProgram->getUniformLocation("bones");

	glUseProgram(0);

#ifdef _GLD

	if (skinningGPU){
		gldAddModelShaderProgram("Skinning muscles", skinningProgram->program);
		//gldAddModelShaderProgram("Stretching only skinning muscle", skinningProgram->program);
		gldAddModelShaderProgram("Skinning cuticle", skinningProgram->program);
	} else {
		gldAddModelShaderProgram("Skinning muscles", 0);
		//gldAddModelShaderProgram("Stretching only skinning muscle", 0);
		gldAddModelShaderProgram("Skinning cuticle", 0);
	}

	addSkeletonToGLD(skinningFilePath);

#endif


}
#endif

#ifdef _GLD

void Openworm_Wrapper::prepareBindPoseSkeletonAndVisualization(string filePath, Visualisation * vis, Export_Wrapper * exportWrapper){

	skinningFilePath = filePath;

	visualization = vis;
	visualization->Skeleton = new SN::SkeletonNode();

	// load skeleton for animtimestep

	if (filePath.size() > 0){
		skinningFilePath = filePath;
	}

	// load bind pose, strreighten worm
	string file = skinningFilePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES)+ "\\skeleton_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + exportWrapper->PadNumber(0, OW_PADDING) + ".skl";

	ifstream ifs(file);
	assert(ifs.good());
	boost::archive::xml_iarchive ia(ifs);

	for (int i=0; i < bindPoseSkeletonNode.nodes.size(); i++){
		delete bindPoseSkeletonNode.nodes[i];
	}

	ia >> BOOST_SERIALIZATION_NVP(bindPoseSkeletonNode);
	assignFathersForSkeletonTree(&bindPoseSkeletonNode);

	copySkeletonNode(&bindPoseSkeletonNode, visualization->Skeleton);

}

#endif

int  Openworm_Wrapper::findClosesCuticleVertexToPoint(CVector3 point){
	int minIndex = -1;
	float minDistance = FLT_MAX;
	for (int i=0; i < serMeshForSkinning->vertices.size() / 3; i++){
		float dist = Magnitude(serMeshForSkinning->GetVertex(i) - point);
		if (dist < minDistance){
			minDistance = dist;
			minIndex = i;
		}
	}
	return minIndex;
}

void  Openworm_Wrapper::calculateSkinningDataForMuscles(){

	// for each muscle, calculate center of mass C_i
	// then find two closests bones to each C_i

	/*vector<CVector3> muscleCenters;
	vector<int> muscleIndices;

	for (int m=0; m < muscleMeshesVector.size(); m++){
	CVector3 muscleCenter;
	meshes::IndexedFace * oneMuscleMesh = &(muscleMeshesVector[m]);
	for  (int i = 0; i < oneMuscleMesh->vertices.size() / 3; i++){
	muscleCenter = muscleCenter + CVector3(oneMuscleMesh->vertices[i * 3], oneMuscleMesh->vertices[i * 3 + 1], oneMuscleMesh->vertices[i * 3 + 2]);
	// 1 as an inicialization
	muscleIndices.push_back(-1);
	muscleIndices.push_back(-1);
	}
	muscleCenter = muscleCenter / (oneMuscleMesh->vertices.size() / 3);

	muscleCenters.push_back(muscleCenter);

	}

	vector<CVector3> boneCenters;
	vector<SN::SkeletonNode*> queue;
	queue.push_back(bindPoseSkeletonNode.nodes[0]);

	while (queue.size() > 0){
	SN::SkeletonNode* pNode = queue[queue.size() - 1];
	queue.pop_back();

	boneCenters.push_back((pNode->point + pNode->father->point) / 2.0);

	for (int i=0; i < pNode->nodes.size(); i++){
	SN::SkeletonNode* pSon = pNode->nodes[i];
	queue.push_back(pSon);
	}
	}

	// find closest bones to each muscle vertex
	for (int c=0; c < serMuscleMeshForSkinning->vertices.size() / 3; c++){
	for (int b=0; b < boneCenters.size(); b++){
	float dist2nd = FLT_MAX;
	float ind2nd = -1;
	float dist = Magnitude(boneCenters[b] - CVector3(serMuscleMeshForSkinning->vertices[c * 3], serMuscleMeshForSkinning->vertices[c * 3 + 1], serMuscleMeshForSkinning->vertices[c * 3 + 2]));
	// get actual distances and find if the bone is closer
	float dist1 = FLT_MAX;
	if (muscleIndices[c * 2] > -1){
	dist1 = Magnitude(boneCenters[muscleIndices[c * 2]] - serMuscleMeshForSkinning->GetVertex(c));
	}
	float dist2 = FLT_MAX;
	if (muscleIndices[c * 2 + 1] > -1){
	dist2 = Magnitude(boneCenters[muscleIndices[c * 2 + 1]] - serMuscleMeshForSkinning->GetVertex(c));
	}

	if (dist < dist1){
	// the most closest bone
	if (muscleIndices[c * 2] != -1){
	muscleIndices[c * 2 + 1] = muscleIndices[c * 2];
	}
	muscleIndices[c * 2] = b;
	} else {
	if (dist < dist2nd && dist != muscleIndices[c * 2]){
	ind2nd = b;
	dist2nd = dist;
	}
	}

	// if 2nd closest bone is closer than our msecond index, change them
	if (muscleIndices[c * 2 + 1] == -1){
	muscleIndices[c * 2 + 1] = ind2nd;
	} else {
	if (dist < dist2){
	muscleIndices[c * 2] = b;
	}
	}
	if (muscleIndices[c * 2 + 1] != -1 && ind2nd != -1){
	if (Magnitude(boneCenters[ind2nd] - CVector3(serMuscleMeshForSkinning->vertices[c * 3], serMuscleMeshForSkinning->vertices[c * 3 + 1], serMuscleMeshForSkinning->vertices[c * 3 + 2])) < Magnitude(boneCenters[muscleIndices[c * 2 + 1]] - CVector3(serMuscleMeshForSkinning->vertices[c * 3], serMuscleMeshForSkinning->vertices[c * 3 + 1], serMuscleMeshForSkinning->vertices[c * 3 + 2]))){
	muscleIndices[c * 2 + 1] = ind2nd;
	}
	}
	}
	}*/

	serMuscleMeshForSkinning->jointIDs.clear();
	serMuscleMeshForSkinning->skinWeights.clear();

	// set skinning indices and weights according to the closest cuticle vertex

	//int idx = 0;
	//for (int m=0; m < muscleMeshesVector.size(); m++){
	//CVector3 muscleCenter;
	//meshes::IndexedFace * oneMuscleMesh = &(muscleMeshesVector[m]);
	//for  (int i = 0; i < oneMuscleMesh->vertices.size() / 3; i++){
	for  (int i = 0; i < serMuscleMeshForSkinning->vertices.size() / 3; i++){
		serMuscleMeshForSkinning->influences.push_back(2);

		//int inf1 = muscleIndices[idx * 2];
		//int inf2 = muscleIndices[idx * 2 + 1];

		//float dist1 = Magnitude(CVector3(oneMuscleMesh->vertices[i * 3], oneMuscleMesh->vertices[i * 3 + 1], oneMuscleMesh->vertices[i * 3 + 2]) - boneCenters[inf1]);
		//float dist2 = Magnitude(CVector3(oneMuscleMesh->vertices[i * 3], oneMuscleMesh->vertices[i * 3 + 1], oneMuscleMesh->vertices[i * 3 + 2]) - boneCenters[inf2]);

		//float weight1 = dist1 / (dist1 + dist2);
		//float weight2 = dist2 / (dist1 + dist2);

		int idOfClosest = findClosesCuticleVertexToPoint(CVector3(serMuscleMeshForSkinning->vertices[i * 3], serMuscleMeshForSkinning->vertices[i * 3 + 1], serMuscleMeshForSkinning->vertices[i * 3 + 2]));
		float weight1 = serMeshForSkinning->skinWeights[idOfClosest * 2];
		float weight2 = serMeshForSkinning->skinWeights[idOfClosest * 2 + 1];

		int inf1 = serMeshForSkinning->jointIDs[idOfClosest * 2];
		int inf2 = serMeshForSkinning->jointIDs[idOfClosest * 2 + 1];

		serMuscleMeshForSkinning->jointIDs.push_back(inf1);
		serMuscleMeshForSkinning->jointIDs.push_back(inf2);

		serMuscleMeshForSkinning->skinWeights.push_back(weight1);
		serMuscleMeshForSkinning->skinWeights.push_back(weight2);

		//idx++;
	}
	//}

}

void Openworm_Wrapper::readWormFileAndLoadMesh(string positionsFile, string membraneFile, int ite){
	cuticleLoader.SetWormFile(positionsFile);
	cuticleLoader.SetWormMeshFile(membraneFile);

	delete mesh;
	mesh = new meshes::Mesh();

	cuticleLoader.ReadWormIteration(mesh, ite);
	cuticleLoader.ReadWormMesh(mesh);
}

void Openworm_Wrapper::prepareGLDDataForSimulationPreview(string positionsFile, string membraneFile){

	readWormFileAndLoadMesh(positionsFile, membraneFile, OW_BINDPOSE_TIMESTEP);

#ifdef _GLD

	//cuticle

	int NumOfV = mesh->vertices.size()/3;
	if (NumOfV>0)
	{
		GLfloat* GLvertices = &mesh->vertices[0];

		glGenBuffers(1, &meshToAddGLDVerticesId);
		glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDVerticesId);
		glBufferData(GL_ARRAY_BUFFER, mesh->vertices.size() * sizeof(float), GLvertices, GL_STATIC_DRAW);

		gldAddModel("Simulation cuticle", NumOfV, meshToAddGLDVerticesId);

		GLint * GLindices = &mesh->indices[0];

		glGenBuffers(1, &meshToAddGLDIndicesId);
		glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDIndicesId);
		glBufferData(GL_ARRAY_BUFFER, mesh->indices.size() * sizeof(int), GLindices, GL_STATIC_DRAW);


		gldAddModelEdges("Simulation cuticle", GL_TRIANGLES, mesh->indices.size(), meshToAddGLDIndicesId, GL_UNSIGNED_INT);


		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}


	animTimestep = 0;
	timeStepDelta = 1;
	simulationDirection = 1;

	gldAddLine("simulation iteration %i: ", &animTimestep);
#endif

	//prepareGLDDataForSkinningPreview has tobe called after this
}

void Openworm_Wrapper::readWormFileAndLoadMuscleMesh(string positionsFile, string indicesFile, int ite){
	muscleLoader.SetWormFile(positionsFile);
	muscleLoader.ReadWormIteration(muscleMesh, ite);

	// if the model is transformed x=-z, we have to use this
	swapXandYInverted(muscleMesh);

	// creation of edges in muscle cloud
	//if (false){
	muscleMeshesVector.clear();
	muscleLoader.ReadWormMuscleWithIndex(muscleMesh, muscleMeshesVector, indicesFile);

	//copy indices from all the muscles into global mesh
	/*
	vector<float> newVertices;
	vector<int> newIndices;

	int muscleOffset = 0;

	for (int m=0; m < muscleMeshesVector.size(); m++){

	meshes::IndexedFace * oneMuscleMesh = &(muscleMeshesVector[m]);

	for  (int i = 0; i < oneMuscleMesh->vertices.size() / 3; i++){

	newVertices.push_back(oneMuscleMesh->vertices[i * 3]);
	newVertices.push_back(oneMuscleMesh->vertices[i * 3 + 1]);
	newVertices.push_back(oneMuscleMesh->vertices[i * 3 + 2]);

	/*for  (int j = 0; j < oneMuscleMesh->vertices.size() / 3; j++){
	if (i != j){
	newIndices.push_back(muscleOffset + i);
	newIndices.push_back(muscleOffset + j);
	newIndices.push_back(muscleOffset + i);
	}
	}*/
	/*}

	muscleOffset = muscleOffset + oneMuscleMesh->vertices.size() / 3;
	}

	// add triangulation from per partes local muscle cells
	muscleMesh->vertices = newVertices;
	muscleMesh->indices = newIndices;
	}*/
}

void Openworm_Wrapper::prepareGLDDataForMuscleSimulationPreview(string positionsFile, string indicesFile){

#ifdef _GLD

	// muscles
	muscleIndexGroupFile = indicesFile;

	int NumOfMV = muscleMesh->vertices.size()/3;
	if (NumOfMV>0)
	{
		GLfloat* GLvertices = &muscleMesh->vertices[0];

		glGenBuffers(1, &muscleMeshToAddGLDVerticesId);
		glBindBuffer(GL_ARRAY_BUFFER, muscleMeshToAddGLDVerticesId);
		glBufferData(GL_ARRAY_BUFFER, muscleMesh->vertices.size() * sizeof(float), GLvertices, GL_STATIC_DRAW);

		gldAddModel("Simulation muscle", NumOfMV, muscleMeshToAddGLDVerticesId);

		/*GLint * GLindices = &muscleMesh->indices[0];

		glGenBuffers(1, &muscleMeshToAddGLDIndicesId);
		glBindBuffer(GL_ARRAY_BUFFER, muscleMeshToAddGLDIndicesId);
		glBufferData(GL_ARRAY_BUFFER, muscleMesh->indices.size() * sizeof(int), GLindices, GL_STATIC_DRAW);


		gldAddModelEdges("Simulation muscle", GL_TRIANGLES, muscleMesh->indices.size(), muscleMeshToAddGLDIndicesId, GL_UNSIGNED_INT);*/

		// create another buffer for inverse skinning of muscles, to check the stretching only

		gldAddModel("Stretching only skinning muscle", NumOfMV, muscleMeshToAddGLDVerticesId);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	animTimestep = 0;
	timeStepDelta = 1;
	simulationDirection = 1;

#endif

}

void Openworm_Wrapper::playSimulationAndSkinningPreview(){
	previewPaused = false;
}

void Openworm_Wrapper::pauseSimulationAndSkinningPreview(){
	previewPaused = true;
}

void Openworm_Wrapper::updateSimulationModelFromMainThread(){
#ifdef _GLD
	int NumOfV = meshToAdd.vertices.size()/3;
	if (NumOfV>0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDVerticesId);
		glBufferData(GL_ARRAY_BUFFER, NumOfV * 3 * sizeof(float), &meshToAdd.vertices[0], GL_STATIC_DRAW);
	}
#endif
}

void Openworm_Wrapper::updateSimulationMuscleModelFromMainThread(){
#ifdef _GLD
	int NumOfMuscleV = muscleMeshToAdd.vertices.size()/3;
	if (NumOfMuscleV>0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, muscleMeshToAddGLDVerticesId);// verticesMuscleBO if inverse skinning
		glBufferData(GL_ARRAY_BUFFER, NumOfMuscleV * 3 * sizeof(float), &muscleMeshToAdd.vertices[0], GL_STATIC_DRAW);
	}
#endif
}

void Openworm_Wrapper::updateSkinningMatricesFromMainThread(){

	// update skinning matrices

#ifdef _GLD
	glUseProgram(skinningProgram->program);

	if (isUpdatedskinningMatrixVec){
		for (int i=0; i < skinningMatrixVec.size(); i++){
			glUniformMatrix4fv(idShaderBones + i, 1, GL_FALSE, glm::value_ptr(skinningMatrixVec[i]));
		}
	} else {
		for (int i=0; i < skinningMatrixVecNew.size(); i++){
			glUniformMatrix4fv(idShaderBones + i, 1, GL_FALSE, glm::value_ptr(skinningMatrixVecNew[i]));
		}
	}

	glUseProgram(0);

#endif

}

/*void renderSkinnedSkeletonFromMainThread(){

}*/

void Openworm_Wrapper::addSkeletonToGLD(string filePath){
	/*GLfloat* GLvertices = &mesh->vertices[0];

	#ifdef _GLD
	glGenBuffers(1, &meshToAddGLDVerticesId);
	glBindBuffer(GL_ARRAY_BUFFER, meshToAddGLDVerticesId);
	glBufferData(GL_ARRAY_BUFFER, NumOfV * 3 * sizeof(float), GLvertices, GL_STATIC_DRAW);
	#endif

	#ifdef _GLD
	gldAddModel("ComparisionSimulationModel", NumOfV, meshToAddGLDVerticesId);
	#endif*/


#ifdef _GLD
	visualization->Skeleton2 = &bindPoseSkeletonNode;
#endif

	b_addSkinningSkeleton = true;

	b_addSkinningSkeleton = false;

#ifdef _GLD
	visualization->makeSkeleton(visualization->Skeleton);
	visualization->makeSkinnedSkeleton(visualization->Skeleton2);
	if (skinningGPU){
		gldAddModelShaderProgram("Skinning skeleton", skinningProgram->program);
	} else {
		gldAddModelShaderProgram("Skinning skeleton", 0);
	}
#endif

	int * indicesSkeleton = new int[OW_SKINNING_NUM_BONES * 2];
	float * weightsSkeleton = new float[OW_SKINNING_NUM_BONES * 2];

	for (int i=0; i < OW_SKINNING_NUM_BONES; i++){
		indicesSkeleton[i * 2] = i;
		weightsSkeleton[i * 2] = 1.0;
		indicesSkeleton[i * 2 + 1] = i;
		weightsSkeleton[i * 2 + 1] = 0.0;
	}

#ifdef _GLD

	//glUseProgram(skinningProgram->program);

	glGenBuffers(1, &indicesSkeletonBO);
	glBindBuffer(GL_ARRAY_BUFFER, indicesSkeletonBO);
	glBufferData(GL_ARRAY_BUFFER, OW_SKINNING_NUM_BONES * 2 * sizeof(int),  indicesSkeleton, GL_STATIC_DRAW);

	glGenBuffers(1, &weightsSkeletonBO);
	glBindBuffer(GL_ARRAY_BUFFER, weightsSkeletonBO);
	glBufferData(GL_ARRAY_BUFFER, OW_SKINNING_NUM_BONES * 2 * sizeof(float), weightsSkeleton, GL_STATIC_DRAW);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);

	//glUseProgram(0);

	gldAddModelSkinningData("Skinning skeleton", indicesSkeletonBO, weightsSkeletonBO);
#endif

	delete[] indicesSkeleton;
	delete[] weightsSkeleton;

}

void Openworm_Wrapper::updateSkeletonToGLDFromMainThread(){

	if (animTimestep < OW_MAX_WORM_ITERATIONS){

#ifdef _GLD

		visualization->updateSkeleton(visualization->Skeleton);

#endif

		//delete visualization->Skeleton;
		//visualization->Skeleton = new SN::SkeletonNode();

	}
}

// animation thread calls

void Openworm_Wrapper::calculateAndExportSkeletonTransformations(string filePath, Export_Wrapper * exportWrapper){

	// go through all timesteps, compute skeletons and export transformations

	int bindPoseTimeStep = 0;
	timeStep = 0;

	oLBSExtractor.sdfHalfVectors = new CVector3[0];
	oLBSExtractor.sdfHalfVectorsMG = new CVector3[0];

	pSkeletonRootBindPose = new skl::SkeletonNode();

	calculateAndExportSkeletonTransformationsForTimestep(bindPoseTimeStep, filePath, exportWrapper, pSkeletonRootBindPose);

	while (timeStep < MAX_WORM_ITERATIONS){

		calculateAndExportSkeletonTransformationsForTimestep(timeStep, filePath, exportWrapper, NULL);

		//timeStep = timeStep + 100;
		timeStep++;
	}

	delete[] oLBSExtractor.sdfHalfVectors;
	delete[] oLBSExtractor.sdfHalfVectorsMG;

}

void Openworm_Wrapper::calculateAndExportMuscleTransformations(string filePath, Export::ColladaExporter * exporter){

	int bindPoseTimeStep = 0;
	timeStep = 0;

	// load muscles in bind pose
	readWormFileAndLoadMuscleMesh(filePath + MUSCLE_POSITION_BUFFER, filePath + MUSCLE_PARTICLE_INDICES, bindPoseTimeStep);

	// calculate skeletons for each muscle in bindpose
	for (int i=0; i < muscleMeshesVector.size(); i++){
		SN::SkeletonNode * muscleSkeleton = new SN::SkeletonNode();
		extractMuscleSkeleton(&(muscleMeshesVector[i]), muscleSkeleton);
		muscleSkeletonBindPoseVec.push_back(muscleSkeleton);
	}

	while (timeStep < MAX_WORM_ITERATIONS){
		// load muscles in actual pose
		readWormFileAndLoadMuscleMesh(filePath + MUSCLE_POSITION_BUFFER, filePath + MUSCLE_PARTICLE_INDICES, timeStep);

		// 3. apply inverse skinning
		// 4. extract skeleton
		// 5. calculate the stretch factor of the skeleton

		timeStep++;
	}

}

void Openworm_Wrapper::skeletonExtractionSDF(int timeStep, string filePath, Export_Wrapper * exportWrapper, skl::SkeletonNode * skeletonOutput, meshes::Mesh  * meshImport){

	oLBSExtractor.sdfModelController = new ModelController::CModel();
	oLBSExtractor.pMesh = new MeshGraph();
	skl::SkeletonNode * loadSkelet = new skl::SkeletonNode();

	std::string pFile = std::string(filePath + "collada\\wormCollada_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".dae");

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

	oLBSExtractor.currentNumberOfSkeletonBones = generateIdForTree(extractedSkeleton, oLBSExtractor.skeletonMeshGraphIndices);

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

void Openworm_Wrapper::calculateAndExportSkeletonTransformationsForTimestep(int timeStep, string filePath, Export_Wrapper * exportWrapper, skl::SkeletonNode * skeletonOutput){

	meshes::Mesh  * meshImport = new meshes::Mesh();
	pSkeletonRootForTimestep = new skl::SkeletonNode();

	skeletonExtractionSDF(timeStep, filePath, exportWrapper, pSkeletonRootForTimestep, meshImport);

	if (skeletonOutput != NULL){
		copySNSkeletonNodeToSkl(pSkeletonRootForTimestep, skeletonOutput);
	}

	// export model

	exportWrapper->exporter.Export(meshImport, pSkeletonRootForTimestep, filePath + "animation_"+std::to_string(OW_SKINNING_NUM_BONES)+"\\model_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".dae");

	// serialize skeleton nodes

	ofstream ofs(filePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) + "\\skeleton_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".skl");
	assert(ofs.good());
	boost::archive::xml_oarchive oa(ofs);
	oa << BOOST_SERIALIZATION_NVP(pSkeletonRootForTimestep);

	// serialize per segment SDF values

	ofstream ofs2(filePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) + "\\SDF_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".sdf", std::ofstream::out);
	assert(ofs2.good());
	for (int i=0; i < perSegmentSDFVec.size(); i++){
		ofs2 << perSegmentSDFVec[i] << std::endl;
	}
	ofs2.close();

	// compare differences with bind pose - timeStep 
	SM::CalculateWormTransformationMatrices(pSkeletonRootBindPose, pSkeletonRootForTimestep);

	// send new skeleton rotations to shader
	Export::SaveQuaternionsToFile(pSkeletonRootForTimestep, filePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) +"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) + "\\quaternion_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".qua");
	Export::SaveAxisAngleToFile(pSkeletonRootForTimestep, filePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) +"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) +"\\quaternion_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".axa");
	Export::SaveMatricesToFile(pSkeletonRootForTimestep, filePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) +"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) + "\\matrix_" + std::to_string(oLBSExtractor.currentNumberOfSkeletonBones - 1) + "S_" + exportWrapper->PadNumber(timeStep, OW_PADDING) + ".mat");

	delete pSkeletonRootForTimestep;
	pSkeletonRootForTimestep = NULL;
	delete meshImport;
	meshImport = NULL;
}


vector<float> Openworm_Wrapper::calculateSkeletonSegmentLengths(SN::SkeletonNode * pNode){
	vector<float> segmentLenghts;

	vector<SN::SkeletonNode*> queue;
	queue.push_back(pNode);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		if(pNode->nodes.size() > 0){
			CVector3 seg = pNode->point - pNode->nodes[0]->point;
			segmentLenghts.push_back(Magnitude(seg));
			queue.push_back(pNode->nodes[0]);
		}
	}
	return segmentLenghts;
}

/*void Openworm_Wrapper::performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::mat4> * pSkinningMatrixVec){
vector<skl::SkeletonNode*> queue;
queue.push_back(skeletonNode);

glm::mat4 fathersAffine(1);

while (queue.size() > 0){
skl::SkeletonNode* pNode = queue[queue.size() - 1];
queue.pop_back();

int nodeId = pNode->id;

if (nodeId >= 0 && nodeId < (*pSkinningMatrixVec).size()){
glm::mat4 nodeMatrix = (*pSkinningMatrixVec)[nodeId];

glm::mat4 translateToOriginMatrix(1);
translateToOriginMatrix = glm::translate(translateToOriginMatrix, glm::vec3(-pNode->point.x, -pNode->point.y, -pNode->point.z));

glm::mat4 translateFromOriginMatrix(1);
translateFromOriginMatrix = glm::translate(translateFromOriginMatrix, glm::vec3(pNode->point.x, pNode->point.y, pNode->point.z));

// apply transformation of father to this childs matrix
nodeMatrix = translateToOriginMatrix * nodeMatrix * translateFromOriginMatrix;

nodeMatrix = fathersAffine * nodeMatrix;

// setn new matrix
(*pSkinningMatrixVec)[nodeId] = nodeMatrix;

// set father for calculation of child nodes
fathersAffine = nodeMatrix;
}

for (int i=0; i < pNode->nodes.size(); i++){
skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
queue.push_back(pSon);
}
}
}*/

void Openworm_Wrapper::performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec){
	(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);

	while (queue.size() > 0){
		nodeId++;
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];

		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(-pNode->father->point.x, -pNode->father->point.y, -pNode->father->point.z));
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z));

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix = translateFromOriginMatrix * rotate * translateToOriginMatrix; /// glm::inverse if inverse skinning

		nodeMatrix = fathersAffine * nodeMatrix;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = nodeMatrix;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}

void Openworm_Wrapper::performTimeConstantTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec){
	(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	float plus = 0.1;
	float ss = 1.0 - plus;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);

	while (queue.size() > 0){
		nodeId++;
		ss += plus;
		if (nodeId == 4) {
			plus = 0;
		}
		if (nodeId == 25) {
			plus = -0.1;
		}
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];

		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(-pNode->father->point.x, -pNode->father->point.y, -pNode->father->point.z));
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 scale = glm::scale(glm::mat4(1.0), glm::vec3(ss, 1.0, 0.9));
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z));

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix = translateFromOriginMatrix * rotate * translateToOriginMatrix; /// glm::inverse if inverse skinning
		glm::mat4 nodeMatrix2 = translateFromOriginMatrix * rotate * scale * translateToOriginMatrix;

		nodeMatrix = fathersAffine * nodeMatrix;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = fathersAffine * nodeMatrix2;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}
/*
void Openworm_Wrapper::performGravitationBasedTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec, float gravitation_magnitude){
	(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);

	float gravitation = WORM_GRAVITATION_CONSTANT * gravitation_magnitude;

	while (queue.size() > 0){
		nodeId++;
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];

		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(-pNode->father->point.x, -pNode->father->point.y, -pNode->father->point.z));
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 scale = glm::scale(glm::mat4(1.0), glm::vec3(1 + gravitation, 1.0, 1.0 - gravitation));
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), glm::vec3(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z));

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix = translateFromOriginMatrix * rotate * translateToOriginMatrix; /// glm::inverse if inverse skinning
		glm::mat4 nodeMatrix2 = translateFromOriginMatrix * rotate * scale * translateToOriginMatrix;

		nodeMatrix = fathersAffine * nodeMatrix;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = fathersAffine * nodeMatrix2;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}
*/
void Openworm_Wrapper::performGravitationBasedTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec, float gravitation_magnitude){
	(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);

	float gravitation = WORM_GRAVITATION_CONSTANT * gravitation_magnitude;
	float curvatureScale = OpenWorm::ScaleFactorFromCurvature(OpenWorm::TotalSkeletonCurvature(*pSkinningQuatVec));//0.85;

	glm::mat4 gravity = glm::scale(glm::mat4(1.0), glm::vec3(1.0 + gravitation, curvatureScale, 1.0 - gravitation));

	while (queue.size() > 0){
		nodeId++;
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];
		glm::vec3 fatherPoint = glm::vec3(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z);
		//glm::vec3 point = glm::vec3(pNode->point.x, pNode->point.y, pNode->point.z);

		fatherPoint = glm::vec3(fathersAffine * glm::vec4(fatherPoint, 1.0));

		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), -fatherPoint);
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), fatherPoint);

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix = (nodeId == 1) ? (translateFromOriginMatrix * rotate * gravity * translateToOriginMatrix) : (translateFromOriginMatrix * rotate * translateToOriginMatrix);

		nodeMatrix = nodeMatrix * fathersAffine;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = nodeMatrix;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}

void Openworm_Wrapper::performSDFBasedTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec, vector<float> &sdf, float gravitation_magnitude, const glm::vec3 &gravitationAxis) {
	(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);
	mmath::CVector3 P = queue.back()->point;
	
	float curvatureScale = OpenWorm::ScaleFactorFromCurvature(OpenWorm::TotalSkeletonCurvature(*pSkinningQuatVec));
	glm::mat4 curvatureTranslateToOriginMatrix = glm::translate(glm::mat4(1.0), glm::vec3(-P.x, -P.y, -P.z));
	glm::mat4 curvatureScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(1.0, curvatureScale, 1.0));
	glm::mat4 curvatureTranslateFromOriginMatrix = glm::translate(glm::mat4(1.0), glm::vec3(P.x, P.y, P.z));
	glm::mat4 curvatureMatrix = curvatureTranslateFromOriginMatrix * curvatureScaleMatrix * curvatureTranslateToOriginMatrix;

	while (queue.size() > 0){
		nodeId++;
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];

		float gravitation = WORM_GRAVITATION_CONSTANT * OpenWorm::ScaleSDFByGravity(sdf[nodeId], gravitation_magnitude);//sdf[nodeId-1] - 1;
		glm::vec3 point = glm::vec3(curvatureMatrix * glm::vec4(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z, 1.0));
		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), -point);
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 scale = glm::scale(glm::mat4(1.0), glm::vec3(1 + gravitation, curvatureScale, 1.0 - gravitation));
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), point);

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix = translateFromOriginMatrix * rotate * translateToOriginMatrix; /// glm::inverse if inverse skinning
		glm::mat4 nodeMatrix2 = translateFromOriginMatrix * rotate * scale * translateToOriginMatrix * curvatureMatrix;

		nodeMatrix = fathersAffine * nodeMatrix;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = fathersAffine * nodeMatrix2;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}
	/*(*pSkinningMatrixVec).resize((*pSkinningQuatVec).size());

	vector<skl::SkeletonNode*> queue;
	queue.push_back((skl::SkeletonNode*)skeletonNode->nodes[0]);

	glm::mat4 fathersAffine(1);

	int nodeId = 0;
	(*pSkinningMatrixVec)[0] = glm::mat4(1);

	float gravitation = WORM_GRAVITATION_CONSTANT * OpenWorm::ScaleSDFByGravity(sdf[1], gravitation_magnitude);
	float curvatureScale = OpenWorm::ScaleFactorFromCurvature(OpenWorm::TotalSkeletonCurvature(*pSkinningQuatVec));//0.85;

	glm::mat4 gravity = glm::scale(glm::mat4(1.0), glm::vec3(1.0 + gravitation, curvatureScale, 1.0 - gravitation));

	while (queue.size() > 0){
		nodeId++;
		skl::SkeletonNode* pNode = queue.back();
		queue.pop_back();

		glm::quat nodeQuat = (*pSkinningQuatVec)[nodeId];
		glm::vec3 fatherPoint = glm::vec3(pNode->father->point.x, pNode->father->point.y, pNode->father->point.z);
		glm::vec3 point = glm::vec3(pNode->point.x, pNode->point.y, pNode->point.z);
		
		fatherPoint = glm::vec3(fathersAffine * glm::vec4(fatherPoint, 1.0));
		point = glm::vec3(fathersAffine * glm::vec4(point, 1.0));

		glm::mat4 translateToOriginMatrix = glm::translate(glm::mat4(1), -fatherPoint);
		glm::mat4 rotate = glm::mat4_cast(nodeQuat);
		glm::mat4 translateFromOriginMatrix = glm::translate(glm::mat4(1), fatherPoint);

		// apply transformation of father to this childs matrix
		glm::mat4 nodeMatrix;
		if (nodeId == 1) {
			nodeMatrix = translateFromOriginMatrix * rotate * gravity * translateToOriginMatrix;
		} else {
			//get variables
			float gravitationFactor = WORM_GRAVITATION_CONSTANT * OpenWorm::ScaleSDFByGravity(sdf[nodeId], gravitation_magnitude);
			float s1 = (1 - gravitationFactor) / (1 - gravitation);
			float s2 = (1 + gravitationFactor) / (1 + gravitation);
			glm::vec3 dir = glm::normalize(point - fatherPoint);
			glm::vec3 bitangent = glm::normalize(glm::cross(dir, gravitationAxis));
			//undo previous gravitation change, apply current gravitation change
			glm::mat4 gravityScale = glm::scale(glm::mat4(1.0), glm::VectorForScaleMatrix(gravitationAxis * s1));
			//glm::mat4 directionScale = glm::mat4(1.0);
			glm::mat4 bitangentScale = glm::scale(glm::mat4(1.0), glm::VectorForScaleMatrix(bitangent * s2));
			//glm::mat4 scale = bitangentScale * gravityScale * directionScale;
			glm::mat4 scale = bitangentScale;

			nodeMatrix = translateFromOriginMatrix * rotate * scale * translateToOriginMatrix;
			//set new gravbitation factor
			gravitation = gravitationFactor;
		}

		nodeMatrix = nodeMatrix * fathersAffine;

		// setn new matrix
		(*pSkinningMatrixVec)[nodeId] = nodeMatrix;

		// set father for calculation of child nodes
		fathersAffine = nodeMatrix;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}*/

void Openworm_Wrapper::loadSDFFromFile(string fileName, vector<float> &sdf) {
	ifstream f(fileName);
	sdf.clear();

	float sdf_value = 0;
	while (f >> sdf_value) {
		sdf.push_back(sdf_value);
	}
}

void Openworm_Wrapper::fixFedUpSDF(vector<float> &sdf_old, vector<float> &sdf_new) {
	vector<float> sdf_temp;

	for (int i = 0; i < sdf_old.size(); i++) {
		sdf_temp.push_back(sdf_old[i]);

		if (i+1 < sdf_old.size()) {
			sdf_temp.push_back((sdf_old[i] + sdf_old[i+1]) / 2);
		}
	}
	
	sdf_new.clear();
	for (int i = 0; i < sdf_temp.size(); i++) {
		sdf_new.push_back(sdf_temp[i]);

		if ((i < 5 || i > 15) && i+1 < sdf_temp.size()) {
			sdf_new.push_back((sdf_temp[i] + sdf_temp[i+1]) / 2);
		}
	}
}

void Openworm_Wrapper::calculateHeuristicSkinningWeights(string projDir) {
	string file = projDir + "export_0\\animation_" + std::to_string(OW_SKINNING_NUM_BONES) + "\\skeleton_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_00000.skl";

	ifstream ifs(file);
	assert(ifs.good());
	boost::archive::xml_iarchive ia(ifs);

	skl::SkeletonNode *loadedSkeletonNode = new skl::SkeletonNode();
	ia >> BOOST_SERIALIZATION_NVP(loadedSkeletonNode);
	assignFathersForSkeletonTree(loadedSkeletonNode);

	serMeshForSkinning->influences.clear();
	serMeshForSkinning->skinWeights.clear();
	serMeshForSkinning->jointIDs.clear();
	//calculate offset along Y axis
	vector<float> skelet;
	SN::SkeletonNode *node = (SN::SkeletonNode*)loadedSkeletonNode;
	while (node != NULL) {
		skelet.push_back(node->point.y);

		if (node->nodes.empty())
			node = NULL;
		else
			node = node->nodes[0];
	}
	int maxIDX = skelet.size() - 1;
	//find closest node in Y for each vertex
	int numOfVerts = serMeshForSkinning->vertices.size() / 3;
	for (int i = 0; i < numOfVerts; i++) {
		float Y = serMeshForSkinning->GetVertex(i).y;
		//find segment
		int segment = 0;
		while (segment < skelet.size() && skelet[segment] < Y) {
			segment++;
		}
		segment--;
		//calculate new skinning weights
		serMeshForSkinning->influences.push_back(2);
		if (segment < 0) {
			serMeshForSkinning->jointIDs.push_back(1);
			serMeshForSkinning->jointIDs.push_back(1);

			serMeshForSkinning->skinWeights.push_back(1.0);
			serMeshForSkinning->skinWeights.push_back(0.0);
		} else if (segment == skelet.size() - 1) {
			serMeshForSkinning->jointIDs.push_back(segment);
			serMeshForSkinning->jointIDs.push_back(segment);

			serMeshForSkinning->skinWeights.push_back(1.0);
			serMeshForSkinning->skinWeights.push_back(0.0);
		} else {
			float MIX_FACTOR = 0.5;
			float factor = (Y - skelet[segment]) / (skelet[segment + 1] - skelet[segment]);
			/*if (factor > MIX_FACTOR && factor < 1 - MIX_FACTOR) {
				serMeshForSkinning->jointIDs.push_back(segment + 1);
				serMeshForSkinning->jointIDs.push_back(segment + 1);

				serMeshForSkinning->skinWeights.push_back(1.0);
				serMeshForSkinning->skinWeights.push_back(0.0);
			} else*/ if (factor <= MIX_FACTOR) {
				//normalize factor, flip and max mixig is 0.5
				float weight = (1 - (factor / MIX_FACTOR)) * 0.5;

				serMeshForSkinning->jointIDs.push_back(glm::clamp(segment, 1, maxIDX));
				serMeshForSkinning->jointIDs.push_back(glm::clamp(segment + 1, 1, maxIDX));

				serMeshForSkinning->skinWeights.push_back(weight);
				serMeshForSkinning->skinWeights.push_back(1.0 - weight);
			} else {
				//normalize factor, flip and max mixig is 0.5
				float weight = ((factor - (1 - MIX_FACTOR)) / MIX_FACTOR) * 0.5;

				serMeshForSkinning->jointIDs.push_back(glm::clamp(segment + 1, 1, maxIDX));
				serMeshForSkinning->jointIDs.push_back(glm::clamp(segment + 2, 1, maxIDX));
				
				serMeshForSkinning->skinWeights.push_back(1.0 - weight);
				serMeshForSkinning->skinWeights.push_back(weight);
			}
		}
	}
}

void Openworm_Wrapper::calculateMuscleMesh(string fileDir, meshes::IndexedFace *muscleMesh) {
	meshes::MeshSkin muscleMeshZero;

	muscleLoader.ReadWormIteration(&muscleMeshZero, 0);
	// if the model is transformed x=-z, we have to use this
	swapXandYInverted(&muscleMeshZero);

	vector<meshes::IndexedFace> muscleMeshes;
	vector<std::vector<int> > muscleIdMap;

	muscleLoader.ReadWormMuscleWithIndex(&muscleMeshZero, muscleMeshes, muscleIdMap, fileDir + MUSCLE_PARTICLE_INDICES);

	vector<meshes::IndexedFace*> muscleMeshes_ptr;
	for (int i = 0; i < muscleMeshes.size(); i++) {
		muscleMeshes_ptr.push_back(&muscleMeshes[i]);
		Import::CreateMeshForMuscle(muscleMeshes_ptr.back());
	}
	Import::TransferMeshFromMuscles(muscleMesh, muscleMeshes_ptr, muscleIdMap);
}

void Openworm_Wrapper::nextTimeStepSkinningPreview(Export_Wrapper * exportWrapper){

	return;

	//load skeleton

	string file = skinningFilePath + "animation_" + std::to_string(OW_SKINNING_NUM_BONES) + "\\skeleton_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + exportWrapper->PadNumber(animTimestep, OW_PADDING) + ".skl";

	ifstream ifs(file);
	assert(ifs.good());
	boost::archive::xml_iarchive ia(ifs);

	//glm::mat4 identityMatrix(1);

	skl::SkeletonNode *loadedSkeletonNode = new skl::SkeletonNode();

	ia >> BOOST_SERIALIZATION_NVP(loadedSkeletonNode);
	assignFathersForSkeletonTree(loadedSkeletonNode);

#ifdef _GLD
	copySkeletonNode(loadedSkeletonNode, visualization->Skeleton);
#endif

	b_addSkinningSkeleton = true;

	// load new transformations

	// update matrices

	/*string matrixFile = skinningFilePath + "animation\\matrix_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + std::to_string(animTimestep) + ".mat";
	ifstream ism(matrixFile);
	assert(ism.good());

	skinningMatrixVecNew.clear();

	ImportAffineMatrix(skinningMatrixVecNew, ism);
	//  clear translation of root;
	skinningMatrixVecNew[0][3][0] = 0;
	skinningMatrixVecNew[0][3][1] = 0;
	skinningMatrixVecNew[0][3][2] = 0;

	performTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningMatrixVecNew);

	isUpdatedskinningMatrixVec = false;

	skinningMatrixVec.clear();

	for (int i = 0; i < skinningMatrixVecNew.size(); i++) {

	skinningMatrixVec.push_back(skinningMatrixVecNew[i]);

	}

	isUpdatedskinningMatrixVec = true;*/

	// update quaternions

	string quaternionFile = skinningFilePath + "animation_"+ std::to_string(OW_SKINNING_NUM_BONES)+"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) +"\\quaternion_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + exportWrapper->PadNumber(animTimestep, OW_PADDING) + ".qua";
	ifstream isq(quaternionFile);
	assert(isq.good());

	skinningQuatVecNew.clear();

	ImportQuaternion(skinningQuatVecNew, isq);
	
	string sdf_file_31_0 = skinningFilePath + "animation_31\\SDF__31S_00000.sdf";
	string sdf_file_0 = skinningFilePath + "animation_31\\SDF_31S_" + exportWrapper->PadNumber(0, OW_PADDING) + ".sdf";
	string sdf_file_anim = skinningFilePath + "animation_31\\SDF_31S_" + exportWrapper->PadNumber(animTimestep, OW_PADDING) + ".sdf";
	vector<float> sdf_loaded;
	vector<float> sdf_reference;
	
	loadSDFFromFile(sdf_file_31_0, sdf_loaded);

	/*loadSDFFromFile(sdf_file_0, sdf_reference);
	fixFedUpSDF(sdf_reference, sdf_reference);
	loadSDFFromFile(sdf_file_anim, sdf_loaded);
	fixFedUpSDF(sdf_loaded, sdf_loaded);

	for (int i = 0; i < sdf_loaded.size(); i++) {
		sdf_loaded[i] = sdf_loaded[i] / sdf_reference[i];
	}*/

	//skinningQuatVecSimplified.clear();

	//SM::CalculateSimplifiedWormMapping(skinningQuatVecNew, simplificationMap, 10.0f);

	//performTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningQuatVecNew, &skinningMatrixVecNew);
	//performTimeConstantTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningQuatVecNew, &skinningMatrixVecNew);
	//performGravitationBasedTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningQuatVecNew, &skinningMatrixVecNew, 9.81);
	performSDFBasedTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningQuatVecNew, &skinningMatrixVecNew, sdf_loaded, 9.81, glm::vec3(0, 0, 1));

	//QuaternionsToMatrices(skinningQuatVecNew, skinningMatrixVecNew);

	isUpdatedskinningMatrixVec = false;

	skinningMatrixVec.clear();

	for (int i = 0; i < skinningMatrixVecNew.size(); i++) {

		skinningMatrixVec.push_back(skinningMatrixVecNew[i]);

	}

	//QuaternionsToMatrices(skinningQuatVec, skinningMatrixVec);

	isUpdatedskinningMatrixVec = true;

	if (skinningGPU){
		updateSkinningMatricesFromMainThread();
	}

	// load skeleton, compress and perform cpu skinning to evaluate error
	// uncomment upper //copySkeletonNode(loadedSkeletonNode, visualization->Skeleton);

	vector<bool> map;
	SM::CalculateSimplifiedWormMapping(skinningQuatVecNew, map, 5);

	performTransformationsInSkeletonTree(&bindPoseSkeletonNode, &skinningQuatVecNew, &skinningMatrixVecNew);

	skl::SkeletonNode * skinnedCPUSkeleton = new skl::SkeletonNode();

	copySkeletonNode(&bindPoseSkeletonNode, skinnedCPUSkeleton);

	//SM::TransformWormSkeleton(skinnedCPUSkeleton, skinningMatrixVecNew);

#ifdef _GLD

	//copySkeletonNode(skinnedCPUSkeleton, visualization->Skeleton);

#endif

	delete skinnedCPUSkeleton;

	delete loadedSkeletonNode;

}

void Openworm_Wrapper::CPUSkinningSkeleton(SN::SkeletonNode * skeleton) {
	vector<int> indicesSkeleton;
	vector<float> weightsSkeleton;
	vector<float> verticesSkeleton;

	vector<SN::SkeletonNode*> queue;
	queue.push_back(skeleton);

	while (queue.size() > 0){
		SN::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		verticesSkeleton.push_back(pNode->point.x);
		verticesSkeleton.push_back(pNode->point.y);
		verticesSkeleton.push_back(pNode->point.z);

		for (int i=0; i < pNode->nodes.size(); i++){
			SN::SkeletonNode* pSon = pNode->nodes[i];
			queue.push_back(pSon);
		}
	}

	for (int i=0; i < OW_SKINNING_NUM_BONES; i++){
		indicesSkeleton.push_back(i);
		weightsSkeleton.push_back(1.0);
		indicesSkeleton.push_back(i);
		weightsSkeleton.push_back(0.0);
	}

	CPUSkinning(verticesSkeleton, indicesSkeleton, weightsSkeleton);

	#ifdef _GLD
		visualization->updateSkeleton2(skinnedVertices.size(), &(skinnedVertices[0]));
	#endif

}

void Openworm_Wrapper::CPUSkinningCuticle(meshes::MeshSkin * model) {
	vector<float> bindPoseVertices = model->vertices;
	CPUSkinning(model->vertices, model->jointIDs, model->skinWeights);
	model->vertices = skinnedVertices;
	loadCuticleModelIntoBuffer(model);

	Export::ColladaExporter ex("worm_skin");
	ex.Export(model, "logs/worm_skin.dae");

	model->vertices = bindPoseVertices;
}

void Openworm_Wrapper::CPUSkinningMuscles(meshes::MeshSkin * model) {

	vector<float> bindPoseVertices = model->vertices;

	//model->vertices = muscleMeshToAdd.vertices;

	CPUSkinning(model->vertices, model->jointIDs, model->skinWeights);
	model->vertices = skinnedVertices;	

	Export::ColladaExporter ex("worm_muscle_cpu");
	ex.Export(model, "logs/worm_muscle_cpu.dae");

	loadMuscleModelIntoBuffer(model);
	model->vertices = bindPoseVertices;
}

void Openworm_Wrapper::CPUSkinning(vector<float> &inputVertices, vector<int> &inputIndices, vector<float> &inputWeights) {
	vector<glm::mat4> * activeMatrices;

	if (isUpdatedskinningMatrixVec){
		activeMatrices = &skinningMatrixVec;
	} else {
		activeMatrices = &skinningMatrixVecNew;
	}

	// perform skinning

	//transformed_vertex += weight.x * (bones[int(index.x)] * pos);
	//transformed_vertex += weight.y * (bones[int(index.y)] * pos);

	skinnedVertices.clear();
	skinnedVertices.resize(inputVertices.size());

	for (int i=0; i < inputVertices.size() / 3; i++){
		glm::vec4 transformed_vertex(0,0,0,0);

		glm::vec4 pos(inputVertices[i * 3], inputVertices[i * 3 + 1], inputVertices[i * 3 + 2], 1.0);

		int index_x = inputIndices[i * 2];
		int index_y = inputIndices[i * 2 + 1];

		float weight_x = inputWeights[i * 2];
		float weight_y = inputWeights[i * 2 + 1];

		transformed_vertex += weight_x * (*activeMatrices)[index_x] * pos;
		transformed_vertex += weight_y * (*activeMatrices)[index_y] * pos;

		skinnedVertices[i * 3] = transformed_vertex.x;
		skinnedVertices[i * 3 + 1] = transformed_vertex.y;
		skinnedVertices[i * 3 + 2] = transformed_vertex.z;
	}
}

void Openworm_Wrapper::ImportAffineMatrix(vector<glm::mat4> &matrices, std::ifstream &input) {
	for (int m = 0; m < OW_SKINNING_NUM_BONES; m++) {
		glm::mat4 mat;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				input >> mat[j][i];
			}
		}
		matrices.push_back(mat);
	}
}

void Openworm_Wrapper::ImportQuaternion(vector<glm::quat> &quaternions, std::ifstream &input) {
	for (int m = 0; m < OW_SKINNING_NUM_BONES; m++) {
		glm::quat quat;
		input >> quat.w >> quat.x >> quat.y >> quat.z;
		quaternions.push_back(quat);
	}
}

void Openworm_Wrapper::QuaternionsToMatrices(vector<glm::quat> &quaternions, vector<glm::mat4> &matrices) {
	for (int m = 0; m < OW_SKINNING_NUM_BONES; m++) {
		glm::mat4 mat = glm::mat4_cast(quaternions[m]);
		matrices.push_back(mat);
	}
}

void Openworm_Wrapper::nextTimeStepSimulationPreview(){

	cuticleLoader.ReadWormIteration(mesh, animTimestep);
	cuticleLoader.ReadWormMesh(mesh);

	meshes::IndexedFace loadedMesh0;
	cuticleLoader.ReadWormIteration(&loadedMesh0, 0);

	CVector3 shiftVec = CVector3(loadedMesh0.vertices[0], loadedMesh0.vertices[1], loadedMesh0.vertices[2]) - CVector3(mesh->vertices[0], mesh->vertices[1], mesh->vertices[2]);

	// shift points
	MEM::ShiftMeshByPoint(mesh, shiftVec);


	// if the model is transformed x=-z, we have to use this
	swapXandYInverted(mesh);

	/*if (animTimestep <= 1 || animTimestep >= 6099){
		int klkk = 0;
	}*/

	//gldReloadTransformations("Simulation muscle");

	Export::ColladaExporter ex("worm_sim");
	ex.Export(mesh, "logs/worm_sim.dae");

	addSimulationModel(mesh);

}

void Openworm_Wrapper::swapXandYInverted(meshes::IndexedFace * mesh){
	vector<float> transformedVertices;

	for (int i=0; i<mesh->vertices.size() / 3; i++){
		transformedVertices.push_back(mesh->vertices[i * 3]);		
		transformedVertices.push_back(mesh->vertices[i * 3 + 2]);		
		transformedVertices.push_back(-mesh->vertices[i * 3 + 1]);		
	}

	mesh->vertices = transformedVertices;
}

void Openworm_Wrapper::nextTimeStepMuscleSimulationPreview(){
	// muscles
	muscleLoader.ReadWormIteration(muscleMesh, animTimestep);

	// if the model is transformed x=-z, we have to use this
	swapXandYInverted(muscleMesh);

	muscleMesh->indices = serMuscleMeshForSkinning->indices;

	Export::ColladaExporter ex("worm_muscle");
	ex.Export(muscleMesh, "logs/worm_muscle.dae");

	/*vector<IndexedFace> muscleMeshes;

	muscleLoader.ReadWormMuscleWithIndex(muscleMesh, muscleMeshes, muscleIndexGroupFile);

	//copy indices from all the muscles into global mesh

	vector<float> newVertices;
	vector<int> newIndices;

	int muscleOffset = 0;

	for (int m=0; m < muscleMeshes.size(); m++){
	IndexedFace * muscleMesh = &(muscleMeshes[m]);

	for  (int i = 0; i < muscleMesh->vertices.size() / 3; i++){

	newVertices.push_back(muscleMesh->vertices[i * 3]);
	newVertices.push_back(muscleMesh->vertices[i * 3 + 1]);
	newVertices.push_back(muscleMesh->vertices[i * 3 + 2]);

	for  (int j = 0; j < muscleMesh->vertices.size() / 3; j++){
	if (i != j){
	newIndices.push_back(muscleOffset + i);
	newIndices.push_back(muscleOffset + j);
	newIndices.push_back(muscleOffset + i);
	}
	}
	}

	muscleOffset = muscleOffset + muscleMesh->vertices.size() / 3;
	}

	mesh->vertices = newVertices;
	mesh->indices = newIndices;

	addSimulationMuscleModel(mesh);*/

	addSimulationMuscleModel(muscleMesh);
}

void Openworm_Wrapper::nextTimeStepSimulationAndSkinningPreview(bool * finished, bool force, Export_Wrapper * exportWrapper){

	delete mesh;
	mesh = new meshes::IndexedFace();

	/*if (animTimestep >= OW_MAX_WORM_ITERATIONS || animTimestep <= 0 && (!previewPaused || force)){

		// change direction of animation
		if (animTimestep <= 0)
			simulationDirection = 1;
		else
			simulationDirection = -1;

		animTimestep += timeStepDelta * simulationDirection;
	}*/

	if (animTimestep >= OW_MAX_WORM_ITERATIONS && (!previewPaused || force)){
		animTimestep = 0;
		//animTimestep += timeStepDelta * simulationDirection;
	}

	if (!previewPaused || force){

		if (b_cuticleReading){
			nextTimeStepSimulationPreview();
			nextTimeStepSkinningPreview(exportWrapper);
		}

		if (b_muscleReading){
			nextTimeStepMuscleSimulationPreview();
		}

		animTimestep += timeStepDelta * simulationDirection;
	}

	*finished = true;
}

void Openworm_Wrapper::addSimulationModel(meshes::IndexedFace * mesh){
	b_addSimulationModel = true;
	meshToAdd.vertices = mesh->vertices;
	meshToAdd.indices = mesh->indices;
	meshToAdd.normals = mesh->normals;
}

void Openworm_Wrapper::addSimulationMuscleModel(meshes::IndexedFace * mesh){
	b_addSimulationMuscleModel = true;
	muscleMeshToAdd.vertices = mesh->vertices;
	muscleMeshToAdd.indices = mesh->indices;
	muscleMeshToAdd.normals = mesh->normals;
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

void Openworm_Wrapper::multiplyPerSegmentSDF(sdf::Extractor * pSDFExtractor){
	vector<float> newPerSegmentSDFVec;
	for (int j=0; j < perSegmentSDFVec.size(); j++){
		for (int i=0; i < pSDFExtractor->skeletonTesselationFactor; i++){
			newPerSegmentSDFVec.push_back(perSegmentSDFVec[j]);
		}
	}	
	perSegmentSDFVec = newPerSegmentSDFVec;
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
	float delta = 0.01;

	bool segmentLengthsAreEqual = true;// should be false when enabled this check
	float diff = 0;

	// binary search to find optimal threshold for extraction of desired number of skeleton nodes

	bool runIt = true;

	while (runIt) {
		while (((polyMeshNumOfBones != wantedNumOfSkeletonNodes) | !segmentLengthsAreEqual) & (ite < iteThreshold)){

			diff = 0;

			pLBSExtractor->polyMesh = new MeshGraph();

			float threshold = (minSDF * (1.0 - interpolationParam) + maxSDF * (interpolationParam));
			perSegmentSDFVec.clear();
			pSDFExtractor->applyMedianMerge(pLBSExtractor->polyMesh, pLBSExtractor->pMesh, threshold, &perSegmentSDFVec, pLBSExtractor->sdfHalfVectorsMG);

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
				continue;
			} 

			if (bindPoseSegmentLengths.size() > 0){
				pLBSExtractor->applyConnectivitySurgery(false, pSkeletonRoot, adaptVMdmax);
				multiplyPerSegmentSDF(pSDFExtractor);
				oSDFExtractor.subdivideSkeletonByCrossSectionMedians(pSkeletonRoot, pLBSExtractor->pMesh, pSDFExtractor->skeletonTesselationFactor);
				pLBSExtractor->currentNumberOfSkeletonBones = generateIdForTree(pSkeletonRoot, pLBSExtractor->skeletonMeshGraphIndices);

				/*vector<float> actualSegmentsLengths = calculateSkeletonSegmentLengths(pSkeletonRoot);

				if (actualSegmentsLengths.size() == bindPoseSegmentLengths.size()){
				segmentLengthsAreEqual = true;
				for (int i=0; i < bindPoseSegmentLengths.size(); i++){
				if (abs(actualSegmentsLengths[i] - bindPoseSegmentLengths[i]) > delta){
				segmentLengthsAreEqual = false;
				diff += actualSegmentsLengths[i] - bindPoseSegmentLengths[i];
				}
				}
				if (!segmentLengthsAreEqual){
				if (diff < 0){
				minIP = interpolationParam;
				interpolationParam = (interpolationParam + maxIP) / 2.0;
				} else {
				maxIP = interpolationParam;
				interpolationParam = (interpolationParam + minIP) / 2.0;
				}
				}
				}*/
			} else {
				segmentLengthsAreEqual = true;
				pLBSExtractor->applyConnectivitySurgery(false, pSkeletonRoot, adaptVMdmax);
				multiplyPerSegmentSDF(pSDFExtractor);
				oSDFExtractor.subdivideSkeletonByCrossSectionMedians(pSkeletonRoot, pLBSExtractor->pMesh, pSDFExtractor->skeletonTesselationFactor);
				pLBSExtractor->currentNumberOfSkeletonBones = generateIdForTree(pSkeletonRoot, pLBSExtractor->skeletonMeshGraphIndices);
			}

			ite++;
		}

		if (polyMeshNumOfBones == wantedNumOfSkeletonNodes && !segmentLengthsAreEqual){
			delta *= 10;
		} else if (polyMeshNumOfBones == wantedNumOfSkeletonNodes && segmentLengthsAreEqual){
			runIt = false;
		}
	}

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

void Openworm_Wrapper::calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat, boost::unordered_map<int, vector<int> > &closestVertexOneRing){
	int offset = 0;

	for(int i = 0; i < pModel->numOfObjects; i++){
		if(pModel->pObject.size() <= 0) break;
		structure::t3DObject *pObject = &(pModel->pObject[i]);

		ObjectSkeletonShaderData skeletonData;

		skeletonData.weights = new float[pObject->numOfVertices * numOfCtrlBones];
		skeletonData.indices = new float[pObject->numOfVertices * numOfCtrlBones];

		pGa->computeObjectSkeletonShaderData(pSkeletonRoot, pLBSExtractor->pMesh, pObject, &skeletonData, pObject->numOfVertices, offset, true, *pDistanceMatrix, numOfCtrlBones, maxBoneMat, closestVertexOneRing);

		// copy skeletonData

		pWormData->weights = new float[skeletonData.numOfVertices * numOfCtrlBones];
		pWormData->indices = new float[skeletonData.numOfVertices * numOfCtrlBones];
		memcpy(skeletonData.indices, pWormData->indices, skeletonData.numOfVertices * numOfCtrlBones * sizeof(float));
		memcpy(skeletonData.weights, pWormData->weights, skeletonData.numOfVertices * numOfCtrlBones * sizeof(float));	

		offset += pObject->numOfVerts;
	}
}

void Openworm_Wrapper::extractMuscleSkeleton(meshes::IndexedFace * muscle, SN::SkeletonNode * pSkelet){

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


#pragma region Additions

namespace OpenWorm {
	float TotalSkeletonCurvature(vector<glm::quat> &quats) {
		float curvature = 0;

		for (int i = 0; i < quats.size(); i++) {
			//float angle = fabsf(acos(quats[i].w) * 2.0);
			float angle = fabsf(glm::AngleFromQuaternion(quats[i]));
			curvature += angle;
		}

		return curvature;
	}

	float ScaleFactorFromCurvature(float curvature) {
		//linear interpolation for now
		return 1 - (0.025 * curvature);
	}

	float ScaleSDFByGravity(float sdf, float gravitationMagnitude) {
		//linear interpolation for now
		return (max(sdf - 4.0, 0.0) / 6.0) * gravitationMagnitude;
	}
}

#pragma endregion
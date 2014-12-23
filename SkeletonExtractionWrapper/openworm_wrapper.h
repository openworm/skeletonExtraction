#pragma once

#define OW_NUM_OF_CTRL_BONES 4
#define OW_MAX_BONE_MAT 24
#define OW_SKINNING_NUM_BONES 31

#define OW_MAX_WORM_ITERATIONS 4000
#define OW_BINDPOSE_TIMESTEP 3000
#define OW_SINPLIFY_TRIANGLECOUNT 3000
#define OW_ITE_TO_LOAD 200

#include <tchar.h>

#ifdef _LOG
	#include <logs/log.h>
#endif

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include <meshes/structure.h>
#include <meshes/SerializableMesh+Skin.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <SDF_lib/sdf_extractor.h>

#include <Import_lib/WormLoader.h>
#include <Import_lib/m_assimpLoader.h>
#include <Export_lib/ColladaExporter.h>
#include <Export_lib/MatrixExport.h>

#include <PCT_lib/pct_PointCloudTriangulation.h>

#include <inline/SkeletonNode/s_skeletonNode.h>

#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>

#ifdef _GLD
	#include <GLW_Lib/GLW_lib.h>
	#include "../WeightsVisualisation.h"
#endif

struct Openworm_Wrapper {

	bool skinningGPU;

	Assimp::Importer importer;

	vector<glm::mat4> skinningMatrixVec;
	vector<glm::mat4> skinningMatrixVecNew;

	std::vector<bool> simplificationMap;

	vector<glm::quat> skinningQuatVec;
	vector<glm::quat> skinningQuatVecNew;
	vector<glm::quat> skinningQuatVecSimplified;

	skl::SkeletonNode bindPoseSkeletonNode;

	bool isUpdatedskinningMatrixVec;

	//cuticle gpu data

	GLuint verticesBO;
	GLuint trianglesBO;
	GLuint indicesBO;
	GLuint weightsBO;

	//muscles gpu data

	GLuint verticesMuscleBO;
	GLuint trianglesMuscleBO;
	GLuint indicesMuscleBO;
	GLuint weightsMuscleBO;

	/////////////////

	GLuint verticesMergedBO;
	GLuint trianglesMergedBO;
	GLuint indicesMergedBO;
	GLuint weightsMergedBO;

	GLuint indicesSkeletonBO;
	GLuint weightsSkeletonBO;

	GLfloat *gPositions;

	meshes::IndexedFace * mesh;
	meshes::MeshSkin * muscleMesh;
	meshes::IndexedFace meshToAdd;
	meshes::IndexedFace muscleMeshToAdd;
	GLuint meshToAddGLDVerticesId;
	GLuint meshToAddGLDIndicesId;
	GLuint muscleMeshToAddGLDVerticesId;
	GLuint muscleMeshToAddGLDIndicesId;
	skl::SkeletonNode * pSkeletonRootBindPose;
	skl::SkeletonNode * pSkeletonRootForTimestep;
	lbse::Extractor oLBSExtractor;
	sdf::Extractor oSDFExtractor;

	vector<float> mergedVertices;
	vector<int>	mergedTriangles;
	vector<int> mergedJointIDs;
	vector<float> mergedWeights;


	vector<SN::SkeletonNode*> muscleSkeletonBindPoseVec;
	vector<SN::SkeletonNode*> muscleSkeletonCurrentVec;

	#ifdef _GLD
		Visualisation * visualization;
	#endif

	int simulationDirection;
	int wantedNumOfSkeletonNodes;

	Import::WormLoader cuticleLoader;
	Import::WormLoader muscleLoader;
	Export::ColladaExporter exporter;
	float adaptVMdmax;

	SN::SkeletonNode * compressedSkeletonNode;

	string skinningFilePath;
	string muscleIndexGroupFile;
	vector<IndexedFace> muscleMeshesVector;

	float maxSDF;
	float minSDF;

	int timeStep;

	bool previewPaused;
	int timeStepDelta;
	int animTimestep;
	bool manualAnimTimestep;
	bool timeStepChangedProgramaticaly;
	bool b_addSimulationModel;
	bool b_addSkinningSkeleton;
	bool b_addSimulationMuscleModel;

	bool b_muscleReading;
	bool b_cuticleReading;

	int idShaderBones; 

	#ifdef _GLD
		GLW::Program * skinningProgram;
		GLW::ArrayBuffer *worm;
	#endif

	meshes::SerializableMeshSkin * serMeshForSkinning;
	meshes::MeshSkin * serMuscleMeshForSkinning;
	vector<float> skinnedVertices;
	bool initSkinning;
	vector<float> bindPoseSegmentLengths;

	Openworm_Wrapper();
	~Openworm_Wrapper();

	void CPUSkinningCuticle(meshes::MeshSkin * model);
	void CPUSkinningMuscles(meshes::MeshSkin * model);
	void CPUSkinningSkeleton(SN::SkeletonNode * skeleton);
	void CPUSkinning(vector<float> &inputVertices, vector<int> &inputIndices, vector<float> &inputWeights);

	int  findClosesCuticleVertexToPoint(CVector3 point);

	void loadAndPrepareSkinningData(meshes::SerializableMeshSkin * serMesh, ObjectSkeletonShaderData * skeletonData);
	void calculateSkinningDataForMuscles();

	void ImportAffineMatrix(vector<glm::mat4> &matrices, std::ifstream &input);
	void ImportQuaternion(vector<glm::quat> &quaternions, std::ifstream &input);

	void QuaternionsToMatrices(vector<glm::quat> &quaternions, vector<glm::mat4> &matrices);

	void loadCuticleModelIntoBuffer(meshes::MeshSkin * model);
	void loadMuscleModelIntoBuffer(meshes::MeshSkin * model);

	void skeletonExtractionSDF(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput, meshes::Mesh  * meshImport);
	void skeletonExtractionSDF_calculateSDF(meshes::Mesh * mesh, lbse::Extractor * pLBSExtractor);
	void skeletonExtractionSDF_extractSkeleton(SN::SkeletonNode * pSkeletonRoot, lbse::Extractor * pLBSExtractor, sdf::Extractor * pSDFExtractor);
	void loadModelToStructures(lbse::Extractor * pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance);
	void calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat, boost::unordered_map<int, vector<int> > &closestVertexOneRing);

	vector<float> calculateSkeletonSegmentLengths(SN::SkeletonNode * pNode);

	// main thread calls

	#ifdef _GLD
		void prepareBindPoseSkeletonAndVisualization(string filePath, Visualisation * vis);
		void prepareGLDDataForSkinningPreview(string projectPath);
	#endif

	void readWormFileAndLoadMesh(string positionsFile, string membraneFile, int ite);
	void readWormFileAndLoadMuscleMesh(string positionsFile, string indicesFile, int ite);
	void prepareGLDDataForSimulationPreview(string positionsFile, string membraneFile);
	void prepareGLDDataForMuscleSimulationPreview(string positionsFile, string membraneFile);
	void prepareMergedVertices();
	void playSimulationAndSkinningPreview();
	void pauseSimulationAndSkinningPreview();
	void updateSimulationModelFromMainThread();
	void updateSimulationMuscleModelFromMainThread();
	void calculateAndExportSkeletonTransformations(string filePath, Export::ColladaExporter * exporter);
	void calculateAndExportSkeletonTransformationsForTimestep(int timeStep, string filePath, Export::ColladaExporter * exporter, skl::SkeletonNode * skeletonOutput);
	void calculateModelMaxDim(MeshGraph * pMesh, float * adaptVMdmax);

	void updateSkeletonToGLDFromMainThread();
	void updateSkinningMatricesFromMainThread();

	void swapXandYInverted(meshes::IndexedFace * mesh);

	//void renderSkinnedSkeletonFromMainThread();

	void addSkeletonToGLD(string filePath);

	// animation thread calls

	void nextTimeStepSkinningPreview();
	void nextTimeStepSimulationPreview();
	void nextTimeStepMuscleSimulationPreview();
	void addSimulationModel(meshes::IndexedFace * mesh);
	void addSimulationMuscleModel(meshes::IndexedFace * mesh);
	void nextTimeStepSimulationAndSkinningPreview(bool * finished, bool force);

	void performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::mat4> * pSkinningMatrixVec);
	void performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec);

	// skeleton extraction for muscles

	void extractMuscleSkeleton(meshes::IndexedFace * muscle, SN::SkeletonNode * pSkelet);

	void calculateAndExportMuscleTransformations(string filePath, Export::ColladaExporter * exporter);
};

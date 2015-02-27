#pragma once

#include <inline/openworm_constants.h>

#include <tchar.h>

#ifdef _LOG
	#include <logs/log.h>
#endif

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include <meshes/MeshErrorMeasures.h>
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

#include "../Wrappers/export_wrapper.h"

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

	vector<float> perSegmentSDFVec;

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

	void multiplyPerSegmentSDF(sdf::Extractor * pSDFExtractor);

	void loadAndPrepareSkinningData(meshes::SerializableMeshSkin * serMesh, ObjectSkeletonShaderData * skeletonData);
	void calculateSkinningDataForMuscles();

	void ImportAffineMatrix(vector<glm::mat4> &matrices, std::ifstream &input);
	void ImportQuaternion(vector<glm::quat> &quaternions, std::ifstream &input);

	void QuaternionsToMatrices(vector<glm::quat> &quaternions, vector<glm::mat4> &matrices);

	void loadCuticleModelIntoBuffer(meshes::MeshSkin * model);
	void loadMuscleModelIntoBuffer(meshes::MeshSkin * model);

	void skeletonExtractionSDF(int timeStep, string filePath, Export_Wrapper * exportWrapper, skl::SkeletonNode * skeletonOutput, meshes::Mesh  * meshImport);
	void skeletonExtractionSDF_calculateSDF(meshes::Mesh * mesh, lbse::Extractor * pLBSExtractor);
	void skeletonExtractionSDF_extractSkeleton(SN::SkeletonNode * pSkeletonRoot, lbse::Extractor * pLBSExtractor, sdf::Extractor * pSDFExtractor);
	void loadModelToStructures(lbse::Extractor * pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance);
	void calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat, boost::unordered_map<int, vector<int> > &closestVertexOneRing);

	vector<float> calculateSkeletonSegmentLengths(SN::SkeletonNode * pNode);

	// main thread calls

	#ifdef _GLD
		void prepareBindPoseSkeletonAndVisualization(string filePath, Visualisation * vis,  Export_Wrapper * exportWrapper);
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
	void calculateAndExportSkeletonTransformations(string filePath, Export_Wrapper * exportWrapperer);
	void calculateAndExportSkeletonTransformationsForTimestep(int timeStep, string filePath, Export_Wrapper * exportWrapperer, skl::SkeletonNode * skeletonOutput);
	void calculateModelMaxDim(MeshGraph * pMesh, float * adaptVMdmax);

	void updateSkeletonToGLDFromMainThread();
	void updateSkinningMatricesFromMainThread();

	void swapXandYInverted(meshes::IndexedFace * mesh);

	//void renderSkinnedSkeletonFromMainThread();

	void addSkeletonToGLD(string filePath);

	// animation thread calls

	void nextTimeStepSkinningPreview(Export_Wrapper * exportWrapper);
	void nextTimeStepSimulationPreview();
	void nextTimeStepMuscleSimulationPreview();
	void addSimulationModel(meshes::IndexedFace * mesh);
	void addSimulationMuscleModel(meshes::IndexedFace * mesh);
	void nextTimeStepSimulationAndSkinningPreview(bool * finished, bool force, Export_Wrapper * exportWrapper);

	void performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::mat4> * pSkinningMatrixVec);
	void performTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec);
	void performTimeConstantTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec);
	void performGravitationBasedTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec, float gravitation_magnitude);
	void performSDFBasedTransformationsInSkeletonTree(skl::SkeletonNode * skeletonNode, vector<glm::quat> * pSkinningQuatVec, vector<glm::mat4> * pSkinningMatrixVec, vector<float> &sdf, float gravitation_magnitude, const glm::vec3 &gravitationAxis);

	void loadSDFFromFile(string fileName, vector<float> &sdf);
	void fixFedUpSDF(vector<float> &sdf_old, vector<float> &sdf_new);
	void calculateHeuristicSkinningWeights(string projDir);
	void calculateMuscleMesh(string fileDir, meshes::IndexedFace *muscleMesh);

	// skeleton extraction for muscles

	void extractMuscleSkeleton(meshes::IndexedFace * muscle, SN::SkeletonNode * pSkelet);

	void calculateAndExportMuscleTransformations(string filePath, Export::ColladaExporter * exporter);
};

#pragma region Additions

namespace OpenWorm {
	float TotalSkeletonCurvature(vector<glm::quat> &quats);
	float ScaleFactorFromCurvature(float curvature);
	float ScaleSDFByGravity(float sdf, float gravitationMagnitude);
}

#pragma endregion
#pragma once

#include "lbse_iterationSolver.h"
#include <SkeletonNode/SkeletonNode.h>
#include <meshes/structure.h>
#include "LBSE_lib/lbse_skeleton.h"
#include <GDSW_lib/gdsw_graphAlgorithms.h>
#include <mmath/mmath.h>

#include <SDF_lib/SDF_Library.h>

#ifdef _GLD
	#include <GLD_lib/gldebugger.h>
#endif

using namespace mmath;
using namespace meshes;

#ifdef _LOG
#include "logs/log.h"
#include "logs/timerlog.h"
#endif

#include <meshes/meshSimplification.h>
#include <PCT_lib/pct_PCLManager.h>

#include <PCT_lib/pct_PointCloudTriangulation.h>

namespace lbse {
	struct Weights {
		float wL;
		float wH;
		float wC;

		Weights(){
			wL = 0.0f;
			wH = 0.0f;
			wC = 0.0f;
		}

		Weights(float _wl, float _wh, float _wc){
			wL = _wl;
			wH = _wh;
			wC = _wc;
		}
	};
	//---------------------------------------------------------------------------
	class Extractor {

		float config_wA;
		float config_wB;
		bool config_s_doBranchingSimplification;

	public:

		Extractor();
		~Extractor();

		int numOfIter;
		bool stopContractionByIteration;
		bool stopContractionByRatio;

		float adaptVMdmax;
		int mainComponentIndex;
		int maxBones; // max number, higher ids arent bone ids
		float volumeTreshold;

		bool isGeometryContracted;
		bool isSkeletonComputed;
		bool doBranchingSimplification;
		bool doDisplacementShifting;
		int numOfBones;  // number of bones really
		int wantedNumOfBones;   // number of bones chosen with slider
		MeshSimplify g_MeshSimplify;
		float groupingTolerance; // distance limit which determinates if two vertices are the same in mesh contraction
		float groupingToleranceSDFMulti;

		void computeSkeleton(structure::t3DModel *pModel, int sourcePointID, SN::SkeletonNode * skeleton, int * ite, float modelMaxDim);
		void applyConnectivitySurgery(bool applyLBSEPostprocessing, SN::SkeletonNode * node, float modelMaxDim);
		//void mergedMGToSkeleton(SN::SkeletonNode * node, float modelMaxDim);

		float calculateVolumeFromMesh(MeshGraph * mesh, structure::t3DModel *pModel, int subdivision);
		float calculateVolumeFromMesh(MeshGraph * mesh, structure::t3DModel *pModel);
		void restoreMeshVolume(MeshGraph * mesh, structure::t3DModel *pModel);
		void logContraction(MeshGraph * mesh1, MeshGraph * mesh2);
		bool checkContractedMesh(MeshGraph * pMesh);

		void computeOptimalJoiningTolerance(int &numOfComponents, vector<int> &compMapping, float * thresholds);
		int calculateNumOfBones(SN::SkeletonNode * node);
		void optimizeMeshGraphComponents();

		SN::SkeletonNode * rerootSkeleton(int newrootId, SN::SkeletonNode* pSkelet);

		template<typename T>
		void recomputeModelValuesToMeshgraph(MeshGraph * pMesh, structure::t3DModel *pModel, T * sdfValues, T * sdfvec);
		void getMGPositionVector(MeshGraph * pMesh, structure::t3DModel *pModel, float * newpos);
		void addRandomNoiseToMeshGraph(MeshGraph * pMesh, structure::t3DModel * pModel, float scale);

		void meshgraphVertexPositionsToModel(MeshGraph * pMesh, structure::t3DModel * pModel);

		OpenCLContext openCLContext;
		bool groupingWithoutEdge;
		bool cyclicSkeleton;
		int currentNumberOfSkeletonBones;
		vector<int> skeletonMeshGraphIndices;
		SurgeryGraph * pSurgeryGraph;
		MeshGraph * pMesh;
		MeshGraph * polyMesh;
		MeshGraph * pMeshOpenCL;
		MeshGraph * originalMesh;
		float origVolume;
		float contractedVolume;
		bool doSimplifyMeshAfterIteration;
		bool doAddRandomNoiseAfterIteration;
		PointCloudTriangulation::DeleunayTriangulator * pTriangulator;

		boost::unordered_map<int, vector<int> > mgDegeneratesMapping;
		bool transferInteroperabilityMesh;
		int laplacianScheme;
		BYTE * LImage;
		float wL;
		float wH;
		float sL;
		float wC;
		float wA;
		float wB;
		ModelController::CModel	* sdfModelController;
		CVector3 * sdfHalfVectors;
		CVector3 * sdfHalfVectorsMG;
		float * sdfValuesNormalizedMG;
		int sdfSize;
		bool isSDFvaluesComputed;
		bool useResolutionIndependencyTerm;
		void setsL(float _sL);
		void setwL(float _wL);
		void setwH(float _wH);
		void setwC(float _wC);
		bool doComputeSDF;
		bool useSeparateWeigthsEachIterations;
		bool useSDFBasedGroupingDistance;
		bool useSDFBasedLaplacianWeights;
		Weights * separateWeights;
		void calculateSDFForMeshGraph(MeshGraph * pMesh, structure::t3DModel * pModel, float * sdfValuesNormalizedMG, CVector3 ** sdfHalfVectors, CVector3 * sdfHalfVectorsMG);
		OpenCLManager openCLManager;
	};
}
//---------------------------------------------------------------------------
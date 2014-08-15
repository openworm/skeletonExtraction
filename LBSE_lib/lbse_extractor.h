#ifndef lbse_extractorH
#define lbse_extractorH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/

/*#pragma warning(push, 0)

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>

#pragma pop*/

//#include <GLDebugger\gldebugger.h>

//using namespace boost::archive;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

//#include "SDF_Library.h"
//#include "SDF_lib/Structures/Math/Vector4.h"
//using namespace MathStructures;

#include "lbse_iterationSolver.h"
#include <SkeletonNode/SkeletonNode.h>
#include <meshes/structure.h>
#include "LBSE_lib/lbse_skeleton.h"
#include <GDSW_lib/gdsw_graphAlgorithms.h>
#include <mmath/mmath.h>
using namespace mmath;

#ifdef _LOG
	#include "logs/log.h"
	#include "logs/timerlog.h"
#endif

#include <meshes/meshSimplification.h>
#include <PCT_lib/pct_PCLManager.h>

#include <PCT_lib/pct_PointCloudTriangulation.h>

namespace LBSE {
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
			//bool isMeshGraphComputed;
			bool isGeometryContracted;
			bool isSkeletonComputed;
			bool doBranchingSimplification;
			bool doDisplacementShifting;
			int numOfBones;  // number of bones really
			int wantedNumOfBones;   // number of bones chosen with slider
			MeshSimplify g_MeshSimplify;
			//float joiningTolerance; // distance limit which determinates if two vertices are neihgbouring
			float groupingTolerance; // distance limit which determinates if two vertices are the same in mesh contraction
			float groupingToleranceSDFMulti;

			//void computeSkeleton(t3DModel *pModel, int sourcePointID, SN::SkeletonNode * skeleton, int * ite, bool &recreateOperator, float modelMaxDim);
			void computeSkeleton(t3DModel *pModel, int sourcePointID, SN::SkeletonNode * skeleton, int * ite, float modelMaxDim);
			void applyConnectivitySurgery(SN::SkeletonNode * node, float modelMaxDim);

			float calculateVolumeFromMesh(MeshGraph * mesh, t3DModel *pModel, int subdivision);
			float calculateVolumeFromMesh(MeshGraph * mesh, t3DModel *pModel);
			void restoreMeshVolume(MeshGraph * mesh, t3DModel *pModel);
			//float calculateConvexHullVolume(CVector3 * points);
			void logContraction(MeshGraph * mesh1, MeshGraph * mesh2);
			bool checkContractedMesh(MeshGraph * pMesh);
			
			float * computeOptimalJoiningTolerance(int &numOfComponents, vector<int> &compMapping);
			int calculateNumOfBones(SN::SkeletonNode * node);
			void optimizeMeshGraphComponents();

			SN::SkeletonNode * rerootSkeleton(int newrootId, SN::SkeletonNode* pSkelet);

			template<typename T>
			void recomputeModelValuesToMeshgraph(MeshGraph * pMesh, t3DModel *pModel, T * sdfValues, T * sdfvec);
			void getMGPositionVector(MeshGraph * pMesh, t3DModel *pModel, float * newpos);
			void addRandomNoiseToMeshGraph(MeshGraph * pMesh, t3DModel * pModel, float scale);

			//void subdivideSkeleton();

			OpenCLContext openCLContext;
			bool groupingWithoutEdge;
			bool cyclicSkeleton;
			int currentNumberOfSkeletonBones;
			vector<int> skeletonMeshGraphIndices;
			SurgeryGraph * pSurgeryGraph;
			MeshGraph * pMesh;
			MeshGraph * pMeshOpenCL;
			MeshGraph * originalMesh;
			float origVolume;
			float contractedVolume;
			bool doSimplifyMeshAfterIteration;
			bool doAddRandomNoiseAfterIteration;
			PointCloudTriangulation::DeleunayTriangulator * pTriangulator;
			// mapovanie meshgraphu, ktore nastava pri vypocte laplacianu z iterativnom kontrahovani meshgraphu
			// degenerovane trojuholniky sa halfedgecollapsnu, hrani sa zmenia
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
			// UNCOMMENT ModelController::CModel	* sdfModelController;
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
			void calculateSDFForMeshGraph(MeshGraph * pMesh, t3DModel * pModel, float * sdfValuesNormalizedMG, CVector3 ** sdfHalfVectors, CVector3 * sdfHalfVectorsMG);
			OpenCLManager openCLManager;
	};
}
//---------------------------------------------------------------------------
#endif

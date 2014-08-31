#ifndef lbse_skeletonH
#define lbse_skeletonH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/

#include <windows.h>

using namespace std;
//using namespace System::Runtime::InteropServices;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

#include "meshes/structure.h"
#include "meshes/MeshGraph.h"
#include "SkeletonNode/SkeletonNode.h"
//#include "BMM_lib/SkeletonNode.h"

//#include "SDF_Library.h"

//---------------------------------------------------------------------------

const float wA = 1.0f;
const float wB = 0.1f;
const bool s_doBranchingSimplification = true;


//---------------------------------------------------------------------------
void destroySkeletonNodeSubtree(SN::SkeletonNode * node);
void collapseTriangle(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int i, int j, int k);
bool manifoldCheck(MeshGraph * pMesh, int e1, int e2, int i, int j);

int generateIdForTree(SN::SkeletonNode * pNode, vector<int> &meshGraphIndices);
int generateIdForTree(SN::SkeletonNode * pNode);
//void generateIdDSF(SN::SkeletonNode * pNode, int * p);
SN::SkeletonNode * findNodeWithIdInTree(SN::SkeletonNode * node, int id);
SN::SkeletonNode * findNodeWithId(SN::SkeletonNode * pRoot, int id);
//float GeodesicDistance(MeshGraph * pMesh,  Array2D<float> distanceMatrix, CVector3 p1, int vertexIndex, bool firstSkeletonPoint);
//void findXMinInTree(MeshGraph * pMesh, Array2D<float> distanceMatrix, int x, SN::SkeletonNode * root, SN::SkeletonNode * mins, int vertexIndex);
void calculateOneRingExtent(MeshGraph * pMesh, float * p, vector<std::set<int>> globalNeighbourhoods);
void copyMeshGraph(MeshGraph * in, MeshGraph * out);
void assignFathersForSkeletonTree(SN::SkeletonNode * pNode);
void copySkeletonNode(SN::SkeletonNode * src, SN::SkeletonNode * dest);
void copySkeletonNodeImpl(SN::SkeletonNode * src, SN::SkeletonNode * dest);
//void copySkeletonNodeToSQMNode(SN::SkeletonNode * src, bmm::SkeletonNode * dest, float radius);
void createSkeletonFromMeshGraph(MeshGraph * pMesh, SN::SkeletonNode node, int numOfNodes);
void createSurgeryGraphFromMeshGraph(SurgeryGraph * pGraph, MeshGraph * pMesh);
SurgeryGraph * createSkeletonFromSurgeryGraph(SurgeryGraph * pGraph, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping ,SN::SkeletonNode * root, int * wantedNumberOfBones, float groupingTolerance, float groupingToleranceSDFMulti, bool doBranchingSimplification,bool doDisplacementShifting, bool groupingWithoutEdge, bool cyclicSkeleton, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
SurgeryGraph * createSurgeryGraphFromMeshGraph(MeshGraph * pMesh);
float computeShapeCost(SurgeryGraph * pGraph, int i, int j);
void collapseCloseVertices(SurgeryGraph * pGraph, float groupingTolerance, float groupingToleranceSDFMulti, int * actualNumOfBones, int * wantedNumberOfBones, bool groupingWithoutEdge, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
float computeErrorValue(SurgeryGraph * pGraph, int i, int j);
Array2D< float > constructKMatrix(SurgeryGraph * pGraph, int i, int j);
float computeSamplingCost(SurgeryGraph * pGraph, int i, int j);
float computePointCloudVolume(vector<int> indices, structure::t3DModel *pModel, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
mmath::CVector3 computeWADisplacement(int j, SurgeryGraph * pGraph, mmath::CVector3 * modelVertices, Array2D<bool> originalE, SN::SkeletonNode * node, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
mmath::CVector3 computeWADisplacement0(int j, SurgeryGraph * pGraph, mmath::CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
mmath::CVector3 computeWADisplacement1(int j, SurgeryGraph * pGraph, mmath::CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
mmath::CVector3 computeWADisplacement2(int j, SurgeryGraph * pGraph, mmath::CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
mmath::CVector3 computeWADisplacement3(int j, SurgeryGraph * pGraph, mmath::CVector3 * modelVertices, Array2D<bool> originalE, boost::unordered_map<int, vector<int> > mgDegeneratesMapping);
float computeCenterdness(int adj, int bran, SurgeryGraph * pGraph);
void mergeAdjNodeWithBran(SN::SkeletonNode * pNode, int adjIndex);
//mmath::CVector3 * createModelVerticesArray(MeshGraph * mesh);
std::vector<int> getDegenerateIndex(boost::unordered_map<int, vector<int> > mgDegeneratesMapping, int idx);
//---------------------------------------------------------------------------
#endif

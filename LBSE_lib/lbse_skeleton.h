#pragma once

#include <windows.h>

using namespace std;

#include "meshes/structure.h"
#include "meshes/MeshGraph.h"
#include "SkeletonNode/SkeletonNode.h"

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
SN::SkeletonNode * findNodeWithIdInTree(SN::SkeletonNode * node, int id);
SN::SkeletonNode * findNodeWithId(SN::SkeletonNode * pRoot, int id);
void calculateOneRingExtent(MeshGraph * pMesh, float * p, vector<std::set<int>> globalNeighbourhoods);
void copyMeshGraph(MeshGraph * in, MeshGraph * out);
void assignFathersForSkeletonTree(SN::SkeletonNode * pNode);
void copySkeletonNode(SN::SkeletonNode * src, SN::SkeletonNode * dest);
void copySkeletonNodeImpl(SN::SkeletonNode * src, SN::SkeletonNode * dest);
void createSkeletonFromMeshGraph(MeshGraph * pMesh, SN::SkeletonNode node, int numOfNodes);
void createSurgeryGraphFromMeshGraph(MeshGraph * pMesh, SurgeryGraph * pGraph);
void createSkeletonFromSurgeryGraph(bool applyLBSEPostprocessing, SurgeryGraph * pGraph, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping ,SN::SkeletonNode * root, int * wantedNumberOfBones, float groupingTolerance, float groupingToleranceSDFMulti, bool doBranchingSimplification,bool doDisplacementShifting, bool groupingWithoutEdge, bool cyclicSkeleton, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
void createSkeletonFromSurgeryGraphSDFTest(SurgeryGraph * pGraph, CVector3 * modelVertices, boost::unordered_map<int, vector<int> > mgDegeneratesMapping ,SN::SkeletonNode * root, int * wantedNumberOfBones, float groupingTolerance, float groupingToleranceSDFMulti, bool doBranchingSimplification,bool doDisplacementShifting, bool groupingWithoutEdge, bool cyclicSkeleton, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
float computeShapeCost(SurgeryGraph * pGraph, int i, int j);
void collapseCloseVertices(SurgeryGraph * pGraph, float groupingTolerance, float groupingToleranceSDFMulti, int * actualNumOfBones, int * wantedNumberOfBones, bool groupingWithoutEdge, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
void collapseCloseVerticesSDFTest(SurgeryGraph * pGraph, float groupingTolerance, float groupingToleranceSDFMulti, int * actualNumOfBones, int * wantedNumberOfBones, bool groupingWithoutEdge, CVector3 * sdfValues, bool useSDFBasedGroupingDistance);
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
std::vector<int> getDegenerateIndex(boost::unordered_map<int, vector<int> > mgDegeneratesMapping, int idx);
//---------------------------------------------------------------------------
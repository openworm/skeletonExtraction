#ifndef s_dijkstraH
#define s_dijkstraH
//---------------------------------------------------------------------------
#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif

#include <meshes/structure.h>
#include <meshes/MeshGraph.h>
#ifdef _LOG
	#include <inline/logs/log.h>
#endif
//#include "v_utils.h"
//#include "s_skeleton.h"
#include <SkeletonNode/SkeletonNode.h>
#include <LBSE_lib/lbse_skeleton.h>

#include<queue>
#include<iostream>
#include<math.h>
#include <float.h>
//#include "m_math.h"
#include <mmath/mmath.h>
using namespace mmath;
using namespace std;

#define MAX 100

//---------------------------------------------------------------------------
struct ObjectSkeletonShaderData {
	int numOfVertices;
	float * weights;
	float * indices;
	int numOfBones;
};

class GraphAlgorithms {
private:
	//---------------------------------------------------------------------------
	int *father;
	bool *visit;

	//---------------------------------------------------------------------------
public:
	void computeDijkstraDistance(MeshGraph * pMesh, Array2D<float>& M);
	void FloydWarshall(MeshGraph * pMesh, Array2D<float>& M);
	void FloydWarshallLocaly(SN::SkeletonNode * pSklRoot, boost::unordered_map<int, std::vector<int> > &closestToSkeletonNode, boost::unordered_map<int, std::vector<int> > &closestSegmentsVertexIndices, MeshGraph * pMesh, Array2D<float>& M);

	void computeClosestSegmentVertices(MeshGraph * pMesh, SN::SkeletonNode * pSkelRoot, boost::unordered_map<int, std::vector<int> > &closestToSkeletonNode,  boost::unordered_map<int, vector<int> > &closestSegmentsVertexIndices);

	void calculateClosestOneRingIndices(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &closestVertexOneRing);

	float GeodesicDistance(MeshGraph * pMesh,  Array2D<float> distanceMatrix, CVector3 p1, int vertexIndex, bool firstSkeletonPoint, vector<int> closestVertexOneRing);
	void findXMinInTree(MeshGraph * pMesh, Array2D<float> distanceMatrix, int x, SN::SkeletonNode *node, SN::SkeletonNode * mins, int vertexIndex, boost::unordered_map<int, vector<int> > &closestVertexOneRing);
	int  findClosestPointIndex(MeshGraph *pMesh, CVector3 sourcePoint);
	void computeObjectSkeletonShaderData(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, structure::t3DObject *pObject, ObjectSkeletonShaderData * data, int length, int indexOffset, bool isMeshGraphComputed, Array2D<float> distanceMatrix, int numOfControlBones, int maxBones, boost::unordered_map<int, vector<int> > &closestVertexOneRing);
	void computeMeshGraphSkinningData(SN::SkeletonNode* currentSkeletonRoot, MeshGraph * pMesh, ObjectSkeletonShaderData * data, bool isMeshGraphComputed, Array2D<float> distanceMatrix, int numOfControlBones, int maxBones, boost::unordered_map<int, vector<int> > &closestVertexOneRing);
};
//---------------------------------------------------------------------------
#endif

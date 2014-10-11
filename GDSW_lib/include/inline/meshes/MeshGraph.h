#ifndef meshGraphH
#define meshGraphH
//---------------------------------------------------------------------------

#include <boost/unordered_map.hpp>
#include <vector>

#include <meshes/structure.h>
#include <mmath/mmath.h>
#include <meshes\IndexedFace.h>
using namespace mmath;

#ifdef _LOG
	#include<logs/log.h>  
#endif // _LOG


#include <PCT_lib/pct_PointCloudTriangulation.h>


//using namespace std;

struct VertexJoiningVisualization {
	int numOfPairs;
	CVector3 * from;
	CVector3 * to;
	bool computed;
	VertexJoiningVisualization(){
		from = new CVector3[0];
		to = new CVector3[0];
	}
	~VertexJoiningVisualization(){
		delete[] from;
		from = NULL;
		delete[] to;
		to = NULL;
	}
};

struct NeighVisualization
{
	Array2D<bool> E_local_visualize;
	bool isE_local_visualize;
	CVector3 * visNormals;
	std::set<int> localNeighs;
	//CVector2 * pointsInTangentPlane;
};

//---------------------------------------------------------------------------
struct MeshGraph {
	bool isPointCloud;
	int numOfVertices;
	CVector3 * pVerts;			// The object's vertices
	boost::unordered_map<int, int> indices; // mGindex = meshGraph->indices[index + offset]; 
	//boost::unordered_map<int, vector<int>> inverseIndices; // inverseIndices[mgIndex] = vector of original triangle indices + offset
	Array2D<bool> E;
	float * wH;
	float * wHorig;
	float wL;
	float wC;
	float faceAreaSum;
	int numOfFaces;
	float * origOneRingArea;
	float * origOneRingExtent;
	// vbo variables
	bool createdVBO;
	int numOfVBOlines;
	//CVector3 * pColor;
	unsigned int gMeshgraphPositionsVB;
	unsigned int gMeshgraphLinesVB;
	unsigned int gMeshgraphColorsVB;
	int * triangleIndices; // array of pointers of mgindices -> 3indices 3ndices 3indices... size is 3xnumof Faces
	//int neighPCIndex;
	//neighVisualization * neighVis;

	MeshGraph(){
		isPointCloud = false;
		pVerts = new CVector3[0];
		wH = new float[0];
		wHorig = new float[0];
		origOneRingArea = new float[0];
		origOneRingExtent = new float[0];
		createdVBO = false;
		gMeshgraphPositionsVB = -1;
		gMeshgraphLinesVB = -1;
		gMeshgraphColorsVB = -1;
		triangleIndices = new int[0];
	}
	~MeshGraph(){
		delete[] pVerts;
		pVerts = NULL;
		delete[] wH;
		wH = NULL;
		delete[] wHorig;
		wHorig = NULL;
		delete[] origOneRingArea;
		origOneRingArea = NULL;
		delete[] origOneRingExtent;
		origOneRingExtent = NULL;
		delete[] triangleIndices;
		triangleIndices = NULL;
	}

	//boost::unordered_map<int, std::vector<int> > returnIdentityDegeneratesMapping();

};


struct SurgeryGraph {
	int numOfVertices;
	CVector3  * pVerts;			// The object's vertices
	boost::unordered_map<int, int> collapsed;
	boost::unordered_map<int, int> segmentIndex;
	boost::unordered_map<int, std::vector<int> > pointClouds;
	//vector<int> collapsed;
	//vector<int> segmentIndex;
	//vector< vector<int> > pointClouds;

	Array2D<bool> E;
	std::vector<Array2D< float > >QMatrices;
	SurgeryGraph(){
		pVerts = new CVector3[0];
	}
	~SurgeryGraph(){
		delete[] pVerts;
		pVerts = NULL;
	}
};

void calculateOneRingArea(MeshGraph * pMesh, float * p);
void createMeshGraph(structure::t3DModel *pModel,MeshGraph * pMesh, VertexJoiningVisualization * pJoining, float * joinings, float delta, float wL, float wH, int numOfComponents, std::vector<int> compMapping);
void createMeshGraph(meshes::IndexedFace *mesh, MeshGraph * pMesh, float wL, float wH);
void halfEdgeCollapse(SurgeryGraph * pGraph, int i, int j, int * actualNumOfBones, bool updateQ);
//void halfEdgeCollapseSDFTest(SurgeryGraph * pGraph, int i, int j, int * actualNumOfBones, bool updateQ);
void halfEdgeCollapse(MeshGraph * pMesh, boost::unordered_map<int, std::vector<int> > &mgDegeneratesMapping, int i, int j);
bool isMeshTriangle(MeshGraph * pMesh, int a,int b,int c);

PCTMeshGraph * MG2PCTMG(MeshGraph * pMesh);

#include "MeshGraph.inl"

#endif

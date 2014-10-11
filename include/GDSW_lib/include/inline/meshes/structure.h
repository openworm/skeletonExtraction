#ifndef c_structureH
#define c_structureH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/

#include <windows.h>
#include <vector>
#include <SDF_lib/Structures/Math/Mat4.h>

//using namespace std;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

//#include "m_math.h"
#include "mmath/mmath.h"
using namespace mmath;

#ifdef _LOG
	#include "logs/log.h"
#endif
//---------------------------------------------------------------------------

namespace structure {
struct BoundingBox {
	float x_min;
	float x_max;
	float y_min;
	float y_max;
	float z_min;
	float z_max;

	CVector3 x_min_v;
	CVector3 x_max_v;
	CVector3 y_min_v;
	CVector3 y_max_v;
	CVector3 z_min_v;
	CVector3 z_max_v;

	BoundingBox(){
		x_min = FLT_MAX;
		x_max = -FLT_MAX;
		y_min = FLT_MAX;
		y_max = -FLT_MAX;
		z_min = FLT_MAX;
		z_max = -FLT_MAX;
	}
};


// This is our 2D point class.  This will be used to store the UV coordinates.
struct tVector2 {
	float x, y;
};

// This is our face structure.  This is is used for indexing into the vertex
// and texture coordinate arrays.  From this information we know which vertices
// from our vertex array go to which face, along with the correct texture coordinates.
struct tFace
{
	int vertIndex[3];			// indicies for the verts that make up this triangle
	int coordIndex[3];			// indicies for the tex coords to texture this face
	//int  materialID;
};

struct tIntPair
{
	int pair[2];	// 2 vertices of an edge
	tIntPair(int p1, int p2){
		pair[0] = p1;
		pair[1] = p2;
	}
};


// This holds the information for a material.  It may be a texture map of a color.
// Some of these are not used, but I left them because you will want to eventually
// read in the UV tile ratio and the UV tile offset for some models.
struct tMaterialInfo
{
	char  strName[255];			// The texture name
	char  strFile[255];			// The texture file name (If this is set it's a texture map)
	BYTE  color[3];				// The color of the object (R, G, B)
	int   textureId;				// the texture ID
	//int * pTextredFaces;
	//int numOfTextredFaces;
	//float uTile;				// u tiling of texture  (Currently not used)
	//float vTile;				// v tiling of texture	(Currently not used)
	//float uOffset;			    // u offset of texture	(Currently not used)
	//float vOffset;				// v offset of texture	(Currently not used)
} ;

// This holds all the information for our model/scene.
// You should eventually turn into a robust class that
// has loading/drawing/querying functions like:
// LoadModel(...); DrawObject(...); DrawModel(...); DestroyModel(...);
struct t3DObject
{
	int  numOfVerts;			// The number of verts in the model
	int  numOfVertices;         // (Vertices > Verts)Number of points displayed, multivertices counted as well
	int  numOfFaces;			// The number of faces in the model
	int  numTexVertex;			// The number of texture coordinates
	int  materialID;			// The texture ID to use, which is the index into our texture array
	bool bHasTexture;			// This is TRUE if there is a texture map for this object
	char strName[255];			// The name of the object
	bool hasMeshController;
	unsigned int *pIndices;		// The straight face indices in a row, used for vertex arrays
	//int * pMeshGraphIndices;
	CVector3  *pVerts;			// The object's vertices
	CVector3  *pNormals;		// The object's normals
	CVector3  *pTangents;		// The object's normals
	CVector2  *pTexVerts;		// The texture's UV coordinates
	tFace *pFaces;				// The faces information of the object
	CVector3  *pFaceNormals;		// The object's normals
	unsigned int *pVertID;              // Vertex IDs in current object
	BoundingBox bb;             // Bouding box of object
	bool skinning;
};

// This holds our model information.  This should also turn into a robust class.
// We use STL's (Standard Template Library) vector class to ease our link list burdens. :)
struct t3DModel
{
	int numOfObjects;					// The number of objects in the model
	int numOfMaterials;					// The number of materials for the model
	std::vector<tMaterialInfo> pMaterials;	// The list of material information (Textures and colors)
	std::vector<t3DObject> pObject;			// The object list for our model
	int numOfVertices;
	int numOfVerts;
	unsigned long tessTrianCount;
	//vector<vector<int> > neighbours;    // Neighbours for each vertex
	BoundingBox modelbb;
	std::string filePath;
	int * cubeMaps;
	MathStructures::Mat4 rootTrans;
};

// Here is our structure for our 3DS indicies (since .3DS stores 4 unsigned shorts)
struct tIndices {

	unsigned short a, b, c, bVisible;		// This will hold point1, 2, and 3 index's into the vertex array plus a visible flag
};

// This holds the chunk info
struct tChunk
{
	unsigned short int ID;					// The chunk's ID
	unsigned int length;					// The length of the chunk
	unsigned int bytesRead;					// The amount of bytes read within that chunk
};

}

//---------------------------------------------------------------------------
void releaseStructure(structure::t3DModel *pModel);
void createModelNeighbours(structure::t3DModel *pModel, float delta);
int  findClosestPointID(structure::t3DModel *pModel, CVector3 sourcePoint);
CVector3 getVertexWithID(structure::t3DModel *pModel, int id);
void dumpNeighbours(structure::t3DModel *pModel);
void dumpDistances(structure::t3DModel *pModel);
void calculateVerticeCount(structure::t3DObject *pObject);
void calculateVerticeCount(structure::t3DModel *pModel);
int getTopPointID(structure::t3DModel *pModel);
//---------------------------------------------------------------------------



#include "structure.inl"

#endif

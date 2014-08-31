#ifndef m_3dsLoaderH
#define m_3dsLoaderH
//---------------------------------------------------------------------------
#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif

#include <float.h>



#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif

#ifdef _LOG
	#include <logs/log.h>
#endif

//#include "m_math.h"
#include <mmath/mmath.h>
using namespace mmath;
#include <meshes/structure.h>
//---------------------------------------------------------------------------
//>------ Primary Chunk, at the beginning of each file
#define PRIMARY_3DS       0x4D4D

//>------ Main Chunks
#define OBJECTINFO_3DS    0x3D3D				// This gives the version of the mesh and is found right before the material and object information
#define VERSION_3DS       0x0002				// This gives the version of the .3ds file
#define EDITKEYFRAME_3DS  0xB000				// This is the header for all of the key frame info

//>------ sub defines of OBJECTINFO
#define MATERIAL_3DS	  0xAFFF				// This stored the texture info
#define OBJECT_3DS		  0x4000				// This stores the faces, vertices, etc...

//>------ sub defines of MATERIAL
#define MATNAME_3DS       0xA000				// This holds the material name
#define MATDIFFUSE_3DS    0xA020				// This holds the color of the object/material
#define MATMAP_3DS        0xA200				// This is a header for a new material
#define MATMAPFILE_3DS    0xA300				// This holds the file name of the texture

#define OBJECT_MESH_3DS   0x4100				// This lets us know that we are reading a new object

//>------ sub defines of OBJECT_MESH
#define OBJECT_VERTICES_3DS     0x4110			// The objects vertices
#define OBJECT_FACES_3DS		0x4120			// The objects faces
#define OBJECT_MATERIAL_3DS		0x4130			// This is found if the object has a material, either texture map or color
#define OBJECT_UV_3DS			0x4140			// The UV texture coordinates

#define MAX_TEXTURES 100								// The maximum amount of textures to load
// This class handles all of the loading code

//---------------------------------------------------------------------------
class CLoad3DS{
public:
	CLoad3DS();		// This inits the data members

	// This is the function that you call to load the 3DS
	bool Import3DS(structure::t3DModel *pModel, char *strFileName);

    // The file pointer
	FILE *m_FilePointer;

	structure::BoundingBox g_modelMax;

	// This computes the vertex normals for the object (used for lighting)
	void ComputeNormals(structure::t3DModel *pModel);

private:
	// This reads in a string and saves it in the char array passed in
	int GetString(char * pBuffer);

	// This reads the next chunk
	void ReadChunk(structure::tChunk *);

	// This reads the next large chunk
	void ProcessNextChunk(structure::t3DModel *pModel, structure::tChunk *, structure::BoundingBox *pObjectMax);

	// This reads the object chunks
	void ProcessNextObjectChunk(structure::t3DModel *pModel, structure::t3DObject *pObject, structure::tChunk *, structure::BoundingBox *pObjectMax);

	// This reads the material chunks
	void ProcessNextMaterialChunk(structure::t3DModel *pModel, structure::tChunk *);

	// This reads the RGB value for the object's color
	void ReadColorChunk(structure::tMaterialInfo *pMaterial, structure::tChunk *pChunk);

	// This reads the objects vertices
	void ReadVertices(structure::t3DObject *pObject, structure::tChunk *pChunk, structure::BoundingBox *pObjectMax);

	// This reads the objects face information
	void ReadVertexIndices(structure::t3DObject *pObject, structure::tChunk *);

	// This reads the texture coodinates of the object
	void ReadUVCoordinates(structure::t3DObject *pObject, structure::tChunk *);

	// This reads in the material name assigned to the object and sets the materialID
	void ReadObjectMaterial(structure::t3DModel *pModel, structure::t3DObject *pObject, structure::tChunk *pPreviousChunk);

	void GenerateTextureCoords(structure::t3DModel *pModel);

	// This frees memory and closes the file
	void CleanUp();

	// These are used through the loading process to hold the chunk information
	structure::tChunk *m_CurrentChunk;
	structure::tChunk *m_TempChunk;

	int buffer[50000];

    int actVertID;
};
void generateTangent(CVector3 &s_tan, CVector3 &t_tan, CVector3 v1, CVector3 v2, CVector2 st1, CVector2 st2);

//---------------------------------------------------------------------------
#endif

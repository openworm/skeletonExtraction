#ifndef m_colladaLoaderH
#define m_colladaLoaderH
//---------------------------------------------------------------------------
#undef GL_BLEND_SRC_ALPHA
#undef GL_BLEND_DST_ALPHA
#undef HEX
//---------------------------------------------------------------------------
#ifdef _MMGR
#include "c_newdeleteOriginal.h"
#endif

//#include <dae.h>
//#include <dom.h>

//#include <windows.h>
//#include <string>
//#include <boost/unordered_map.hpp>

//#include <dom/domInput_local_offset.h>
//#include <dom/domMaterial.h>
//#include <dom/domCOLLADA.h>
//#include <dom/domConstants.h>
//#include <dom/domProfile_COMMON.h>
//#include <dae/daeUtils.h>


//#include <TextureLoader.h>

//using namespace cdom;
//using namespace std;

#ifdef _MMGR
#include "c_newdeleteDebug.h"
#endif
/*
//#include "s_skeletonNode.h
#include "c_structure.h"
#include "s_skeleton.h"
//---------------------------------------------------------------------------
#define SafeAdd(elt, name, var) \
	daeElement* var = elt->add(name);
//---------------------------------------------------------------------------
struct sRGBColor
{
	float fRed;
	float fGreen;
	float fBlue;
	float fAlpha;

	sRGBColor(float fR = 1.0f, float fG = 1.0f, float fB = 1.0f, float fA = 1.0f) : fRed(fR), fGreen(fG), fBlue(fB), fAlpha(fA) {}
	BYTE * toByte3();
};

class Poly
{
public:
	unsigned int iIndexCount;
	unsigned int *pindices;
	Poly(){
		iIndexCount = 3; // default is triangle
		pindices = new unsigned int[iIndexCount];
	}
	Poly(int * indices){
		iIndexCount = 3; // default is triangle
		pindices = new unsigned int[iIndexCount];
		for (int i=0; i < iIndexCount; i++)
			pindices[i] = indices[i];
	}
	~Poly(){
		//delete[] pindices;
	}
};

// Our material structure, which we create by converting a domMaterial object
class Material {
public:
	string strMaterialName;
	string refToTexture;
	bool initialized;
	sRGBColor rgbAmbient;
	sRGBColor rgbEmissive;
	sRGBColor rgbDiffuse;
	sRGBColor rgbSpecular;
	float shininnes;


	unsigned int textureID;
	Material(domMaterial& mtl);
	Material(string str);
};

class CBone {
public:
	string        id;                  // ID of this bone
	TNT::Array2D<float>   m_InverseBindMatrix;  // The Inverse Bind Pose Matrix
};


// Our mesh structure, which we create by converting a domGeometry object
class CMesh {
public:
	Material* mtl;
	unsigned int iVerticesCount;
	unsigned int iVertsCount;
	CVector3 *pVertices;
	vector<unsigned int> vIndices;
	vector<CVector3> vNormals;
	vector<CVector2> vTexVerts;
	vector<Poly> vPolygons;
	CMesh(domGeometry& geom);
	void ConstructTriangles(domMesh *thisMesh, domTriangles *thisTriangles);
	void ConstructPolylist(domMesh *thisMesh, domPolylist *thisPolylist);
	void ConstructPolygon(domMesh *thisMesh, domPolygons *thisPolygons);
	void ConstructTrifans(domMesh *thisMesh, domTrifans *thisTrifans);
	void ConstructTristrips(domMesh *thisMesh, domTristrips *thisTristrip);
};

class Controller {
public:
	domFloat4x4 bindShape;
	daeElement * geoElement;
	vector<Material> materials;
	vector<string> symbols;
	vector<CBone> bones;

	boost::unordered_map<string, int> boneIndices;

	float * weights;
	float * indices;
	Controller(domController& controller);
};


// Our node structure, which we create by converting a domNode object
class Node {
public:
	vector<domFloat4x4> vMatrix;
	vector<domFloat4> vRotate;
	vector<domFloat3> vTranslate;
	vector<domFloat3> vScale;

	vector<CMesh*> meshes;
	vector<Controller*> controllers;
	//CBone *pBoneInfo;
	bool isBone;
	bool skinning;
	string sId;
	vector<Node*> childNodes;

	// This is defined later to work around a circular dependency on the lookup function
	Node(domNode& node);
};

class CColladaLoader
{

	boost::unordered_map<string, string> imageLib;

	vector<Node> CColladaLoader::convertModel(domCOLLADA& root);
	vector<Node> sceneNodes;
	BoundingBox g_modelMax;
	string MakeURI(const string strFilePath);
	void loadTextures(domImage_Array* pImageAry);

	//export

	bool addAsset(daeElement* root);
	template<typename T>
	daeTArray<T> rawArrayToDaeArray(T rawArray[], size_t count);
	string makeUriRef(const string& id);
	bool addSource(daeElement* mesh,
		const string& srcID,
		const string& srcName,
		const string& paramNames,
		domFloat values[],
		int valueCount);
	bool addNameSource(daeElement* mesh,
		const string& srcID,
		daeStringRef values[],
		int valueCount);
	bool addTransformSource(daeElement* mesh,
		const string& srcID,
		domFloat values[],
		int valueCount);
	bool addWeightSource(daeElement* mesh,
		const string& srcID,
		domFloat values[],
		int valueCount);
	bool addInput(daeElement* triangles,
		const string& semantic,
		const string& srcID,
		int offset);
	bool addGeometry(vector<tMaterialInfo> infos, t3DModel * pModel, daeElement* root);
	bool addControllers(ObjectSkeletonShaderData * objectSkeletonDataArray, SN::SkeletonNode * skeletonroot,  t3DModel * pModel, daeElement* root);
	bool addImages(t3DModel * pModel, daeElement* root);
	bool addEffects(t3DModel * pModel, daeElement* root);
	bool addMaterials(t3DModel * pModel, daeElement* root);
	bool addVisualScene(t3DModel * pModel, bool exportSkeleton, SN::SkeletonNode * skeletonroot, daeElement* root);

public:
	//CColladaLoader();
	//~CColladaLoader(void);

	//
	CMesh* LoadColladaFile(const string& strFilePath);
	bool LoadColladaModel(int importStyle, t3DModel * pModel, SN::SkeletonNode * bproot, vector<ObjectSkeletonShaderData*> &skeletonData, const string file);
	bool ExportToFile(t3DModel * pModel, ObjectSkeletonShaderData * objectSkeletonDataArray, const string file, SN::SkeletonNode * root, bool exportSkeleton);
};

CVector3 multiplyVectorAndMatrix(CVector3 v, domFloat4x4 m);
domFloat4x4 addMatrixToMatrix(domFloat4x4 m1, domFloat4x4 m2);
TNT::Array2D<float> VFloat4x4ToTNTMatrix(domFloat4x4 m);
TNT::Array2D<float> ScaleFloat3ToTNTMatrix(domFloat3 m);
TNT::Array2D<float> TranslateFloat3ToTNTMatrix(domFloat3 m);
TNT::Array2D<float> RotateFloat4ToTNTMatrix(domFloat4 m);
void copyObjectSkeletonShaderData(ObjectSkeletonShaderData * src, ObjectSkeletonShaderData * dest);
//---------------------------------------------------------------------------*/

#endif
#ifndef m_assimpLoaderH
#define m_assimpLoaderH
//---------------------------------------------------------------------------
#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif

#include <float.h>
#include <gl/glew.h>

#include <assimp/Importer.hpp>
#include <assimp/PostProcess.h>
#include <assimp/Scene.h>

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
//#include "s_skeletonNode.h"
#include <SkeletonNode/SkeletonNode.h>

//#include "SDF_Library.h"
#include <SDF_lib/Controllers/Model.h>
#include <SDF_lib/Structures/Math/Mat4.h>


void ImportModel(t3DModel * p_3DModel, char * strFileName, ModelController::CModel * sdfController, SN::SkeletonNode * pNode);
void recursiveModelFill(int *i, t3DModel * p_3DModel, const aiScene *sc,const aiNode* nd, BoundingBox * pModelMax, SN::SkeletonNode * pNode);

//---------------------------------------------------------------------------
#endif

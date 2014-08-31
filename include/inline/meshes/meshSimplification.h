#ifndef c_meshSimplificationH
#define c_meshSimplificationH

/*#ifdef _MMGR
#include "c_newdeleteDebug.h"
#endif*/

#include <JMSP/mesh.h>
#include <JMSP/pmesh.h>
#include <JMSP/triangle.h>
#include <JMSP/vec3.h>
#include <JMSP/vertex.h>

#include <meshes/structure.h>
//#include "s_skeleton.h"
#ifdef _LOG
	#include <logs/log.h>
#endif

class MeshSimplify {
	public:
		jmsp::Mesh* g_pMesh;
		jmsp::PMesh* g_pProgMesh;
		jmsp::PMesh::EdgeCost g_edgemethod;
		MeshSimplify();
		void setInitMesh(MeshGraph * pMesh, structure::t3DModel * pModel);
		void setMeshGraphVertices(MeshGraph * pMesh, MeshGraph * pMesh2);
		void simplifyMesh(int numberOfCollapses);
		void getVerticesToMeshGraph(MeshGraph * pMeshOld, structure::t3DModel * pModel);
};

#include "meshSimplification.inl"

#endif
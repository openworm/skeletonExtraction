#pragma once

#include <inline/meshes/MeshGraph.h>
#include <LBSE_lib/lbse_skeleton.h>

#ifdef _LOG
	#include <logs/log.h>
#endif

using namespace std;

namespace sdf {
	class Extractor {

	public:

		int skeletonTesselationFactor;

		void applyMedianMerge(MeshGraph * polyMesh, MeshGraph * pMesh, float threshold);
		void subdivideSkeletonByCrossSectionMedians(SN::SkeletonNode* pRoot, MeshGraph * pMesh, int segmentSubdivision);
		void subdivideSkeletonMeshGraphByCrossSectionMedians(MeshGraph * polyMesh, MeshGraph * pMesh, int segmentSubdivision);
	};
}
//---------------------------------------------------------------------------
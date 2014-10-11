#pragma once

#include <inline/meshes/MeshGraph.h>

using namespace std;

namespace sdf {
	class Extractor {

	public:
		void applyMedianMerge(MeshGraph * polyMesh, MeshGraph * pMesh, float threshold);
	};
}
//---------------------------------------------------------------------------
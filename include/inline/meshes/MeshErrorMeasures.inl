#include <algorithm>

namespace MEM {

	inline float MeanSquareError(meshes::IndexedFace *meshA, meshes::IndexedFace *meshB) {
		int vertices = meshA->vertices.size() < meshB->vertices.size() ? (meshA->vertices.size() / 3) : (meshB->vertices.size() / 3);

		float mse = 0.0;
		for (int i = 0; i < vertices; i++) {
			mmath::CVector3 P = meshA->GetVertex(i);
			mmath::CVector3 Q = meshB->GetVertex(i);
			mse += mmath::Distance(P, Q);
		}
		mse /= (float)vertices;

		return mse;
	}

	/*inline mmath::CVector3 AABBBottomCenter(meshes::IndexedFace *mesh) {
		if (mesh->vertices.empty()) return mmath::CVector3(0, 0, 0);

		int vertices = mesh->vertices.size() / 3;
		float Z = 0, minX = 0, maxX = 0, minY = 0, maxY = 0;

		for (int i = 0; i < vertices; i++) {
			mmath::CVector3 P = mesh->GetVertex(i);
			Z = std::min(Z, P.z);
			minX = std::min(minX, P.x);
			maxX = std::max(maxX, P.x);
			minY = std::min(minY, P.y);
			maxY = std::max(maxY, P.y);
		}

		return mmath::CVector3(0.5*minX + 0.5*maxX, 0.5*minY + 0.5*maxY, Z);
	}*/

	inline void ShiftMeshByPoint(meshes::IndexedFace *mesh, const mmath::CVector3 &P) {
		int vertices = mesh->vertices.size() / 3;
		std::vector<float> points;

		for (int i = 0; i < vertices; i++) {
			mmath::CVector3 Q = mesh->GetVertex(i);
			points.push_back(Q.x + P.x);
			points.push_back(Q.y + P.y);
			points.push_back(Q.z + P.z);
		}

		mesh->vertices = points;
	}

}
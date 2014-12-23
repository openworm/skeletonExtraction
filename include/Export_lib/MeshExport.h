#include <meshes\IndexedFace.h>
#include <fstream>

namespace Export {
	std::string PadNumber(int number, int places) {
		std::string result = "";
		while (powf(10, places) > number) {
			result += "0";
			places--;
		}
		result += std::to_string(number);
		return result;
	}

	void ExportMeshToFile(meshes::IndexedFace *mesh, std::string fileName) {
		int numOfVerts = mesh->vertices.size() / 3;
		int numOfTriangles = mesh->indices.size() / 3;

		std::ofstream off(fileName);

		off << "OFF" << std::endl;
		off << numOfVerts << " " << numOfTriangles << " 0" << std::endl;

		for (int i = 0; i < numOfVerts; i++) {
			mmath::CVector3 P = mesh->GetVertex(i);
			off << P.x << " " << P.y << " " << P.z << std::endl;
		}

		for (int i = 0; i < numOfTriangles; i++) {
			mmath::CVector3 T = mesh->GetTriangle(i);
			off << "3 " << (int)T.x << " " << (int)T.y << " " << (int)T.z << std::endl;
		}
	}
}
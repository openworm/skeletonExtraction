#pragma once
#include <string>
#include <vector>
#include <tinyxml\tinyxml2.h>
#include <meshes\IndexedFace.h>

namespace Loader {
	struct Parameters {
		std::string id;
		std::string name;
		std::vector<std::string> params;
		int elements;
	};

	class Exporter
	{
		tinyxml2::XMLDocument *document;
		tinyxml2::XMLElement *root;

		tinyxml2::XMLElement* ExportVectorToSource(std::vector<float> &data, Parameters &params);
		void ExportVertices(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		bool ExportNormals(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		bool ExportUVs(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		void ExportTriangles(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement, bool normals, bool uvs);
		void CleanDocument();
	public:
		Exporter(void);
		~Exporter(void);

		void GenerateHeader();
		bool ExportMesh(meshes::IndexedFace *mesh);
		bool SaveToFile(std::string fileName);
	};
}
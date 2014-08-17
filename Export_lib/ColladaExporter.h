#pragma once
#include <string>
#include <vector>
#include <tinyxml\tinyxml2.h>
#include <meshes\Mesh+Skin.h>
#include <SkeletonNode\SkeletonNode.h>

namespace Export {
	struct Parameters {
		std::string element;
		std::string id;
		std::string name;
		std::vector<std::pair<std::string, std::string> > params;
		int stride;
	};

	typedef enum {
		X_UP = 0,
		Y_UP,
		Z_UP
	} Axis;

	class ColladaExporter
	{
		std::string bindPosesID;
		std::string jointsID;
		std::string meshID;
		std::string skinID;
		std::string weightsID;
		std::string modelID;
		std::string upAxisValue;
		std::string authorName;

		tinyxml2::XMLDocument *document;
		tinyxml2::XMLElement *root;

		template< typename T > tinyxml2::XMLElement* ExportVectorToSource(std::vector<T> &data, Parameters &params);
		void ExportVertices(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		void ExportNormals(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		void ExportUVs(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		void ExportTriangles(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement);
		void ExportSkeleton(SN::SkeletonNode *aNode, tinyxml2::XMLElement *nodeRoot);
		void ExportSkeletonToJoints(SN::SkeletonNode *node, tinyxml2::XMLElement *element);
		void ExportSkeletonToMatrices(SN::SkeletonNode *node, tinyxml2::XMLElement *element);
		void ExportWeights(std::vector<float> &weights, tinyxml2::XMLElement *element);
		void ExportVertexWeights(meshes::MeshSkin *mesh, tinyxml2::XMLElement *element);

		void CleanDocument();

		void GenerateHeader();
		bool ExportMesh(meshes::IndexedFace *mesh);
		void ExportSkin(meshes::MeshSkin *mesh, SN::SkeletonNode *skeleton);
		void ExportSkeleton(SN::SkeletonNode *skeleton);
		void FinishMeshExport();
		bool SaveToFile(std::string fileName);
	public:
		ColladaExporter(void);
		ColladaExporter(std::string modelName);
		~ColladaExporter(void);

		void SetModelName(std::string modelName);
		void SetUpAxis(Axis axis);

		bool Export(meshes::IndexedFace *mesh, std::string fileName);
		bool Export(meshes::MeshSkin *mesh, SN::SkeletonNode *skeleton, std::string fileName);
	};
}
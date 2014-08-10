// Assimp.h : subor pre nacitanie / ulozenie modelov
#pragma once
#include "SDF_lib/Structures/Mesh/Face.h"
#include "SDF_lib/Structures/Math/Mat4.h"

namespace AssimpFileHandler
{
	using namespace std;
	using namespace GenericStructures;
	using namespace MeshStructures;

	class CAssimp
	{
	public:
		CAssimp();
		bool Import3DFromFile( const string& pFile);
		void logInfo(std::string logString);
		void logDebug(std::string logString);
		void LoadData(LinkedList<Face>* fc, LinkedList<Vertex>* pts);
		void SetScene(const struct aiScene *scena);
		~CAssimp();
	private:
		void RecursiveLoad (const struct aiScene *sc, const struct aiNode* nd, LinkedList<Face>* fc, LinkedList<Vertex>* pts);
	protected:
		void createAILogger();
		void destroyAILogger();
	};
}
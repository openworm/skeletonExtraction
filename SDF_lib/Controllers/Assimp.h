// Assimp.h : subor pre nacitanie / ulozenie modelov
#pragma once
#include "Face.h"
#include "Mat4.h"

#include "assimp/Importer.hpp"
#include "assimp/PostProcess.h"
#include "assimp/Scene.h"
#include "assimp/DefaultLogger.hpp"
#include "assimp/LogStream.hpp"

//#include <GL/glew.h>

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
		void RecursiveLoad (const struct aiScene *sc, const struct aiNode* nd, LinkedList<Face>* fc, LinkedList<Vertex>* pts, float * matrix);
	protected:
		void createAILogger();
		void destroyAILogger();
	};
}
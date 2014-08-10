// Model.h : subor pre kontrolu modelov
#pragma once
#include "SDF_lib/Structures/Mesh/Octree.h"
#include "SDF_lib/Controllers/SDFController.h"
#include "SDF_lib/Controllers/Assimp.h"
#include "SDF_lib/Helpers/MathHelper.h"	

namespace ModelController
{
	using namespace std;
	using namespace AssimpFileHandler;
	using namespace SDFController;

	const int color_step = 4;

	class CModel
	{
	public:
		CModel();
		~CModel();

		void logInfo(string logString);
		void logDebug(string logString);

		void LoadFile(string Filename);
		void LoadAssimp(aiScene* scene);
		//float GetSDF(const struct aiFace* face, bool smoothed);
		void Triangulate();
		float* GetSDF(int& size, bool smoothed);
		float* GetNormals(int& size);
		void SetNewPositions(float* pos);
		void ResetSettings();
		void ComputeBoundary();
		void CreateOctree();
		void SetColors();
		void ComputeSusedov();
		void GetBoundary(float &siz, float &x, float &y, float &z);
		void DrawModel();
		void ColorToRGB(int color, unsigned char &R, unsigned char &G, unsigned char &B);
		void HLSToRGB(float SDF_value, unsigned char &R, unsigned char &G, unsigned char &B);
		void setDrawMode(int mode);
		int getDrawMode();
		int GetTriangleCount();
		void ProcessPick(int x, int y);
		void ComputeSDF();
		void DeleteIdenticalVertices();
		void RecomputeNormals();

		bool loaded;
		bool show_octree;
		bool show_normals;
		Face* selected;

	private:
		CAssimp* Assimp;
		CSDFController* SDF_control;
		LinkedList<Face>* triangles;
		LinkedList<Vertex>* points;
		Octree* m_root;
		int draw_mode;						// 0 picking (default), 1 selected triangle, 2 SDF, 3 wireframe

		// rozmery modelu
		Vector4		b_stred;				// center of minN - maxN
		float		b_size;					// size for octree
		float		b_sf;					// scale factor
		float		b_max;					// diagonala v octree
	};
}
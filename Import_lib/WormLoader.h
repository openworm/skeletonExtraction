#pragma once
#include <meshes\IndexedFace.h>
#include <meshes/MeshErrorMeasures.h>
#include <memory>
#include <string>
#include <fstream>

#define ELASTIC_PARTICLES_LINE 7
#define LIQUID_PARTICLES_LINE 8
#define DATA_START_LINE 9

namespace Import {
	class WormLoader
	{
		std::string wormFile;
		std::string wormMeshFile;
		std::string wormMuscleFile;

		std::ifstream fileStream;
		int particlesPerIteration;

		void CleanFileStream();
		void ReadNextWormParticle(std::ifstream &fs, meshes::IndexedFace *mesh);
	public:
		WormLoader(void);
		WormLoader(std::string wormFile, std::string wormMeshFile);
		~WormLoader(void);
		
		// worm cuticle only

		bool SetupWormFile(std::string file);
		void SetWormFile(std::string file);
		void SetWormMeshFile(std::string file);
		void ReadWormHeader();
		void ReadNextWormIteration(meshes::IndexedFace *mesh);
		void ReadWormIteration(meshes::IndexedFace *mesh, int iteration);
		void ReadWormIteration(std::vector<meshes::IndexedFace> &meshes, int iteration);
		void ReadWormMesh(meshes::IndexedFace *mesh);

		// also with worm muscles
		void ReadWormMuscleWithIndex(meshes::IndexedFace * wholeMesh, std::vector<meshes::IndexedFace> &muscleMeshes, std::string muscleIndexFile);
		void ReadWormMuscleWithIndex(meshes::IndexedFace * wholeMesh, std::vector<meshes::IndexedFace> &muscleMeshes, std::vector<std::vector<int> > &muscleIdMap, std::string muscleIndexFile);
	};

#pragma region Additions
	void FixNormals(meshes::IndexedFace *mesh, bool swap = false, bool fixTriangles = true);
	void CalculatePerVertexNormals(meshes::IndexedFace *mesh);
	void CreateMeshForMuscle(meshes::IndexedFace *muscle);
	void CreateMeshForMuscle(meshes::IndexedFace *muscle, mmath::CVector3 start, mmath::CVector3 end);
	void TransferMeshFromMuscles(meshes::IndexedFace *muscle, std::vector<meshes::IndexedFace*> &muscles, std::vector<std::vector<int> > &muscleIdMap);
#pragma endregion
}

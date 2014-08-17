#pragma once
#include <meshes\IndexedFace.h>
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
		std::ifstream fileStream;
		int particlesPerIteration;

		void CleanFileStream();
		void ReadNextWormParticle(std::ifstream &fs, meshes::IndexedFace *mesh);
	public:
		WormLoader(void);
		WormLoader(std::string wormFile, std::string wormMeshFile);
		~WormLoader(void);
		
		bool SetupWormFile(std::string file);
		void SetWormFile(std::string file);
		void SetWormMeshFile(std::string file);
		void ReadWormHeader();
		void ReadNextWormIteration(meshes::IndexedFace *mesh);
		void ReadWormIteration(meshes::IndexedFace *mesh, int iteration);
		void ReadWormMesh(meshes::IndexedFace *mesh);
	};
}

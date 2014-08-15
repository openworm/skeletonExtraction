#pragma once
#include <meshes\IndexedFace.h>
#include <memory>
#include <string>
#include <fstream>

#define ELASTIC_PARTICLES_LINE 7
#define LIQUID_PARTICLES_LINE 8
#define DATA_START_LINE 9

namespace Loader {
	typedef struct {
		float boundingBox[6];
		int elasticParticles;
		int liquidParticles;
	} Header;

	class WormLoader
	{
		std::string wormFile;
		std::ifstream fileStream;
		int particlesPerIteration;

		void CleanFileStream();
	public:
		WormLoader(void);
		~WormLoader(void);

		bool SetWormFile(std::string file);
		void ReadWormHeader();
		std::unique_ptr<meshes::IndexedFace> GetWormIteration();
		std::unique_ptr<meshes::IndexedFace> GetWormIteration(int iteration);
	};
}

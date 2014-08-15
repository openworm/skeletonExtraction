#include "WormLoader.h"
#include <sstream>

namespace Loader {
	WormLoader::WormLoader(void) : particlesPerIteration(0)
	{
	}


	WormLoader::~WormLoader(void)
	{
		CleanFileStream();
	}

	void WormLoader::CleanFileStream() {
		if (fileStream.is_open()) {
			fileStream.close();
		}
	}

	bool WormLoader::SetWormFile(std::string file) {
		CleanFileStream();
		wormFile = file;
		fileStream.open(wormFile, std::ifstream::in);
		bool good = fileStream.is_open();
		if (good) {
			ReadWormHeader();
		}

		return good;
	}

	void WormLoader::ReadWormHeader() {
		float dummy = 0, elasticParticles = 0, liquidParticles = 0;

		fileStream >> dummy;
		fileStream >> dummy;
		fileStream >> dummy;
		fileStream >> dummy;
		fileStream >> dummy;
		fileStream >> dummy;
		fileStream >> elasticParticles;
		fileStream >> liquidParticles;

		particlesPerIteration = elasticParticles + liquidParticles;
	}

	std::unique_ptr<meshes::IndexedFace> WormLoader::GetWormIteration() {
		if (!fileStream.is_open()) return std::unique_ptr<meshes::IndexedFace>(nullptr);
		//create mesh structure
		std::unique_ptr<meshes::IndexedFace> mesh(new meshes::IndexedFace());
		mesh->vertices.reserve(particlesPerIteration*3);
		//load particles
		int i = 0;
		while (fileStream.good() && i < particlesPerIteration) {
			float x = 0, y = 0, z = 0, dummy = 0;
			fileStream >> x >> y >> z >> dummy;
			mesh->vertices.push_back(x);
			mesh->vertices.push_back(y);
			mesh->vertices.push_back(z);
			i++;
		}

		return mesh;
	}

	std::unique_ptr<meshes::IndexedFace> WormLoader::GetWormIteration(int iteration) {
		std::ifstream fs(wormFile, std::ifstream::in | std::ifstream::binary);
		//read header
		int elasticParticles = 0, liqudParticles = 0;
		fs.seekg(6*sizeof(float));
		fs.read((char*)(&elasticParticles), sizeof(int));
		fs.read((char*)(&liqudParticles), sizeof(int));
		int particlesPerIteration = elasticParticles + liqudParticles;
		fs.seekg((iteration)*particlesPerIteration*3*sizeof(float), std::ifstream::cur);
		std::unique_ptr<meshes::IndexedFace> mesh(new meshes::IndexedFace());
		//read data
		mesh->vertices.reserve(particlesPerIteration*3);
		for (int i = 0; i < particlesPerIteration; i++) {
			for (int j = 0; j < 3; j++) {
				float value = 0;
				fs.read((char*)(&value), sizeof(float));
				mesh->vertices.push_back(value);
			}
		}
		fs.close();
		return mesh;
	}
}
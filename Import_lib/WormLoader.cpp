/************************************************************************************
*
* The MIT License (MIT)
* 
* Copyright (c) 2014 Michal Piovarèi
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
***********************************************************************************/

#include "WormLoader.h"
#include <sstream>

namespace Import {
	WormLoader::WormLoader(void) : particlesPerIteration(0)
	{
	}

	WormLoader::WormLoader(std::string wormFile, std::string wormMeshFile) : wormFile(wormFile), wormMeshFile(wormMeshFile)
	{
		//SetWormFile(wormFile);
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

	bool WormLoader::SetupWormFile(std::string file) {
		CleanFileStream();
		wormFile = file;
		fileStream.open(wormFile, std::ifstream::in);
		bool good = fileStream.is_open();
		if (good) {
			ReadWormHeader();
		}

		return good;
	}

	void WormLoader::SetWormFile(std::string file) {
		wormFile = file;
	}

	void WormLoader::SetWormMeshFile(std::string file) {
		wormMeshFile = file;
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

	void WormLoader::ReadNextWormIteration(meshes::IndexedFace *mesh) {
		if (!fileStream.is_open() || !mesh) return;
		//create mesh structure
		mesh->vertices.clear();
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
		//maybe clear mesh if not loaded correctly?
	}

	void WormLoader::ReadNextWormParticle(std::ifstream &fs, meshes::IndexedFace *mesh) {
		int i = 0;
		while (fs.good() && i < 3) {
			float value = 0;
			fs.read((char*)(&value), sizeof(float));
			mesh->vertices.push_back(value);
			i++;
		}
		if (fs.good()) {
			float dummy = 0;
			fs.read((char*)(&dummy), sizeof(float));
		}
	}

	void WormLoader::ReadWormIteration(meshes::IndexedFace *mesh, int iteration) {
		if (!mesh) return;

		std::ifstream fs(wormFile, std::ifstream::in | std::ifstream::binary);
		//read header
		float iterations = 0, elasticParticles = 0, liqudParticles = 0;
		fs.seekg(6*sizeof(float));
		fs.read((char*)(&iterations), sizeof(float));
		fs.read((char*)(&elasticParticles), sizeof(float));
		fs.read((char*)(&liqudParticles), sizeof(float));
		int particlesPerIteration = elasticParticles + liqudParticles;
		if (iteration > iterations) return;
		fs.seekg(iteration*particlesPerIteration*4*sizeof(float), std::ifstream::cur);
		//read data
		mesh->vertices.clear();
		mesh->vertices.reserve(particlesPerIteration*3);
		int i = 0;
		while (fs.good() && i < particlesPerIteration) {
			ReadNextWormParticle(fs, mesh);
			i++;
		}
		fs.close();
		//maybe clear mesh if not loaded correctly?
	}

	void WormLoader::ReadWormMesh(meshes::IndexedFace *mesh) {
		if (!mesh) return;

		int verts = mesh->vertices.size()/3;
		std::ifstream inputStream(wormMeshFile);
		if (inputStream.is_open()) {
			while (inputStream.good()) {
				int i = 0, j = 0, k = 0;
				inputStream >> i >> j >> k;
				if (i < verts && j < verts && k < verts) {
					mesh->indices.push_back(i);
					mesh->indices.push_back(j);
					mesh->indices.push_back(k);
				}
			}
			inputStream.close();
		}
	}
}
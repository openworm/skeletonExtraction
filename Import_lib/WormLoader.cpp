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
#include <queue>

namespace Import {
	typedef std::pair<int, mmath::CVector3> TrianglePair;
	typedef std::pair<int, int> Edge;

	namespace {
		mmath::CVector3 CalculateNormal(meshes::IndexedFace *mesh, mmath::CVector3 triangle) {
			mmath::CVector3 A = mesh->GetVertex(triangle.x);
			mmath::CVector3 B = mesh->GetVertex(triangle.y);
			mmath::CVector3 C = mesh->GetVertex(triangle.z);
			mmath::CVector3 normal = mmath::Normalize(mmath::Cross(B - A, C - A));
			return normal;
		}

		bool CommonEdge(mmath::CVector3 A, mmath::CVector3 B) {
			//one extra for cycle
			int a[4];
			a[0] = A.x; a[1] = A.y; a[2] = A.z; a[3] = A.x; 
			int b[4];
			b[0] = B.x; b[1] = B.y; b[2] = B.z; b[3] = B.x;

			for (int i = 0; i < 3; i++) {
				Edge edge1 = std::make_pair(a[i], a[i+1]);
				for (int j = 0; j < 3; j++) {
					Edge edge2 = std::make_pair(b[j], b[j+1]);
					if ((edge1.first == edge2.first && edge1.second == edge2.second) || (edge1.first == edge2.second && edge1.second == edge2.first)) {
						return true;
					}
				}
			}
			return false;
		}

		void FindNeighbors(meshes::IndexedFace *mesh, TrianglePair pair, std::vector<TrianglePair> &neighbors) {
			int numOfTriangles = mesh->indices.size() / 3;
			for (int i = 0; i < numOfTriangles; i++) {
				mmath::CVector3 triangle = mesh->GetTriangle(i);
				//add if they have common edge
				if (CommonEdge(pair.second, triangle)) {
					neighbors.push_back(std::make_pair(i, triangle));
				}
			}
		}

		bool Inside(TrianglePair pair, std::vector<TrianglePair> visited) {
			for (int i = 0; i < visited.size(); i++) {
				if (pair.first == visited[i].first) return true;
			}

			return false;
		}

		void NeighboringTriangles(meshes::IndexedFace *mesh, int vertex, std::vector<TrianglePair> &neighbors) {
			int numOfTriangles = mesh->indices.size() / 3;
			for (int i = 0; i < numOfTriangles; i++) {
				mmath::CVector3 triangle = mesh->GetTriangle(i);
				//add if they have common vertex
				if (triangle.x == vertex || triangle.y == vertex || triangle.z == vertex) {
					neighbors.push_back(std::make_pair(i, triangle));
				}
			}
		}
	}

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

#pragma region Additions

	void FixNormals(meshes::IndexedFace *mesh, bool swap, bool fixTriangles) {
		int numOfTriangles = mesh->indices.size() / 3;
		int numOfVertices = mesh->vertices.size() / 3;
		//get first triangle
		mmath::CVector3 triangle = mesh->GetTriangle(0);
		//calculate normal
		mmath::CVector3 normal = CalculateNormal(mesh, triangle);
		//swap if neeeded
		if (swap) normal = -normal;
		//create visited and queue
		std::queue<TrianglePair> queue;
		std::vector<TrianglePair> visited;
		std::vector<mmath::CVector3> normals;
		normals.resize(numOfTriangles);

		queue.push(std::make_pair(0, triangle));
		normals[0] = normal;
		visited.push_back(queue.front());

		while(!queue.empty()) {
			TrianglePair pair = queue.front();
			queue.pop();
			//find neighbors
			std::vector<TrianglePair> neighbors;
			FindNeighbors(mesh, pair, neighbors);
			for (int i = 0; i < neighbors.size(); i++) {
				//if neighbor not in visited
				if (Inside(neighbors[i], visited)) continue;
				//calculate neighbor normal
				mmath::CVector3 aNormal = CalculateNormal(mesh, neighbors[i].second);
				//swap with dot product
				if (mmath::Dot(normals[pair.first], aNormal) < 0) {
					aNormal = -aNormal;
					if (fixTriangles) {
						int temp = mesh->indices[neighbors[i].first * 3 + 0];
						mesh->indices[neighbors[i].first * 3 + 0] = mesh->indices[neighbors[i].first * 3 + 2];
						mesh->indices[neighbors[i].first * 3 + 2] = temp;
					}
				}
				//add to queue and visited
				queue.push(neighbors[i]);
				visited.push_back(neighbors[i]);
				//add normal to array at triangle index
				normals[neighbors[i].first] = aNormal;
			}
		}

		//for each vertex
		for (int i = 0; i < numOfVertices; i++) {
			//find neighboring triangles
			std::vector<TrianglePair> triangles;
			NeighboringTriangles(mesh, i, triangles);
			//calculate mean normal
			mmath::CVector3 normal;
			for (int i = 0; i < triangles.size(); i++) {
				normal = normal + normals[triangles[i].first];
			}
			normal = mmath::Normalize(normal);
			//store normal in mesh
			mesh->normals.push_back(normal.x);
			mesh->normals.push_back(normal.y);
			mesh->normals.push_back(normal.z);
		}
	}

#pragma endregion
}
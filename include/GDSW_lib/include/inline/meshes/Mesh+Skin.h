#pragma once
#include <meshes/IndexedFace.h>
#include <string>

namespace meshes {
	struct MeshSkin : IndexedFace
	{
		std::vector<float> skinWeights;
		std::vector<int> jointIDs;
		std::vector<int> influences;

		MeshSkin() : IndexedFace() {};
		MeshSkin(IndexedFace *mesh) {
			vertices = mesh->vertices;
			normals = mesh->normals;
			tangents = mesh->tangents;
			uvs = mesh->uvs;
			indices = mesh->indices;
			fdata = mesh->fdata;
			idata = mesh->idata;
		};
	};
}
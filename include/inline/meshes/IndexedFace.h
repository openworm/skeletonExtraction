#pragma once
#include <vector>

namespace meshes {
	struct IndexedFace
	{
		std::vector<float> vertices;
		std::vector<float> normals;
		std::vector<float> tangents;
		std::vector<float> uvs;
		std::vector<int> indices;
		std::vector<float> fdata;
		std::vector<int> idata;
		IndexedFace() {};
		IndexedFace(const std::vector<float> &vertices, const std::vector<int> &indices) : vertices(vertices), indices(indices) {};
	};
}
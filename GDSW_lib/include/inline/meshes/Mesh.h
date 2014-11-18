#pragma once
#include <meshes/IndexedFace.h>
#include <string>
#include <glm\glm.hpp>

namespace meshes {

	struct Material {
		std::string material;
		glm::vec4 diffuseColor;
		int TextureID;
		std::string texFile;
	};

	struct BoundingBox {
		float x_min;
		float x_max;
		float y_min;
		float y_max;
		float z_min;
		float z_max;

		BoundingBox(){
			x_min = FLT_MAX;
			x_max = -FLT_MAX;
			y_min = FLT_MAX;
			y_max = -FLT_MAX;
			z_min = FLT_MAX;
			z_max = -FLT_MAX;
		}
	};

	struct Mesh : IndexedFace
	{
		std::vector<Material> materialDefinitions;
		BoundingBox modelbb;
		std::vector<int> materialIndices;

		Mesh() : IndexedFace() {};
		~Mesh() {};
	};
}
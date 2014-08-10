#pragma once
#include <meshes/IndexedFace.h>
#include <string>
#include <glm\glm.hpp>

namespace meshes {
	struct Mesh : IndexedFace
	{
		std::string material;
		glm::vec4 diffuseColor;
		Mesh() : IndexedFace() {};
	};
}
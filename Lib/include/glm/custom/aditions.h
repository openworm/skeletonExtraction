#pragma once

#ifndef GLM_CUSTOM_aditions
#define GLM_CUSTOM_aditions GLM_VERSION

#include "..\glm.hpp"
#include "..\gtc\quaternion.hpp"
#include "..\gtx\quaternion.hpp"

namespace glm {
	bool operator==(const glm::vec3 &vecA, const glm::vec3 &vecB);
	bool operator!=(const glm::vec3 &vecA, const glm::vec3 &vecB);

	bool IsZeroLength(vec3 v);

	quat QuatBetweenVectors(vec3 start, vec3 dest, vec3 fallbackAxis = glm::vec3(0));

	quat QuatBetweenVectorsWithAxis(vec3 start, vec3 dest, vec3 axis);

	float AngleFromQuaternion(const quat &q);

	glm::vec2 CubicBezierInterpolate(const glm::vec2 &P0, const glm::vec2 &P1, const glm::vec2 &P2, const glm::vec2 &P3, float t);
}

#include "aditions.inl"

#endif
#include "m_math_additions.h"
#include <glm\gtc\matrix_transform.hpp>

using namespace mmath;

CVector3 TransformCPoint(CVector3 u, glm::mat4 M) {
	glm::vec4 v(u.x, u.y, u.z, 1.0);
	glm::vec4 r = M * v;
	return CPoint(r.x, r.y, r.z);
}

glm::mat4 AddRotation(glm::mat4 M, CVector4 axisAngle, CPoint center) {
	if (IsZeroLength(CVector3(axisAngle.i, axisAngle.j, axisAngle.k))) return M;

	glm::vec3 c(center.x, center.y, center.z);
	glm::vec3 axis(axisAngle.i, axisAngle.j, axisAngle.k);
	float angle = axisAngle.s;
	//float angle = axisAngle.s*180.0f/M_PI;

	glm::mat4 T = glm::translate(glm::mat4(), -c);
	glm::mat4 R = glm::rotate(glm::mat4(), angle, axis);
	glm::mat4 TB = glm::translate(glm::mat4(), c);

	return TB * R * T * M;
}
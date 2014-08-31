#pragma once
#include <mmath\mmath.h>
#include <glm\glm.hpp>

using namespace mmath;

typedef CVector3 CPoint;

CPoint TransformCPoint(CPoint u, glm::mat4 M);
glm::mat4 AddRotation(glm::mat4 M, CVector4 axisAngle, CPoint center);
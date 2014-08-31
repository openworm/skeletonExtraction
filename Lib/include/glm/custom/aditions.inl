namespace glm {
	GLM_FUNC_QUALIFIER bool operator==(const glm::vec3 &vecA, const glm::vec3 &vecB) 
	{ 
		const double epsilion = 0.000001;  // choose something apprpriate.

		return    fabs(vecA[0] -vecB[0]) < epsilion   
			&& fabs(vecA[1] -vecB[1]) < epsilion   
			&& fabs(vecA[2] -vecB[2]) < epsilion;
	}; 
	GLM_FUNC_QUALIFIER bool operator!=(const glm::vec3 &vecA, const glm::vec3 &vecB) 
	{ 
		return !(vecA == vecB);
	}; 

	GLM_FUNC_QUALIFIER bool IsZeroLength(vec3 v) {
		float sqlen = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
		return (sqlen < (1e-06 * 1e-06));
	}


	GLM_FUNC_QUALIFIER quat QuatBetweenVectors(vec3 start, vec3 dest, vec3 fallbackAxis) {
		// Based on Stan Melax's article in Game Programming Gems
		quat q;
		// Copy, since cannot modify local
		vec3 v0 = normalize(start);
		vec3 v1 = normalize(dest);

		float d = dot(v0, v1);
		// If dot == 1, vectors are the same
		if (d >= 1.0f) {
			return q;
		}
		if (d < (1e-6f - 1.0f)) {
			if (fallbackAxis != glm::vec3(0)) {
				// rotate 180 degrees about the fallback axis
				q = angleAxis<float>(180, fallbackAxis);
			} else {
				// Generate an axis
				vec3 axis = cross(vec3(1, 0, 0), v0);
				if (IsZeroLength(axis)) { // pick another if colinear
					axis = cross(vec3(0, 1, 0), v0);
				}
				axis = normalize(axis);
				q = angleAxis<float>(180, axis);
			}
		} else {//alpha from dot product, axis from cross product
			float s = sqrtf((1+d)*2);
			float invs = 1 / s;

			vec3 c = cross(v0, v1);

			q.x = c.x * invs;
			q.y = c.y * invs;
			q.z = c.z * invs;
			q.w = s * 0.5f;
			q = normalize(q);
		}

		return q;
	}

	GLM_FUNC_QUALIFIER quat QuatBetweenVectorsWithAxis(vec3 start, vec3 dest, vec3 axis) {
		// Copy, since cannot modify local
		vec3 v0 = start;
		v0 = normalize(v0 - axis*dot(v0, axis));
		vec3 v1 = dest;
		v1 = normalize(v1 - axis*dot(v1, axis));

		float d = dot(v0, v1);
		// If dot == 1, vectors are the same
		if (d >= 1.0f) {
			return quat(1, 0, 0, 0);
		}
		if (d < (1e-6f - 1.0f)) {
			return angleAxis<float>(180, axis);
		}

		float alfa = acosf(d);
		return angleAxis<float>(glm::degrees(alfa), axis);
	}
}
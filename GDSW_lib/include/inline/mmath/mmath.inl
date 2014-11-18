namespace mmath {
	inline float Absolute(float num)
	{
		// If num is less than zero, we want to return the absolute value of num.
		// This is simple, either we times num by -1 or subtract it from 0.
		if(num < 0)
			return (0 - num);

		// Return the original number because it was already positive
		return num;
	}


	/////////////////////////////////////// CROSS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns a perpendicular vector from 2 given vectors by taking the cross product.
	/////
	/////////////////////////////////////// CROSS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline CVector3 Cross(CVector3 vVector1, CVector3 vVector2)
	{
		CVector3 vNormal;									// The vector to hold the cross product

		// The X value for the vector is:  (V1.y * V2.z) - (V1.z * V2.y)													// Get the X value
		vNormal.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

		// The Y value for the vector is:  (V1.z * V2.x) - (V1.x * V2.z)
		vNormal.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

		// The Z value for the vector is:  (V1.x * V2.y) - (V1.y * V2.x)
		vNormal.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

		return vNormal;										// Return the cross product (Direction the polygon is facing - Normal)
	}


	inline CVector4 QuaternionToAxisAngle(CVector4 q){
		float scale = sqrt(pow(q.i, 2) + pow(q.j, 2) + pow(q.k, 2));
		if (scale != 0)
			return CVector4(2 * acos(q.s), q.i / scale, q.j / scale, q.k / scale);
		return  CVector4(0,0,0,0);
	}

	inline Array2D<float> QuaternionToMatrix3x3(CVector4 q){
		Array2D<float> m(3,3,0.0f);
		/*m[0][0] = pow(q.s, 2) + pow(q.i, 2) - pow(q.j, 2) - pow(q.k, 2);
		m[1][0] = 2 * q.i * q.j + 2 * q.s * q.k;
		m[2][0] = 2 * q.i * q.k - 2 * q.s * q.j;

		m[0][1] = 2 * q.i * q.j - 2 * q.s * q.k;
		m[1][1] = pow(q.s, 2) - pow(q.i, 2) + pow(q.j, 2) - pow(q.k, 2);
		m[2][1] = 2 * q.j * q.k + 2 * q.s * q.i;

		m[0][2] = 2 * q.i * q.k + 2 * q.s * q.j;
		m[1][2] = 2 * q.j * q.k - 2 * q.s * q.i;
		m[2][2] = pow(q.s, 2) - pow(q.i, 2) - pow(q.j, 2) + pow(q.k, 2);*/

		float Nq = pow(q.i, 2) + pow(q.j, 2) + pow(q.k, 2) + pow(q.s, 2);
		float s;
		if (Nq > 0.0)
			s = 2.0/Nq; 
		else
			s = 0.0;

		float X = q.i*s;
		float Y = q.j*s;
		float Z = q.k*s;
		float wX = q.s*X;
		float wY = q.s*Y;
		float wZ = q.s*Z;
		float xX = q.i*X;
		float xY = q.i*Y;
		float xZ = q.i*Z;
		float yY = q.j*Y;
		float yZ = q.j*Z;
		float zZ = q.k*Z;

		float xx = pow(q.i, 2);
		float yy = pow(q.j, 2);
		float zz = pow(q.k, 2);

		m[0][0] = 1.0-(yY+zZ);
		m[1][0] = xY+wZ;
		m[2][0] = xZ-wY;

		m[0][1] = xY-wZ;
		m[1][1] = 1.0-(xX+zZ);
		m[2][1] = yZ+wX;

		m[0][2] = xZ+wY;
		m[1][2] = yZ-wX;
		m[2][2] = 1.0-(xX+yY);

		return m;
	}

	inline Array2D<float> Rot3x3ToRot4x4(Array2D<float> in){
		Array2D<float> out(4,4,0.0f);
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++)
				out[i][j] = in[i][j];
		out[3][3] = 1.0f;
		return out;
	}

	inline Array2D<float> CVecToTntVec (CVector3 vec){
		Array2D<float> ret(1,3,0.0f);
		ret[0][0] = vec.x;
		ret[0][1] = vec.y;
		ret[0][2] = vec.z;
		return ret;
	}

	inline Array2D<float> QuaternionToTntVec (CVector4 q){
		Array2D<float> ret(1,4,0.0f);
		ret[0][0] = q.i;
		ret[0][1] = q.j;
		ret[0][2] = q.k;
		ret[0][3] = q.s;
		return ret;
	}

	inline CVector4 TntVecToQuaternion (Array2D<float> vec){
		if (vec.dim1() == 4)
			return CVector4(vec[3][0], vec[0][0], vec[1][0], vec[2][0]);
		else
			return CVector4(vec[0][3], vec[0][0], vec[0][1], vec[0][2]);
	}

	inline Array2D<float> CVecToTntVec4(CVector3 vec){
		Array2D<float> ret(1,4,0.0f);
		ret[0][0] = vec.x;
		ret[0][1] = vec.y;
		ret[0][2] = vec.z;
		ret[0][3] = 1.0f;
		return ret;
	}

	inline float * TntMatrix4x4ToFloatPointer(Array2D<float> m){
		float * fm = new float[16];
		for (int i=0; i<4; i++)
			for (int j=0; j<4; j++)
				fm[i * 4 + j] = m[i][j];
		return fm;
	}

	inline CVector3 TntVecToCVec (Array2D<float> vec){
		if (vec.dim1() == 3)
			return CVector3(vec[0][0], vec[1][0], vec[2][0]);
		else
			return CVector3(vec[0][0], vec[0][1], vec[0][2]);
	}

	inline CVector4 AxisRotToQuaternion(float angle, CVector3 axis){
		angle *= 0.5f;
		float sinAngle = sin(angle);

		return CVector4(cos(angle), axis.x * sinAngle, axis.y * sinAngle, axis.z * sinAngle);
	}

	inline CVector4 Matrix3x3ToQuaternion(Array2D<float> m){
		float d = m[0][0] + m[1][1] + m[2][2];
		if (d <= -1){
			float x = (m[0][1] * m[0][2]) / sqrt( pow(m[0][1] * m[0][2], 2) + pow(m[0][1] * m[1][2], 2) + pow(m[0][2] * m[1][2], 2) );
			float y = (m[0][1] * m[1][2]) / sqrt( pow(m[0][1] * m[0][2], 2) + pow(m[0][1] * m[1][2], 2) + pow(m[0][2] * m[1][2], 2) );
			float z = (m[0][2] * m[1][2]) / sqrt( pow(m[0][1] * m[0][2], 2) + pow(m[0][1] * m[1][2], 2) + pow(m[0][2] * m[1][2], 2) );
			return CVector4(0, x, y, z);
		}
		float w = 0.5 * sqrt(1 + d);
		float x = (m[2][1] - m[1][2]) / (4*w);
		float y = (m[0][2] - m[2][0]) / (4*w);
		float z = (m[1][0] - m[0][1]) / (4*w);
		if (w == 0 && x == 0 && y == 0 && z == 0)
			return CVector4(0, 1, 1, 1);
		else
			return CVector4(w, x, y, z);
	}

	inline CVector4 lerp(CVector4 v0, CVector4 v1, float t){
		return Normalize((v0*(1-t) + v1*t)); 
	}

	inline CVector4 slerp(CVector4 v0, CVector4 v1, double t) {
		// v0 and v1 should be unit length or else
		// something broken will happen.

		// Compute the cosine of the angle between the two vectors.
		float dot = Dot(v0, v1);

		const double DOT_THRESHOLD = 0.9995;
		if (dot > DOT_THRESHOLD) {
			// If the inputs are too close for comfort, linearly interpolate
			// and normalize the result.

			return Normalize(lerp(v0, v1, t));
		}

		//Clamp(dot, -1, 1);           // Robustness: Stay within domain of acos()
		float theta_0 = acos(dot);  // theta_0 = angle between input vectors
		float theta = theta_0*t;    // theta = angle between v0 and result 

		CVector4 v2;
		v2 = v1 - v0*dot;
		v2 = Normalize(v2);              // { v0, v2 } is now an orthonormal basis

		return v0*cos(theta) + v2*sin(theta);
	}

	inline CVector4 QuaternionProduct(CVector4 vVector1, CVector4 vVector2)
	{
		float q0 = vVector1.s;
		float q1 = vVector1.i;
		float q2 = vVector1.j;
		float q3 = vVector1.k;

		float r0 = vVector2.s;
		float r1 = vVector2.i;
		float r2 = vVector2.j;
		float r3 = vVector2.k;
		/*
		float t0 = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
		float t1 = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
		float t2 = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
		float t3 = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;*/

		float t0 = q0 * q1 - Dot(vVector1, vVector2);
		CVector3 t1 = CVector3(vVector2.i, vVector2.j, vVector2.k) * q0;
		CVector3 t2 = CVector3(vVector1.i, vVector1.j, vVector1.k) * r0;
		CVector3 t3 = Cross(CVector3(vVector1.i, vVector1.j, vVector1.k), CVector3(vVector2.i, vVector2.j, vVector2.k));
		CVector3 s = t1 + t2 + t3;

		return CVector4(t0, s.x, s.y, s.z);

		//return CVector4(t0, t1, t2, t3);
	}

	inline float Dot(CVector4 vVector1, CVector4 vVector2)
	{
		return ( (vVector1.i * vVector2.i) + (vVector1.j * vVector2.j) + (vVector1.k * vVector2.k));
	}

	inline float Magnitude(CVector4 q)
	{
		double v = (q.s * q.s) + (q.i * q.i) + (q.j * q.j) + (q.k * q.k);
		return (float)sqrt( v );
	}

	inline CVector4 Normalize(CVector4 q)
	{
		float magnitude = Magnitude(q);				


		q.s /= magnitude;								
		q.i /= magnitude;								
		q.j /= magnitude;								
		q.k /= magnitude;	



		return q;										
	}


	/////////////////////////////////////// MAGNITUDE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the magnitude of a normal (or any other vector)
	/////
	/////////////////////////////////////// MAGNITUDE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline float Magnitude(CVector3 vNormal)
	{
		// This will give us the magnitude or "Norm" as some say, of our normal.
		// Here is the equation:  magnitude = sqrt(V.x^2 + V.y^2 + V.z^2)  Where V is the vector
		double v = (vNormal.x * vNormal.x) + (vNormal.y * vNormal.y) + (vNormal.z * vNormal.z);
		return (float)sqrt( v );
	}


	/////////////////////////////////////// NORMALIZE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns a normalize vector (A vector exactly of length 1)
	/////
	/////////////////////////////////////// NORMALIZE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline CVector3 Normalize(CVector3 vNormal)
	{
		float magnitude = Magnitude(vNormal);				// Get the magnitude of our normal

		// Now that we have the magnitude, we can divide our normal by that magnitude.
		// That will make our normal a total length of 1.  This makes it easier to work with too.

		vNormal.x /= magnitude;								// Divide the X value of our normal by it's magnitude
		vNormal.y /= magnitude;								// Divide the Y value of our normal by it's magnitude
		vNormal.z /= magnitude;								// Divide the Z value of our normal by it's magnitude

		// Finally, return our normalized normal.

		return vNormal;										// Return the new normal of length 1.
	}


	/////////////////////////////////////// NORMAL \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the normal of a polygon (The direction the polygon is facing)
	/////
	/////////////////////////////////////// NORMAL \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline CVector3 Normal(CVector3 vPolygon[])
	{														// Get 2 vectors from the polygon (2 sides), Remember the order!
		CVector3 vVector1 = vPolygon[2] - vPolygon[0];
		CVector3 vVector2 = vPolygon[1] - vPolygon[0];

		CVector3 vNormal = Cross(vVector1, vVector2);		// Take the cross product of our 2 vectors to get a perpendicular vector

		// Now we have a normal, but it's at a strange length, so let's make it length 1.

		vNormal = Normalize(vNormal);						// Use our function we created to normalize the normal (Makes it a length of one)

		return vNormal;										// Return our normal at our desired length
	}


	/////////////////////////////////// DISTANCE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the distance between 2 3D points
	/////
	/////////////////////////////////// DISTANCE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline float Distance(CVector3 vPoint1, CVector3 vPoint2)
	{
		// This is the classic formula used in beginning algebra to return the
		// distance between 2 points.  Since it's 3D, we just add the z dimension:
		//
		// Distance = sqrt(  (P2.x - P1.x)^2 + (P2.y - P1.y)^2 + (P2.z - P1.z)^2 )
		//

		double distance = sqrt( (vPoint2.x - vPoint1.x) * (vPoint2.x - vPoint1.x) +
			(vPoint2.y - vPoint1.y) * (vPoint2.y - vPoint1.y) +
			(vPoint2.z - vPoint1.z) * (vPoint2.z - vPoint1.z) );

		// Return the distance between the 2 points
		return (float)distance;
	}
	

	inline float Distance(CVector2 vPoint1, CVector2 vPoint2)
	{

		double distance = sqrt( (vPoint2.x - vPoint1.x) * (vPoint2.x - vPoint1.x) +
			(vPoint2.y - vPoint1.y) * (vPoint2.y - vPoint1.y));

		// Return the distance between the 2 points
		return (float)distance;
	}


	inline float DistancePointToLine(CVector3 vA, CVector3 vB, CVector3 vPoint){
		return Magnitude(Cross(vPoint-vA, vPoint - vB)) / Magnitude(vB - vA);
	}


	////////////////////////////// CLOSEST POINT ON LINE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the point on the line vA_vB that is closest to the point vPoint
	/////
	////////////////////////////// CLOSEST POINT ON LINE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline CVector3 ClosestPointOnLine(CVector3 vA, CVector3 vB, CVector3 vPoint)
	{
		// Create the vector from end point vA to our point vPoint.
		CVector3 vVector1 = vPoint - vA;

		// Create a normalized direction vector from end point vA to end point vB
		CVector3 vVector2 = Normalize(vB - vA);

		// Use the distance formula to find the distance of the line segment (or magnitude)
		float d = Distance(vA, vB);

		// Using the dot product, we project the vVector1 onto the vector vVector2.
		// This essentially gives us the distance from our projected vector from vA.
		float t = Dot(vVector2, vVector1);

		// If our projected distance from vA, "t", is less than or equal to 0, it must
		// be closest to the end point vA.  We want to return this end point.
		if (t <= 0)
			return vA;

		// If our projected distance from vA, "t", is greater than or equal to the magnitude
		// or distance of the line segment, it must be closest to the end point vB.  So, return vB.
		if (t >= d)
			return vB;

		// Here we create a vector that is of length t and in the direction of vVector2
		CVector3 vVector3 = vVector2 * t;

		// To find the closest point on the line segment, we just add vVector3 to the original
		// end point vA.
		CVector3 vClosestPoint = vA + vVector3;

		// Return the closest point on the line segment
		return vClosestPoint;
	}


	/////////////////////////////////// PLANE DISTANCE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the distance between a plane and the origin
	/////
	/////////////////////////////////// PLANE DISTANCE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline float PlaneDistance(CVector3 Normal, CVector3 Point)
	{
		float distance = 0;									// This variable holds the distance from the plane tot he origin

		// Use the plane equation to find the distance (Ax + By + Cz + D = 0)  We want to find D.
		// So, we come up with D = -(Ax + By + Cz)
		// Basically, the negated dot product of the normal of the plane and the point. (More about the dot product in another tutorial)
		distance = - ((Normal.x * Point.x) + (Normal.y * Point.y) + (Normal.z * Point.z));

		return distance;									// Return the distance
	}


	/////////////////////////////////// INTERSECTED PLANE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This checks to see if a line intersects a plane
	/////
	/////////////////////////////////// INTERSECTED PLANE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline bool IntersectedPlane(CVector3 vPoly[], CVector3 vLine[], CVector3 &vNormal, float &originDistance)
	{
		float distance1=0, distance2=0;						// The distances from the 2 points of the line from the plane

		vNormal = Normal(vPoly);							// We need to get the normal of our plane to go any further

		// Let's find the distance our plane is from the origin.  We can find this value
		// from the normal to the plane (polygon) and any point that lies on that plane (Any vertex)
		originDistance = PlaneDistance(vNormal, vPoly[0]);

		// Get the distance from point1 from the plane using: Ax + By + Cz + D = (The distance from the plane)

		distance1 = ((vNormal.x * vLine[0].x)  +					// Ax +
			(vNormal.y * vLine[0].y)  +					// Bx +
			(vNormal.z * vLine[0].z)) + originDistance;	// Cz + D

		// Get the distance from point2 from the plane using Ax + By + Cz + D = (The distance from the plane)

		distance2 = ((vNormal.x * vLine[1].x)  +					// Ax +
			(vNormal.y * vLine[1].y)  +					// Bx +
			(vNormal.z * vLine[1].z)) + originDistance;	// Cz + D

		// Now that we have 2 distances from the plane, if we times them together we either
		// get a positive or negative number.  If it's a negative number, that means we collided!
		// This is because the 2 points must be on either side of the plane (IE. -1 * 1 = -1).

		if(distance1 * distance2 >= 0)			// Check to see if both point's distances are both negative or both positive
			return false;						// Return false if each point has the same sign.  -1 and 1 would mean each point is on either side of the plane.  -1 -2 or 3 4 wouldn't...

		return true;							// The line intersected the plane, Return TRUE
	}


	inline float DeterminantInPlane(CVector3 vVector1, CVector3 vVector2, CVector3 vVector3)
	{

		return vVector1.x * vVector2.y * vVector3.z +	
			vVector1.z * vVector2.x * vVector3.y +
			vVector1.y * vVector2.z * vVector3.x -
			vVector1.z * vVector2.y * vVector3.x -
			vVector1.x * vVector2.z * vVector3.y -
			vVector1.y * vVector2.x * vVector3.z;

	}


	/////////////////////////////////// DOT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This computers the dot product of 2 vectors
	/////
	/////////////////////////////////// DOT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline float Dot(CVector3 vVector1, CVector3 vVector2)
	{
		// The dot product is this equation: V1.V2 = (V1.x * V2.x  +  V1.y * V2.y  +  V1.z * V2.z)
		// In math terms, it looks like this:  V1.V2 = ||V1|| ||V2|| cos(theta)

		//    (V1.x * V2.x        +        V1.y * V2.y        +        V1.z * V2.z)
		return ( (vVector1.x * vVector2.x) + (vVector1.y * vVector2.y) + (vVector1.z * vVector2.z) );
	}


	/////////////////////////////////// ANGLE BETWEEN VECTORS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This checks to see if a point is inside the ranges of a polygon
	/////
	/////////////////////////////////// ANGLE BETWEEN VECTORS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline double AngleBetweenVectors(CVector3 Vector1, CVector3 Vector2)
	{
		// Get the dot product of the vectors
		float dotProduct = Dot(Vector1, Vector2);

		// Get the product of both of the vectors magnitudes
		float vectorsMagnitude = Magnitude(Vector1) * Magnitude(Vector2) ;

		float v = dotProduct / vectorsMagnitude;
		if (v > 1.0)
			v = 1.0;
		if (v < -1.0)
			v = -1.0;

		// Get the angle in radians between the 2 vectors
		double angle = acos( v );

		// Here we make sure that the angle is not a -1.#IND0000000 number, which means indefinate
		if(_isnan(angle))
			return 0;

		// Return the angle in radians
		return( angle );
	}


	/////////////////////////////////// INTERSECTION POINT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This returns the intersection point of the line that intersects the plane
	/////
	/////////////////////////////////// INTERSECTION POINT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline CVector3 IntersectionPoint(CVector3 vNormal, CVector3 vLine[], double distance)
	{
		CVector3 vPoint, vLineDir;					// Variables to hold the point and the line's direction
		double Numerator = 0.0, Denominator = 0.0, dist = 0.0;

		// 1)  First we need to get the vector of our line, Then normalize it so it's a length of 1
		vLineDir = vLine[1] - vLine[0];		// Get the Vector of the line
		vLineDir = Normalize(vLineDir);				// Normalize the lines vector


		// 2) Use the plane equation (distance = Ax + By + Cz + D) to find the
		// distance from one of our points to the plane.
		Numerator = - (vNormal.x * vLine[0].x +		// Use the plane equation with the normal and the line
			vNormal.y * vLine[0].y +
			vNormal.z * vLine[0].z + distance);

		// 3) If we take the dot product between our line vector and the normal of the polygon,
		Denominator = Dot(vNormal, vLineDir);		// Get the dot product of the line's vector and the normal of the plane

		// Since we are using division, we need to make sure we don't get a divide by zero error
		// If we do get a 0, that means that there are INFINATE points because the the line is
		// on the plane (the normal is perpendicular to the line - (Normal.Vector = 0)).
		// In this case, we should just return any point on the line.

		if( Denominator == 0.0)						// Check so we don't divide by zero
			return vLine[0];						// Return an arbitrary point on the line

		dist = Numerator / Denominator;				// Divide to get the multiplying (percentage) factor

		// Now, like we said above, we times the dist by the vector, then add our arbitrary point.
		vPoint.x = (float)(vLine[0].x + (vLineDir.x * dist));
		vPoint.y = (float)(vLine[0].y + (vLineDir.y * dist));
		vPoint.z = (float)(vLine[0].z + (vLineDir.z * dist));

		return vPoint;								// Return the intersection point
	}



	/////////////////////////////////// INSIDE POLYGON \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This checks to see if a point is inside the ranges of a polygon
	/////
	/////////////////////////////////// INSIDE POLYGON \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline bool InsidePolygon(CVector3 vIntersection, CVector3 Poly[], long verticeCount)
	{
		const double MATCH_FACTOR = 0.99;		// Used to cover up the error in floating point
		double Angle = 0.0;						// Initialize the angle
		CVector3 vA, vB;						// Create temp vectors

		for (int i = 0; i < verticeCount; i++)		// Go in a circle to each vertex and get the angle between
		{
			vA = Poly[i] - vIntersection;			// Subtract the intersection point from the current vertex
			// Subtract the point from the next vertex
			vB = Poly[(i + 1) % verticeCount] - vIntersection;

			Angle += AngleBetweenVectors(vA, vB);	// Find the angle between the 2 vectors and add them all up as we go along
		}

		if(Angle >= (MATCH_FACTOR * (2.0 * PI)) )	// If the angle is greater than 2 PI, (360 degrees)
			return true;							// The point is inside of the polygon

		return false;								// If you get here, it obviously wasn't inside the polygon, so Return FALSE
	}


	/////////////////////////////////// INTERSECTED POLYGON \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*
	/////
	/////	This checks if a line is intersecting a polygon
	/////
	/////////////////////////////////// INTERSECTED POLYGON \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*

	inline bool IntersectedPolygon(CVector3 vPoly[], CVector3 vLine[], int verticeCount)
	{
		CVector3 vNormal;
		float originDistance = 0;

		// First, make sure our line intersects the plane
		// Reference   // Reference
		if(!IntersectedPlane(vPoly, vLine,   vNormal,   originDistance))
			return false;

		// Now that we have our normal and distance passed back from IntersectedPlane(),
		// we can use it to calculate the intersection point.
		CVector3 vIntersection = IntersectionPoint(vNormal, vLine, originDistance);

		// Now that we have the intersection point, we need to test if it's inside the polygon.
		if(InsidePolygon(vIntersection, vPoly, verticeCount))
			return true;							// We collided!	  Return success

		return false;								// There was no collision, so return false
	}

	inline double cotan(double i) {
		return(1 / tan(i));
	}

	/////// * /////////// * /////////// * NEW * /////// * /////////// * /////////// *

	inline float QuaternionNorm(Quaternion q) {
		return q.i*q.i + q.j*q.j + q.k*q.k + q.s*q.s;
	}

	inline Quaternion QuaternionNormalize(Quaternion q) {
		float len = QuaternionNorm(q);
		float factor = 1.0 / sqrtf(len);
		return q*factor;
	}

	inline Quaternion QuaternionOpposite(Quaternion q) {
		return Quaternion(q.s, -q.i, -q.j, -q.k);
	}

	inline float Length(CVector3 v) {
		return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
	}

	inline bool IsZeroLength(CVector3 v) {
		float sqlen = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
		return (sqlen < (1e-06 * 1e-06));
	}

	inline Quaternion QuaternionFromAngleAxis(float angle, CVector3 axis) {
		// assert:  axis[] is unit length
		//
		// The quaternion representing the rotation is
		//   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)
		Quaternion q;

		float halfAngle = 0.5*angle;
		float fSin = sin(halfAngle);
		q.s = cos(halfAngle);
		q.i = fSin*axis.x;
		q.j = fSin*axis.y;
		q.k = fSin*axis.z;

		return q;
	}

	inline Quaternion QuaternionBetweenVectorsWithAxis(CVector3 start, CVector3 dest, CVector3 axis) {
		// Copy, since cannot modify local
		CVector3 v0 = start;
		v0 = Normalize(v0 - axis*Dot(v0, axis));
		CVector3 v1 = dest;
		v1 = Normalize(v1 - axis*Dot(v1, axis));

		float d = Dot(v0, v1);
		// If dot == 1, vectors are the same
		if (d >= 1.0f) {
			return Quaternion(0,1,0,0);
		}
		if (d < (1e-6f - 1.0f)) {
			return QuaternionFromAngleAxis(M_PI, axis);
		}

		float alfa = acosf(d);
		return QuaternionFromAngleAxis(alfa, axis);
	}

	inline Quaternion QuaternionBetweenVectors(CVector3 start, CVector3 dest, CVector3 fallbackAxis) {
		// Based on Stan Melax's article in Game Programming Gems
		Quaternion q;
		// Copy, since cannot modify local
		CVector3 v0 = Normalize(start);
		CVector3 v1 = Normalize(dest);

		float d = Dot(v0, v1);
		// If dot == 1, vectors are the same
		if (d >= 1.0f) {
			return q;
		}
		if (d < (1e-6f - 1.0f)) {
			if (fallbackAxis != CVector3()) {
				// rotate 180 degrees about the fallback axis
				q = QuaternionFromAngleAxis(M_PI, fallbackAxis);
			} else {
				// Generate an axis
				CVector3 axis = Cross(CVector3(1, 0, 0), v0);
				if (IsZeroLength(axis)) { // pick another if colinear
					axis = Cross(CVector3(0, 1, 0), v0);
				}
				axis = Normalize(axis);
				q = QuaternionFromAngleAxis(M_PI, axis);
			}
		} else {//alpha from dot product, axis from cross product
			float s = sqrtf((1+d)*2);
			float invs = 1 / s;

			CVector3 c = Cross(v0, v1);

			q.i = c.x * invs;
			q.j = c.y * invs;
			q.k = c.z * invs;
			q.s = s * 0.5f;
			QuaternionNormalize(q);
		}

		return q;
	}

	inline CVector3 QuaternionRotateVector(Quaternion q, CVector3 v) {
		// nVidia SDK implementation
		//CVector3 uv, uuv;
		CVector3 qvec(q.i, q.j, q.k);
		CVector3 vv = v;
		CVector3 uv = Cross(qvec, v);
		CVector3 uuv = Cross(qvec, uv);
		uv = uv * (2.0f * q.s);
		uuv = uuv * 2.0f;

		return (v + uv + uuv);
	}

	inline CVector3 PlaneRayIntersection(CVector3 rayOrigin, CVector3 direction, CVector3 planeOrigin, CVector3 normal) {
		float t = Dot(normal, planeOrigin - rayOrigin)/Dot(normal, direction);
		return rayOrigin + (direction * t);
	}

	inline bool equal(float x, float y) {
		return fabs(x - y) < FLOAT_ZERO;
	}
}
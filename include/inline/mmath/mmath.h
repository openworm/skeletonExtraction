#pragma once

#ifndef m_math
#define m_math

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>

#pragma warning(push, 0)
#include <boost\serialization\serialization.hpp>
#include <boost\serialization\variant.hpp>
#include <boost\serialization\vector.hpp>
#include <boost\serialization\nvp.hpp>
#include <boost\serialization\assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>
#pragma pop

#include <tnt\tnt.h>

using namespace TNT;

#define PI 3.1415926535897932					// This is our famous PI
#define BEHIND		0
#define INTERSECTS	1
#define INFRONT		2

#define FLOAT_ZERO 0.00001
#define EPSILON 0.00001

namespace mmath {
	struct CVector2
	{
	public:

		// A default constructor
		CVector2() {}

		// This is our constructor that allows us to initialize our data upon creating an instance
		CVector2(float X, float Y)
		{
			x = X; y = Y;
		}

		// Here we overload the * operator so we can multiply by scalars
		CVector2 operator*(float num)
		{
			// Return the scaled vector
			return CVector2(x * num, y * num);
		}
		CVector2 operator/(float num)
		{
			// Return the scale vector
			return CVector2(x / num, y / num);
		}
		// Here we overload the + operator so we can add vectors together
		CVector2 operator+(CVector2 vVector)
		{
			// Return the added vectors result.
			return CVector2(vVector.x + x, vVector.y + y);
		}
		CVector2 operator-(CVector2 vVector)
		{
			// Return the added vectors result.
			return CVector2(vVector.x - x, vVector.y - y);
		}

		float x, y;
	};

	// This is our basic 3D point/vector class
	struct CVector3
	{
	public:

		// A default constructor
		CVector3() : x(0), y(0), z(0) {}

		// This is our constructor that allows us to initialize our data upon creating an instance
		CVector3(float X, float Y, float Z)
		{
			x = X; y = Y; z = Z;
		}

		CVector3(float *v) {
			x = v[0]; y = v[1]; z = v[2];
		}

		// Here we overload the = operator so we can compare vectors
		bool operator==(CVector3 vVector)
		{
			// Return the added vectors result.
			return (x == vVector.x) && (y == vVector.y) && (z == vVector.z);
		}

		// Here we overload the != operator so we can compare vectors
		bool operator!=(CVector3 vVector)
		{
			// Return the added vectors result.
			return (x != vVector.x) || (y != vVector.y) || (z != vVector.z);
		}

		// Here we overload the + operator so we can add vectors together
		CVector3 operator+(CVector3 vVector)
		{
			// Return the added vectors result.
			return CVector3(vVector.x + x, vVector.y + y, vVector.z + z);
		}

		// Here we overload the - operator so we can subtract vectors
		CVector3 operator-(CVector3 vVector)
		{
			// Return the subtracted vectors result
			return CVector3(x - vVector.x, y - vVector.y, z - vVector.z);
		}

		// Here we overload the * operator so we can multiply by scalars
		CVector3 operator*(float num)
		{
			// Return the scaled vector
			return CVector3(x * num, y * num, z * num);
		}

		// Here we overload the / operator so we can divide by a scalar
		CVector3 operator/(float num)
		{
			// Return the scale vector
			return CVector3(x / num, y / num, z / num);
		}
		float x, y, z;
	private:

		friend class boost::serialization::access;
		// When the class Archive corresponds to an output archive, the
		// & operator is defined similar to <<.  Likewise, when the class Archive
		// is a type of input archive the & operator is defined similar to >>.
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(x);
			ar & BOOST_SERIALIZATION_NVP(y);
			ar & BOOST_SERIALIZATION_NVP(z);
		}

	};


	// Quaternion structure
	struct CVector4
	{
	public:

		// A default constructor
		CVector4() : s(1), i(0), j(0), k(0) {}

		// This is our constructor that allows us to initialize our data upon creating an instance
		CVector4(float S, float I, float J, float K)
		{
			s = S; i = I; j = J; k = K;
		}

		CVector4(float alfa, CVector3 axis) : s(alfa), i(axis.x), j(axis.y), k(axis.z) { }

		// Here we overload the + operator so we can add vectors together
		CVector4 operator+(CVector4 vVector)
		{
			// Return the added vectors result.
			return CVector4(vVector.s + s, vVector.i + i, vVector.j + j, vVector.k + k);
		}

		// Here we overload the - operator so we can subtract vectors
		CVector4 operator-(CVector4 vVector)
		{
			// Return the subtracted vectors result
			return CVector4(s - vVector.s, i - vVector.i, j - vVector.j, k - vVector.k);
		}

		// Here we overload the * operator so we can multiply by scalars
		CVector4 operator*(float num)
		{
			// Return the scaled vector
			return CVector4(s * num, i * num, j * num, k * num);
		}

		CVector4 operator* (CVector4 rkQ)
		{
			// NOTE:  Multiplication is not generally commutative, so in most
			// cases p*q != q*p.

			return CVector4
				(
				s * rkQ.s - i * rkQ.i - j * rkQ.j - k * rkQ.k,
				s * rkQ.i + i * rkQ.s + j * rkQ.k - k * rkQ.j,
				s * rkQ.j + j * rkQ.s + k * rkQ.i - i * rkQ.k,
				s * rkQ.k + k * rkQ.s + i * rkQ.j - j * rkQ.i
				);
		}

		// Here we overload the / operator so we can divide by a scalar
		CVector4 operator/(float num)
		{
			// Return the scale vector
			return CVector4(s / num, i / num, j / num, k / num);
		}

		float s, i, j, k;
	private:

		friend class boost::serialization::access;
		// When the class Archive corresponds to an output archive, the
		// & operator is defined similar to <<.  Likewise, when the class Archive
		// is a type of input archive the & operator is defined similar to >>.
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(s);
			ar & BOOST_SERIALIZATION_NVP(i);
			ar & BOOST_SERIALIZATION_NVP(j);
			ar & BOOST_SERIALIZATION_NVP(k);
		}

	};

	typedef CVector4 Quaternion;

	//---------------------------------------------------------------------------

	float * TntMatrix4x4ToFloatPointer(Array2D<float> m);

	CVector4 lerp(CVector4 v0, CVector4 v1, float t);

	CVector4 slerp(CVector4 v0, CVector4 v1, double t);

	float Dot(CVector4 vVector1, CVector4 vVector2);

	CVector4 QuaternionProduct(CVector4 vVector1, CVector4 vVector2);

	CVector4 Normalize(CVector4 q);

	CVector4 QuaternionToAxisAngle(CVector4 q);

	Array2D<float> Rot3x3ToRot4x4(Array2D<float> in);

	Array2D< float > QuaternionToMatrix3x3(CVector4 q);

	Array2D<float> QuaternionToTntVec (CVector4 q);

	CVector4 TntVecToQuaternion(Array2D<float> vec);

	CVector4 Matrix3x3ToQuaternion(Array2D<float> m);

	Array2D<float> CVecToTntVec (CVector3 vec);

	Array2D<float> CVecToTntVec4 (CVector3 vec);

	CVector4 AxisRotToQuaternion(float angle, CVector3 axis);

	CVector3 TntVecToCVec (Array2D<float> vec);


	// This returns the absolute value of "num"
	float Absolute(float num);

	//	This returns a perpendicular vector from 2 given vectors by taking the cross product.
	CVector3 Cross(CVector3 vVector1, CVector3 vVector2);

	//	This returns the magnitude of a normal (or any other vector)
	float Magnitude(CVector3 vNormal);

	//	This returns a normalize vector (A vector exactly of length 1)
	CVector3 Normalize(CVector3 vNormal);

	//	This returns the normal of a polygon (The direction the polygon is facing)
	CVector3 Normal(CVector3 vPolygon[]);

	// This returns the distance between 2 3D points
	float Distance(CVector3 vPoint1, CVector3 vPoint2);

	float DistancePointToLine(CVector3 vA, CVector3 vB, CVector3 vPoint);

	// This returns the point on the line segment vA_vB that is closest to point vPoint
	CVector3 ClosestPointOnLine(CVector3 vA, CVector3 vB, CVector3 vPoint);

	// This returns the distance the plane is from the origin (0, 0, 0)
	// It takes the normal to the plane, along with ANY point that lies on the plane (any corner)
	float PlaneDistance(CVector3 Normal, CVector3 Point);

	// This takes a triangle (plane) and line and returns true if they intersected
	bool IntersectedPlane(CVector3 vPoly[], CVector3 vLine[], CVector3 &vNormal, float &originDistance);

	// This returns the dot product between 2 vectors
	float Dot(CVector3 vVector1, CVector3 vVector2);

	float DeterminantInPlane(CVector3 vVector1, CVector3 vVector2, CVector3 vVector3);

	// This returns the angle between 2 vectors
	double AngleBetweenVectors(CVector3 Vector1, CVector3 Vector2);

	// This returns an intersection point of a polygon and a line (assuming intersects the plane)
	CVector3 IntersectionPoint(CVector3 vNormal, CVector3 vLine[], double distance);

	// This returns true if the intersection point is inside of the polygon
	bool InsidePolygon(CVector3 vIntersection, CVector3 Poly[], long verticeCount);

	// Use this function to test collision between a line and polygon
	bool IntersectedPolygon(CVector3 vPoly[], CVector3 vLine[], int verticeCount);

	double cotan(double i);
	//---------------------------------------------------------------------------

	float QuaternionNorm(Quaternion q);

	Quaternion QuaternionNormalize(Quaternion q);

	Quaternion QuaternionOpposite(Quaternion q);

	float Length(CVector3 v);

	bool IsZeroLength(CVector3 v);

	Quaternion QuaternionFromAngleAxis(float angle, CVector3 axis);

	Quaternion QuaternionBetweenVectorsWithAxis(CVector3 start, CVector3 dest, CVector3 axis);

	Quaternion QuaternionBetweenVectors(CVector3 start, CVector3 dest, CVector3 fallbackAxis = CVector3());

	CVector3 QuaternionRotateVector(Quaternion q, CVector3 v);

	CVector3 PlaneRayIntersection(CVector3 rayOrigin, CVector3 direction, CVector3 planeOrigin, CVector3 normal);

	bool equal(float x, float y);
}

#include "mmath.inl"

#endif

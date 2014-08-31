// ******************************
// triangle.cpp
//
// Triangle class.
// Contains vetices & normal to
// the place defined by the 3
// vertices.
//
// Jeff Somers
// Copyright (c) 2002
//
// jsomers@alumni.williams.edu
// March 27, 2002
// ******************************

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma warning(disable:4710) // function not inlined
#pragma warning(disable:4702) // unreachable code
#pragma warning(disable:4514) // unreferenced inline function has been removed
#endif

#include <assert.h>
#include "triangle.h"
#include "mesh.h"

// retrieve vertices as an array of floats
const float* jmsp::triangle::getVert1() {return (_mesh->getVertex(_vert1)).getArrayVerts();}
const float* jmsp::triangle::getVert2() {return (_mesh->getVertex(_vert2)).getArrayVerts();}
const float* jmsp::triangle::getVert3() {return (_mesh->getVertex(_vert3)).getArrayVerts();}

// retrieve vertices as a vertex object
const jmsp::vertex& jmsp::triangle::getVert1vertex() const {return _mesh->getVertex(_vert1);};
const jmsp::vertex& jmsp::triangle::getVert2vertex() const {return _mesh->getVertex(_vert2);};
const jmsp::vertex& jmsp::triangle::getVert3vertex() const {return _mesh->getVertex(_vert3);};

// Calculate normal of triangle
void
jmsp::triangle::calcNormal()
{
	assert(_mesh);

	Vec3 vec1 = (_mesh->getVertex(_vert1)).getXYZ();
	Vec3 vec2 = (_mesh->getVertex(_vert2)).getXYZ();
	Vec3 vec3 = (_mesh->getVertex(_vert3)).getXYZ();
	Vec3 veca = vec2 - vec1;
	Vec3 vecb = vec3 - vec2;

	_normal = veca.unitcross(vecb);
	// Note that if the triangle is degenerate (all vertices lie in a line),
	// the normal will be <0,0,0>

	// This is the "d" from the plane equation ax + by + cz + d = 0;

	_d = -_normal.dot(vec1);
}

// Calculate area of triangle
float
jmsp::triangle::calcArea()
{
	assert(_mesh);

	// If a triangle is defined by 3 points, say p, q and r, then
	// its area is 0.5 * length of ((p - r) cross (q - r))
	// See Real-Time Rendering book, Appendix A
	Vec3 vec1 = (_mesh->getVertex(_vert1)).getXYZ();
	Vec3 vec2 = (_mesh->getVertex(_vert2)).getXYZ();
	Vec3 vec3 = (_mesh->getVertex(_vert3)).getXYZ();
	Vec3 vecA = vec1 - vec2;
	Vec3 vecB = vec3 - vec2;

	Vec3 cross = vecA.cross(vecB);
	float area = float(0.5 * cross.length());
	return area;
}


// Used for output
/*std::ostream&
jmsp::operator<<(std::ostream& os, const jmsp::triangle& to)
{
	os << "vert1: " << to._vert1 << " vert2: " << to._vert2 << " vert3: " << to._vert3; // for some reason this isn't working as a friend function, not sure why
	os << " Normal: " << to._normal << " Active? " << to.isActive();
	os << " Index: " << to._index;

	// it is pulling ostream from the STL typedef, not the regular ostream, though.
	return os;
}*/


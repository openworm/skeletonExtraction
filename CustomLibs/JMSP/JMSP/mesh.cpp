// ******************************
// mesh.cpp
//
// Mesh class, which stores a list
// of vertices & a list of triangles.
//
// Jeff Somers
// Copyright (c) 2002
//
// jsomers@alumni.williams.edu
// March 27, 2002
// ******************************

#include <assert.h>
#include <float.h>

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma warning(disable:4710) // function not inlined
#pragma warning(disable:4702) // unreachable code
#pragma warning(disable:4514) // unreferenced inline function has been removed
#endif

#include <iostream>

#include "mesh.h"

using namespace jmsp;


Mesh::Mesh(char* filename)
{
	_numVerts = _numTriangles = 0;
	if (!loadFromFile(filename))
	{
		// we failed to load mesh from the file
		_numVerts = _numTriangles = 0;
		_vlist.clear();
		_plist.clear();
	}
}

Mesh::Mesh(const Mesh& m)
{
	_numVerts = m._numVerts;
	_numTriangles = m._numTriangles;
	_vlist = m._vlist; // NOTE: triangles are still pointing to original mesh
	_plist = m._plist;
	// NOTE: should reset tris in _vlist, _plist
}

Mesh& Mesh::operator=(const Mesh& m)
{
	if (this == &m) return *this; // don't assign to self
	_numVerts = m._numVerts;
	_numTriangles = m._numTriangles;
	_vlist = m._vlist; // NOTE: triangles are still pointing to original mesh
	_plist = m._plist;
	// NOTE: should reset tris in _vlist, _plist
	return *this;
}

Mesh::~Mesh()
{
	_numVerts = _numTriangles = 0;
	_vlist.erase(_vlist.begin(), _vlist.end());
	_plist.erase(_plist.begin(), _plist.end());
}

// Helper function for reading PLY mesh file
bool Mesh::readNumPlyVerts(FILE *&inFile, int& nVerts)
{
	// Read # of verts
	bool bElementFound = false;
	/* Get number of vertices in mesh*/
	for(;;)
	{
		char tempStr[1024];
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File and string \"element vertex\" not found!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);

		if (bElementFound && !strncmp(tempStr, "vertex", 6))
		{
			break;
		}

		if (!strncmp(tempStr, "element", 7))
		{
			bElementFound = true;
			continue;
		}
	}

	fscanf(inFile, "%d", &nVerts); 
	if (feof(inFile))
	{
		MessageBox(NULL, "Reached End of File before \"element face\" found!\n",
			NULL, MB_ICONEXCLAMATION);
		return false;
	}
	return true;
}

// Helper function for reading PLY mesh file
bool Mesh::readNumPlyTris(FILE *&inFile, int& nTris)
{
	bool bElementFound = false;
	/* Get number of faces in mesh*/
	for(;;)
	{
		char tempStr[1024];
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File and string \"element face\" not found!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);

		if (bElementFound && !strncmp(tempStr, "face", 4))
		{
			break;
		}

		if (!strncmp(tempStr, "element", 7))
		{
			bElementFound = true;
			continue;
		}
	}

	fscanf(inFile, "%d", &nTris);
	if (feof(inFile))
	{
		MessageBox(NULL, "Reached End of File before list of vertices found!\n",
			NULL, MB_ICONEXCLAMATION);
		return false;
	}
	return true;
}

// Helper function for reading PLY mesh file
bool Mesh::readPlyHeader(FILE *&inFile)
{
	char tempStr[1024];

	// Read "ply" string
	do
	{
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File and the string \"ply\" NOT FOUND!!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}
		ChangeStrToLower(tempStr); // change tempStr to lower case 
	} while (strncmp(tempStr, "ply", 3));

	// Read # of verts
	if (!readNumPlyVerts(inFile, _numVerts))
	{
		return false;
	}

	// Read # of triangles
	if (!readNumPlyTris(inFile, _numTriangles))
	{
		return false;
	}

	// get end_header
	do
	{
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File and string \"end_header\" not found!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);
	} while (strncmp(tempStr, "end_header", 10));

	////////// end of header
	return true;
}

// Helper function for reading PLY mesh file
bool Mesh::readPlyVerts(FILE *&inFile)
{
	int i;
	// read vertices
	for (i = 0; i < _numVerts; i++)
	{
		char tempStr[1024];

#pragma warning(disable:4244)		/* disable double -> float warning */
		fscanf(inFile, "%s", tempStr);
		float x = atof(tempStr); 
		fscanf(inFile, "%s", tempStr);
		float y = atof(tempStr); 
		fscanf(inFile, "%s", tempStr);
		float z = atof(tempStr); 
#pragma warning(default:4244)		/* double -> float */

		vertex v(x, y, z);
		v.setIndex(i);

		_vlist.push_back(v); // push_back puts a *copy* of the element at the end of the list
		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File before all vertices found!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}

		// read until end of line
		while (fgetc(inFile) != '\n');
	}
	return true;
}

// Helper function for reading PLY mesh file
bool Mesh::readPlyTris(FILE *&inFile)
{
	int i;
	// read triangles
	for (i = 0; i < _numTriangles; i++)
	{
		int v1, v2, v3;
		int nVerts;
		fscanf(inFile, "%d", &nVerts);
		if (3 != nVerts)
		{
			MessageBox(NULL, "Error:  Ply file contains polygons which are not triangles!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}
		fscanf(inFile, "%d", &v1);   // get value for vertex A
		fscanf(inFile, "%d", &v2);   // get value for vertex B
		fscanf(inFile, "%d", &v3);   // get value for vertex C

		// make sure verts in correct range
		assert(v1 < _numVerts && v2 < _numVerts && v3 < _numVerts);

		triangle t(this, v1, v2, v3);
		t.setIndex(i);

		_plist.push_back(t); // push_back puts a *copy* of the element at the end of the list

		// update each vertex w/ its neighbors (vertrices & triangles)
		_vlist[v1].addTriNeighbor(i);
		_vlist[v1].addVertNeighbor(v2);
		_vlist[v1].addVertNeighbor(v3);

		_vlist[v2].addTriNeighbor(i);
		_vlist[v2].addVertNeighbor(v1);
		_vlist[v2].addVertNeighbor(v3);

		_vlist[v3].addTriNeighbor(i);
		_vlist[v3].addVertNeighbor(v1);
		_vlist[v3].addVertNeighbor(v2);

		if (feof(inFile))
		{
			MessageBox(NULL, "Reached End of File before all faces found!\n",
				NULL, MB_ICONEXCLAMATION);
			return false;
		}
		// read until end of line
		while (fgetc(inFile) != '\n');
	}
	return true;
}


// Load mesh from PLY file
bool Mesh::loadFromFile(char* filename)
{
    FILE* inFile = fopen(filename, "rt");
    if (inFile == NULL)
    {
        char pszError[_MAX_FNAME + 1];
		sprintf(pszError, "%s does not exist!\n", filename);
        MessageBox(NULL, pszError, NULL, MB_ICONEXCLAMATION);
		return FALSE;
    }

	// read header to PLY file
	if (!readPlyHeader(inFile))
	{
		return false;
	}

	// read vertex data from PLY file
	if (!readPlyVerts(inFile))
	{
		return false;
	}

	// read triangle data from PLY file
	if (!readPlyTris(inFile))
	{
		return false;
	}

    fclose(inFile); // close the file

	calcVertNormals();

	return true;
}


// Recalculate the normal for one vertex
void Mesh::calcOneVertNormal(unsigned vert)
{
	vertex& v = getVertex(vert);
	const set<int>& triset = v.getTriNeighbors();

	set<int>::iterator iter;

	Vec3 vec;

	for (iter = triset.begin(); iter != triset.end(); ++iter)
	{
		// get the triangles for each vertex & add up the normals.
		vec += getTri(*iter).getNormalVec3();
	}

	vec.normalize(); // normalize the vertex	
	v.setVertNomal(vec);
}


// Calculate the vertex normals after loading the mesh.
void Mesh::calcVertNormals()
{
	// Iterate through the vertices
	for (unsigned i = 0; i < _vlist.size(); ++i)
	{
		calcOneVertNormal(i);
	}
}


// Used for debugging
void Mesh::dump()
{
	/*std::cout << "*** Mesh Dump ***" << std::endl;
	std::cout << "# of vertices: " << _numVerts << std::endl;
	std::cout << "# of triangles: " << _numTriangles << std::endl;
	for (unsigned i = 0; i < _vlist.size(); ++i)
	{
		std::cout << "\tVertex " << i << ": " << _vlist[i] << std::endl;
	}
	std::cout << std::endl;
	for (i = 0; i < _plist.size(); ++i)
	{
		std::cout << "\tTriangle " << i << ": " << _plist[i] << std::endl;
	}
	std::cout << "*** End of Mesh Dump ***" << std::endl;
	std::cout << std::endl;*/
}

// Get min, max values of all verts
void Mesh::setMinMax(float min[3], float max[3])
{
	max[0] = max[1] = max[2] = -FLT_MAX;
	min[0] = min[1] = min[2] = FLT_MAX;

	for (unsigned int i = 0; i < _vlist.size(); ++i)
	{
		const float* pVert = _vlist[i].getArrayVerts();
		if (pVert[0] < min[0]) min[0] = pVert[0];
		if (pVert[1] < min[1]) min[1] = pVert[1];
		if (pVert[2] < min[2]) min[2] = pVert[2];
		if (pVert[0] > max[0]) max[0] = pVert[0];
		if (pVert[1] > max[1]) max[1] = pVert[1];
		if (pVert[2] > max[2]) max[2] = pVert[2];
	}
}

// Center mesh around origin.
// Fit mesh in box from (-1, -1, -1) to (1, 1, 1)
void Mesh::Normalize()  
{
	float min[3], max[3], Scale;

	setMinMax(min, max);

	Vec3 minv(min);
	Vec3 maxv(max);

	Vec3 dimv = maxv - minv;
	
	if (dimv.x >= dimv.y && dimv.x >= dimv.z) Scale = 2.0f/dimv.x;
	else if (dimv.y >= dimv.x && dimv.y >= dimv.z) Scale = 2.0f/dimv.y;
	else Scale = 2.0f/dimv.z;

	Vec3 transv = minv + maxv;

	transv *= 0.5f;

	for (unsigned int i = 0; i < _vlist.size(); ++i)
	{
		_vlist[i].getXYZ() -= transv;
		_vlist[i].getXYZ() *= Scale;
	}
}


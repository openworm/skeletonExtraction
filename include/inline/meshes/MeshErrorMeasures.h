#pragma once
#include <meshes/IndexedFace.h>

#ifndef MESH_ERROR_MEASURES
#define MESH_ERROR_MEASURES

namespace MEM {

	float MeanSquareError(meshes::IndexedFace *meshA, meshes::IndexedFace *meshB);
	//mmath::CVector3 AABBBottomCenter(meshes::IndexedFace *mesh);
	void ShiftMeshByPoint(meshes::IndexedFace *mesh, const mmath::CVector3 &P);
}

#include "MeshErrorMeasures.inl"

#endif
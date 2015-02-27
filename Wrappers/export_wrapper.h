#pragma once

#include <meshes/Mesh.h>
#include <meshes/SerializableMesh+Skin.h>
#include <meshes/MeshGraph.h>

#ifndef _OW

	#include <SM_lib/MatchingStruct.h>
	#include <STM_lib/stm_skeleton.h>

#endif

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>

#include <Export_lib/ColladaExporter.h>

#ifndef _OW

	#include "sm_wrapper.h"

#endif

#include <inline/SkeletonNode/s_skeletonNode.h>
#include <GDSW_lib/gdsw_graphAlgorithms.h>

struct Export_Wrapper {
	Export_Wrapper();
	Export::ColladaExporter exporter;

	ObjectSkeletonShaderData wormSkinningData;

	bool bfNormalRepair;

	#ifndef _OW

		bool serializeSkeletonMatching(char * file, SkeletonMatching * pSkeletonMatching);
		bool serializeSkeletonNode(char * file, skl::SkeletonNode * pSkeletonRoot);
		bool serializeSkeletonBoundingMesh(char * file, SkeletonBoundingMesh * pSkeletonBoundingMesh);
		bool serializeSurgeryGraph(char * file, SurgeryGraph * pSurgeryGraph, CVector3 * pVerts);
		bool serializeMeshGraph(char * file, MeshGraph * mg, vector<int>& skeletonMeshGraphIndices, boost::unordered_map<int, vector<int> >& mgDegeneratesMapping);
		bool serializeDistanceMatrix(char * file, Array2D<float>& distanceMatrix);

	#endif

	void addSkinningDataToMeshForExport(meshes::MeshSkin *meshSkin);
	void exportWormMeshToCollada(meshes::IndexedFace *mesh, std::string file);
	void exportWormMeshAndSkeletonToCollada(meshes::IndexedFace *mesh, SN::SkeletonNode * pSkeleton, std::string file);
	void exportWormMeshAndSkinningDataToCollada(meshes::IndexedFace *mesh, SN::SkeletonNode * pSkeleton, std::string file);
	void exportWormMeshAndSkinningDataToXML(meshes::IndexedFace *mesh, std::string file);

	std::string PadNumber(int number, int places);
	void ExportMeshToOFFFile(meshes::IndexedFace *mesh, std::string fileName);
};

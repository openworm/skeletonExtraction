#include "export_wrapper.h"

namespace {
	SN::SkeletonNode* ToSNSkeletonNode(SN::SkeletonNode *skeleton, SN::SkeletonNode *parent, std::map<int, SN::SkeletonNode*> &map) {
		SN::SkeletonNode *node = new SN::SkeletonNode(skeleton->point, skeleton->id, parent);
		map[node->id] = node;
		node->cyclic = skeleton->cyclic;
		for (int i = 0; i < skeleton->nodes.size(); i++) {
			node->nodes.push_back(ToSNSkeletonNode(skeleton->nodes[i], node, map));
		}
		return node;
	}
	SN::SkeletonNode* ToSNSkeletonNode(SN::SkeletonNode *skeleton) {
		if (skeleton == NULL) return NULL;
		std::map<int, SN::SkeletonNode*> map;
		SN::SkeletonNode *root = ToSNSkeletonNode(skeleton, NULL, map);
		//fix cyclic
		std::queue<SN::SkeletonNode*> queue;
		queue.push(root);
		while (!queue.empty()) {
			SN::SkeletonNode *node = queue.front();
			queue.pop();
			if (node->cyclic) node->cyclic = map[node->cyclic->id];
			for (int i = 0; i < node->nodes.size(); i++) {
				queue.push(node->nodes[i]);
			}
		}

		return root;
	}
}

Export_Wrapper::Export_Wrapper(){
	bfNormalRepair = false;
}

#ifndef _OW

bool Export_Wrapper::serializeSkeletonNode(char * file, skl::SkeletonNode * pSkeletonRoot){
	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);

	//SN::SkeletonNode * skeletonNodeToSave = (SN::SkeletonNode*)pSkeletonRoot;
	SN::SkeletonNode * skeletonNodeToSave = ToSNSkeletonNode(pSkeletonRoot);

	oa << BOOST_SERIALIZATION_NVP(skeletonNodeToSave);

	delete skeletonNodeToSave;

	return true;
}
//---------------------------------------------------------------------------
bool Export_Wrapper::serializeSkeletonMatching(char * file, SkeletonMatching * pSkeletonMatching){
	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);

	oa << BOOST_SERIALIZATION_NVP(pSkeletonMatching);

	return true;
}
//---------------------------------------------------------------------------
bool Export_Wrapper::serializeSkeletonBoundingMesh(char * file, SkeletonBoundingMesh * pSkeletonBoundingMesh){
	//make an archive
	ofstream ofs(file);
	assert(ofs.good());
	boost::archive::xml_oarchive oa(ofs);

	oa  << BOOST_SERIALIZATION_NVP(pSkeletonBoundingMesh);

	return true;
}
//---------------------------------------------------------------------------
bool Export_Wrapper::serializeMeshGraph(char * file, MeshGraph * mg, vector<int>& skeletonMeshGraphIndices, boost::unordered_map<int, vector<int> >& mgDegeneratesMapping){
	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);

	vector<float> v_verts;
	//vector<float> v_nonContractedVerts;
	vector<float> v_wH;
	vector<float> v_wHorig;
	vector<float> v_origOneRingArea;
	vector<float> v_origOneRingExtent;
	vector<bool> v_E;

	vector<int> v_indices_keys;
	vector<int> v_indices_values;
	vector<int> v_triangleIndices;

	for (int i=0; i<mg->numOfVertices; i++){
		v_verts.push_back(mg->pVerts[i].x);
		v_verts.push_back(mg->pVerts[i].y);
		v_verts.push_back(mg->pVerts[i].z);

		//v_nonContractedVerts.push_back(modelVertices[i].x);
		//v_nonContractedVerts.push_back(modelVertices[i].y);
		//v_nonContractedVerts.push_back(modelVertices[i].z);

		v_origOneRingArea.push_back(mg->origOneRingArea[i]);
		v_origOneRingExtent.push_back(mg->origOneRingExtent[i]);
		v_wH.push_back(mg->wH[i]);
		v_wHorig.push_back(mg->wHorig[i]);
		for (int j=0; j<mg->numOfVertices; j++)
			v_E.push_back(mg->E[i][j]);
	}

	for (int i=0; i<mg->numOfFaces; i++){
		v_triangleIndices.push_back(mg->triangleIndices[i * 3]);
		v_triangleIndices.push_back(mg->triangleIndices[i * 3 + 1]);
		v_triangleIndices.push_back(mg->triangleIndices[i * 3 + 2]);
	}

	for(boost::unordered_map<int, int>::iterator it = mg->indices.begin();it != mg->indices.end();it++){
		v_indices_keys.push_back(it->first);
		v_indices_values.push_back(it->second);
	}

	int numOfVertices = mg->numOfVertices;
	int numOfFaces = mg->numOfFaces;
	float faceAreaSum = mg->faceAreaSum;
	float wL = mg->wL;

	oa << BOOST_SERIALIZATION_NVP(skeletonMeshGraphIndices);

	vector<int> v_mgDegeneratesMapping_keys;
	vector< vector<int>> v_mgDegeneratesMapping_values;

	for(boost::unordered_map<int, vector<int>>::iterator it = mgDegeneratesMapping.begin();it != mgDegeneratesMapping.end();it++){
		v_mgDegeneratesMapping_keys.push_back(it->first);
		v_mgDegeneratesMapping_values.push_back(it->second);
	}

	oa << BOOST_SERIALIZATION_NVP(v_mgDegeneratesMapping_keys);
	oa << BOOST_SERIALIZATION_NVP(v_mgDegeneratesMapping_values);

	oa  << BOOST_SERIALIZATION_NVP(numOfVertices);
	oa  << BOOST_SERIALIZATION_NVP(numOfFaces);
	oa  << BOOST_SERIALIZATION_NVP(v_triangleIndices);
	oa  << BOOST_SERIALIZATION_NVP(faceAreaSum);
	oa  << BOOST_SERIALIZATION_NVP(v_indices_keys);
	oa  << BOOST_SERIALIZATION_NVP(v_indices_values);
	oa  << BOOST_SERIALIZATION_NVP(wL);
	oa  << BOOST_SERIALIZATION_NVP(v_E);
	oa  << BOOST_SERIALIZATION_NVP(v_verts);
	//oa  << BOOST_SERIALIZATION_NVP(v_nonContractedVerts);
	oa  << BOOST_SERIALIZATION_NVP(v_origOneRingArea);
	oa  << BOOST_SERIALIZATION_NVP(v_origOneRingExtent);
	oa  << BOOST_SERIALIZATION_NVP(v_wH);
	oa  << BOOST_SERIALIZATION_NVP(v_wHorig);
	oa  << BOOST_SERIALIZATION_NVP(skeletonMeshGraphIndices);

	return true;
}
//---------------------------------------------------------------------------
bool Export_Wrapper::serializeSurgeryGraph(char * file, SurgeryGraph * pSurgeryGraph, CVector3 * pVerts){

	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);

	vector<float> v_verts;

	vector<int> v_collapsed_keys;
	vector<int> v_collapsed_values;

	vector<int> v_segmentIndex_keys;
	vector<int> v_segmentIndex_values;

	vector<int> v_pointClouds_keys;
	vector< vector<int>> v_pointClouds_values;

	for (int i=0; i<pSurgeryGraph->numOfVertices; i++){
		v_verts.push_back(pVerts[i].x);
		v_verts.push_back(pVerts[i].y);
		v_verts.push_back(pVerts[i].z);
	}

	for(boost::unordered_map<int, int>::iterator it = pSurgeryGraph->collapsed.begin();it != pSurgeryGraph->collapsed.end();it++){
		v_collapsed_keys.push_back(it->first);
		v_collapsed_values.push_back(it->second);
	}

	for(boost::unordered_map<int, int>::iterator it = pSurgeryGraph->segmentIndex.begin();it != pSurgeryGraph->segmentIndex.end();it++){
		v_segmentIndex_keys.push_back(it->first);
		v_segmentIndex_values.push_back(it->second);
	}

	for(boost::unordered_map<int, vector<int>>::iterator it = pSurgeryGraph->pointClouds.begin();it != pSurgeryGraph->pointClouds.end();it++){
		v_pointClouds_keys.push_back(it->first);
		v_pointClouds_values.push_back(it->second);
	}

	int numOfVertices = pSurgeryGraph->numOfVertices;

	oa  << BOOST_SERIALIZATION_NVP(numOfVertices);
	oa  << BOOST_SERIALIZATION_NVP(v_verts);
	oa  << BOOST_SERIALIZATION_NVP(v_collapsed_keys);
	oa  << BOOST_SERIALIZATION_NVP(v_collapsed_values);
	oa  << BOOST_SERIALIZATION_NVP(v_segmentIndex_keys);
	oa  << BOOST_SERIALIZATION_NVP(v_segmentIndex_values);
	oa  << BOOST_SERIALIZATION_NVP(v_pointClouds_keys);
	oa  << BOOST_SERIALIZATION_NVP(v_pointClouds_values);

	return true;
}
//---------------------------------------------------------------------------
bool Export_Wrapper::serializeDistanceMatrix(char * file, Array2D<float>& distanceMatrix){

	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);

	vector<float> dm(distanceMatrix.dim1() * distanceMatrix.dim2());

	for (int i=0; i < distanceMatrix.dim1(); i++)
		for (int j=0; j < distanceMatrix.dim2(); j++)
			dm[j * distanceMatrix.dim1() + i] = distanceMatrix[i][j];

	int numOfVertices = distanceMatrix.dim1();

	oa  << BOOST_SERIALIZATION_NVP(numOfVertices);
	oa  << BOOST_SERIALIZATION_NVP(dm);

	return true;
}


void Export_Wrapper::exportWormMeshToCollada(meshes::IndexedFace *mesh, std::string file){
	exporter.Export(mesh, file);
}

void Export_Wrapper::exportWormMeshAndSkeletonToCollada(meshes::IndexedFace *mesh, SN::SkeletonNode * pSkeleton, std::string file){

	exporter.Export(mesh, pSkeleton, file);

}

void Export_Wrapper::addSkinningDataToMeshForExport(meshes::MeshSkin *meshSkin){

	for (int i=0; i < wormSkinningData.numOfVertices; i++){
		meshSkin->influences.push_back(config.NUM_OF_CTRL_BONES);
	}

	for (int i=0; i < wormSkinningData.numOfVertices * config.NUM_OF_CTRL_BONES; i++){
		meshSkin->jointIDs.push_back((int)wormSkinningData.indices[i]);
	}

	for (int i=0; i < wormSkinningData.numOfVertices * config.NUM_OF_CTRL_BONES; i++){
		meshSkin->skinWeights.push_back(wormSkinningData.weights[i]);
	}
}


void Export_Wrapper::exportWormMeshAndSkinningDataToCollada(meshes::IndexedFace *mesh, SN::SkeletonNode * pSkeleton, std::string file){

	meshes::MeshSkin * meshSkin = new meshes::MeshSkin(mesh);

	addSkinningDataToMeshForExport(meshSkin);
	
	exporter.Export(meshSkin, pSkeleton, file);

	delete meshSkin;
}

void Export_Wrapper::exportWormMeshAndSkinningDataToXML(meshes::IndexedFace *mesh, std::string file){

	meshes::MeshSkin * meshSkin = new meshes::MeshSkin(mesh);

	addSkinningDataToMeshForExport(meshSkin);

	meshes::SerializableMeshSkin * serMesh = new meshes::SerializableMeshSkin(meshSkin);

	//make an archive
	ofstream ofs(file);
	assert(ofs.good());

	boost::archive::xml_oarchive oa(ofs);
	oa  << BOOST_SERIALIZATION_NVP(serMesh);

	delete meshSkin;
	delete serMesh;
}

#endif

std::string Export_Wrapper::PadNumber(int number, int places) {
	std::string result = "";
	if (number != 0){
		while (powf(10, places) > number) {
			result += "0";
			places--;
		}
	} else {
		for (int i=0; i < places; i++){
			result += "0";
		}
	}
	result += std::to_string(number);
	return result;
}

void Export_Wrapper::ExportMeshToOFFFile(meshes::IndexedFace *mesh, std::string fileName) {
	int numOfVerts = mesh->vertices.size() / 3;
	int numOfTriangles = mesh->indices.size() / 3;

	std::ofstream off(fileName);

	off << "OFF" << std::endl;
	off << numOfVerts << " " << numOfTriangles << " 0" << std::endl;

	for (int i = 0; i < numOfVerts; i++) {
		mmath::CVector3 P = mesh->GetVertex(i);
		off << P.x << " " << P.y << " " << P.z << std::endl;
	}

	for (int i = 0; i < numOfTriangles; i++) {
		mmath::CVector3 T = mesh->GetTriangle(i);
		off << "3 " << (int)T.x << " " << (int)T.y << " " << (int)T.z << std::endl;
	}
}





#pragma once
#include <meshes/Mesh.h>
#include <string>
#include <glm\glm.hpp>
#include <mmath/mmath.h>

namespace meshes {

	struct CVectorMesh
	{
		std::vector<mmath::CVector3> vertices;
		std::vector<mmath::CVector3> normals;
		std::vector<mmath::CVector3> tangents;
		std::vector<mmath::CVector2> uvs;

		std::vector<int> indices;
		std::vector<float> fdata;
		std::vector<int> idata;

		std::vector<Material> materials;
		BoundingBox modelbb;

		CVectorMesh(){

		}
		
		CVectorMesh(Mesh pModel){
			for (int i=0; i < pModel.vertices.size() / 3; i++){
				vertices.push_back(mmath::CVector3(pModel.vertices[i * 3], pModel.vertices[i * 3 + 1], pModel.vertices[i * 3 + 2]));
			}

			for (int i=0; i < pModel.indices.size(); i++){
				indices.push_back(pModel.indices[i]);
			}

			for (int i=0; i < pModel.normals.size() / 3; i++){
				normals.push_back(mmath::CVector3(pModel.normals[i * 3], pModel.normals[i * 3 + 1], pModel.normals[i * 3 + 2]));
			}

			for (int i=0; i < pModel.tangents.size() / 3; i++){
				tangents.push_back(mmath::CVector3(pModel.tangents[i * 3], pModel.tangents[i * 3 + 1], pModel.tangents[i * 3 + 2]));
			}

			for (int i=0; i < pModel.uvs.size() / 2; i++){
				uvs.push_back(mmath::CVector2(pModel.uvs[i * 2], pModel.uvs[i * 2 + 1]));
			}

			indices = pModel.indices;
			fdata = pModel.fdata;
			idata = pModel.idata;
			materials = pModel.materialDefinitions;
			modelbb = pModel.modelbb;
		}

		Mesh getMesh(){
			Mesh pModel;

			for (int i=0; i < vertices.size(); i++){
				pModel.vertices.push_back(vertices[i].x);
				pModel.vertices.push_back(vertices[i].y);
				pModel.vertices.push_back(vertices[i].z);
			}

			for (int i=0; i < pModel.indices.size(); i++){
				pModel.indices[i] = indices[i];
			}

			for (int i=0; i < normals.size(); i++){
				pModel.normals.push_back(normals[i].x);
				pModel.normals.push_back(normals[i].y);
				pModel.normals.push_back(normals[i].z);
			}

			for (int i=0; i < tangents.size(); i++){
				pModel.tangents.push_back(tangents[i].x);
				pModel.tangents.push_back(tangents[i].y);
				pModel.tangents.push_back(tangents[i].z);
			}

			for (int i=0; i < uvs.size(); i++){
				pModel.uvs.push_back(uvs[i].x);
				pModel.uvs.push_back(uvs[i].y);
			}

			pModel.indices = indices;
			pModel.fdata = fdata;
			pModel.idata = idata;
			pModel.materialDefinitions = materials;
			pModel.modelbb = modelbb;

			return pModel;
		}
	};

}
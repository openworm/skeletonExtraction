#include "m_assimpLoader.h"

void ImportModel(meshes::Mesh * pModel, structure::t3DModel * p_3DModel, char * strFileName, ModelController::CModel * sdfController, SN::SkeletonNode * pNode){

	releaseStructure(p_3DModel);
	pModel->materialDefinitions.clear();
	pModel->materialIndices.clear();
	pModel->indices.clear();
	pModel->uvs.clear();
	pModel->vertices.clear();
	pModel->tangents.clear();
	pModel->normals.clear();
	
	// Create an instance of the Importer class
	Assimp::Importer importer;

	std::string pFile = std::string(strFileName);

	const aiScene* scene = importer.ReadFile( pFile, aiProcess_JoinIdenticalVertices | aiProcess_Triangulate | aiProcess_SplitLargeMeshes | aiProcess_FindDegenerates | aiProcess_FindInvalidData | aiProcess_ValidateDataStructure);
	// | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices  | aiProcess_Triangulate  | aiProcess_ImproveCacheLocality

	sdfController->LoadAssimp((aiScene*)scene);

	// If the import failed, report it
	if(!scene)
	{
		printf("%s\n", importer.GetErrorString());
		return;
	}

	p_3DModel->numOfObjects = scene->mNumMeshes;
	p_3DModel->pObject = std::vector<structure::t3DObject>(scene->mNumMeshes);

	if (scene->HasMaterials()){
		p_3DModel->pMaterials = std::vector<structure::tMaterialInfo>(scene->mNumMaterials);
		pModel->materialDefinitions.reserve(scene->mNumMaterials);

		p_3DModel->numOfMaterials = scene->mNumMaterials;
		for (int t = 0; t < scene->mNumMaterials; t++){
			structure::tMaterialInfo pMat = structure::tMaterialInfo();
			aiMaterial * mat = scene->mMaterials[t];

			meshes::Material material;

			aiString name;
			mat->Get(AI_MATKEY_NAME,name);
			aiColor3D color (0.f,0.f,0.f);
			mat->Get(AI_MATKEY_COLOR_DIFFUSE,color);
			
			pMat.color[0] = 255 * color.r;
			pMat.color[1] = 255 * color.g;
			pMat.color[2] = 255 * color.b;

			material.diffuseColor = glm::vec4(color.r, color.g, color.b, 1.0f);

			int texIndex = 0;
			aiString texPath;  // filename

			aiReturn texFound = scene->mMaterials[t]->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath);
			while (texFound == AI_SUCCESS) {
				strcpy(pMat.strFile, texPath.data);
				texIndex++;
				texFound = scene->mMaterials[t]->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath);
			}

			strcpy(pMat.strName, name.data);

			material.material = std::string(name.C_Str());
			material.texFile = std::string(texPath.C_Str());

			p_3DModel->pMaterials[t] = pMat;
			pModel->materialDefinitions.push_back(material);
		}
	}

	structure::BoundingBox * pModelMax = new structure::BoundingBox(); 
	int i=0;

	glLoadIdentity();

	aiMatrix4x4 matica = scene->mRootNode->mTransformation;
	matica.Transpose();
	glMultMatrixf((float*)&matica);

	GLfloat m[16]; 
	glGetFloatv(GL_MODELVIEW_MATRIX, m);
	m[12] = 0.0;
	m[13] = 0.0;
	m[14] = 0.0;
	m[15] = 1.0;
	p_3DModel->rootTrans = Mat4(m);

	recursiveModelFill(&i, pModel, p_3DModel, scene, scene->mRootNode, pModelMax, pNode);

	p_3DModel->modelbb = *pModelMax;

	pModel->modelbb.x_max = pModelMax->x_max;
	pModel->modelbb.x_min = pModelMax->x_min;

	pModel->modelbb.y_max = pModelMax->y_max;
	pModel->modelbb.y_min = pModelMax->y_min;

	pModel->modelbb.z_max = pModelMax->z_max;
	pModel->modelbb.z_min = pModelMax->z_min;
}

void recursiveModelFill(int *i, meshes::Mesh * pModel, structure::t3DModel * p_3DModel, const aiScene *sc,const aiNode* nd, structure::BoundingBox * pModelMax, SN::SkeletonNode * pNode){

	aiMatrix4x4 m = nd->mTransformation;
	m.Transpose();
	//glPushMatrix();
	//glMultMatrixf((float*)&m);

	//float mw[16];
	// get the current modelview matrix
	//glGetFloatv(GL_MODELVIEW_MATRIX , mw);
	//aiMatrix4x4 m = aiMatrix4x4();
	//memcpy (&m.a1, mw, 16*4 );


	for (int n=0; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* tmpMesh = sc->mMeshes[nd->mMeshes[n]];
		structure::t3DObject pObject = structure::t3DObject();

		pObject.numOfFaces = tmpMesh->mNumFaces;
		pObject.pFaces = new structure::tFace[tmpMesh->mNumFaces];

		//int index = 0;

		for (unsigned int t = 0; t < tmpMesh->mNumFaces; ++t)
		{
			const struct aiFace* face = &tmpMesh->mFaces[t];
			structure::tFace pFace = structure::tFace();
			memcpy(&pFace.coordIndex, face->mIndices,3 * sizeof(unsigned int));
			memcpy(&pFace.vertIndex, face->mIndices,3 * sizeof(unsigned int));

			//memcpy(&pObject.pIndices[index], face->mIndices,3 * sizeof(unsigned int));
			//index += 3;

			pObject.pFaces[t] = pFace;

			for (int v=0; v < 3; v++){
				pModel->indices.push_back(face->mIndices[v]);
			}
		}

		pObject.materialID = tmpMesh->mMaterialIndex;

		pObject.numOfVerts = tmpMesh->mNumVertices;
		pObject.pVerts = new CVector3[tmpMesh->mNumVertices];
		pObject.pNormals = new CVector3[tmpMesh->mNumVertices];
		pObject.pTangents = new CVector3[tmpMesh->mNumVertices];
		pObject.pTexVerts = new CVector2[tmpMesh->mNumVertices];
		if (tmpMesh->HasPositions())
		{
			for (unsigned int t = 0; t < tmpMesh->mNumVertices; ++t)
			{
				CVector3 pVert = CVector3();
				memcpy(&pVert, &tmpMesh->mVertices[t], 3 * sizeof(float));

				CVector3 newp = CVector3();

				newp.x = m.a1 * pVert.x + m.b1 * pVert.y + m.c1 * pVert.z + m.d1;
				newp.y = m.a2 * pVert.x + m.b2 * pVert.y + m.c2 * pVert.z + m.d2;
				newp.z = m.a3 * pVert.x + m.b3 * pVert.y + m.c3 * pVert.z + m.d3;

				pVert = newp;

				pObject.pVerts[t] = pVert;

				pModel->vertices.push_back(pVert.x);
				pModel->vertices.push_back(pVert.y);
				pModel->vertices.push_back(pVert.z);

				
				if ( pVert.x < pModelMax->x_min) {
					pModelMax->x_min = pVert.x;
				}
				if (pVert.x > pModelMax->x_max) {
					pModelMax->x_max = pVert.x;
				}

				if ( pVert.y < pModelMax->y_min) {
					pModelMax->y_min = pVert.y;
				}
				if ( pVert.y > pModelMax->y_max) {
					pModelMax->y_max = pVert.y;
				}
				if ( pVert.z < pModelMax->z_min) {
					pModelMax->z_min = pVert.z;
				}
				if ( pVert.z > pModelMax->z_max) {
					pModelMax->z_max = pVert.z;
				}

			}

		}
		if (tmpMesh->HasTextureCoords(0))
			for (unsigned int t = 0; t < tmpMesh->mNumVertices; ++t)
			{
				CVector2 pTex = CVector2();
				memcpy(&pTex, &tmpMesh->mTextureCoords[0][t], 2 * sizeof(float));
				pObject.pTexVerts[t] = pTex;

				pModel->uvs.push_back(pTex.x);
				pModel->uvs.push_back(pTex.y);
			}

		
		aiMaterial *mtl = sc->mMaterials[tmpMesh->mMaterialIndex];
		int texIndex = 0;
		aiString texPath;
		pObject.bHasTexture = (AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath));


		if (tmpMesh->HasNormals()){
			for (unsigned int t = 0; t < tmpMesh->mNumVertices; ++t)
			{
				CVector3 pNorm = CVector3();
				memcpy(&pNorm, &tmpMesh->mNormals[t], 3 * sizeof(float));
				pObject.pNormals[t] = pNorm;

				pModel->normals.push_back(pNorm.x);
				pModel->normals.push_back(pNorm.y);
				pModel->normals.push_back(pNorm.z);
			}
		}
		if (tmpMesh->HasTangentsAndBitangents()){
			for (unsigned int t = 0; t < tmpMesh->mNumVertices; ++t)
			{
				CVector3 pTang = CVector3();
				memcpy(&pTang, &tmpMesh->mTangents[t], 3 * sizeof(float));
				pObject.pTangents[t] = pTang;

				pModel->normals.push_back(pTang.x);
				pModel->normals.push_back(pTang.y);
				pModel->normals.push_back(pTang.z);
			}
		}

		if (tmpMesh->HasBones()){
			for (int b = 0; b < tmpMesh->mNumBones; ++b) {
				aiBone * tmpBone = tmpMesh->mBones[b];
				aiMatrix4x4 m = tmpBone->mOffsetMatrix;

				Array2D<float> mm = Array2D<float>(4, 4, 0.0f);
				for (int x=0; x < 4; x++)
					for (int y=0; y < 4; y++)
						mm[x][y] = m[x][y];

				//UNCOMMENT ((skl::SkeletonNode*)pNode)->matrices.currentAffine = mm.copy();

				//logg.log(0, mm);

				//UNCOMMENT ((skl::SkeletonNode*)pNode)->matrices.getQuaternionFromAffine();
				//UNCOMMENT ((skl::SkeletonNode*)pNode)->matrices.getTranslationFromAffine();

				for (int w = 0; w < tmpMesh->mBones[b]->mNumWeights; ++w) {
					
					int vertexID = tmpMesh->mBones[b]->mWeights[w].mVertexId;
					float weight = tmpMesh->mBones[b]->mWeights[w].mWeight; 
					int kkk = 0;
				}
				
			}
			
		}
		
		p_3DModel->pObject[*i] = pObject;
		(*i)++;
	}

	// fill all children
	for (int c = 0; c < nd->mNumChildren; ++c) {
		//UNCOMMENT skl::
		SN::SkeletonNode * pChildNode = new SN::SkeletonNode();
		pNode->nodes.push_back(pChildNode);
		recursiveModelFill(i ,pModel, p_3DModel, sc, nd->mChildren[c], pModelMax, pChildNode);
	}

	//glPopMatrix();

}
//---------------------------------------------------------------------------
#include <inline/SkeletonNode/s_skeletonNode.h>


inline BonesMatrices::BonesMatrices(){
	currentAffine = Array2D< float >(4, 4, 0.0f);
	for (int i=0; i < 4; i++)
		currentAffine[i][i] = 1.0f;
	qRotation = CVector4(0,1,0,0);
}

//---------------------------------------------------------------------------

inline skl::SkeletonNode * findNodeWithId(skl::SkeletonNode * pRoot, std::string id){
	std::vector<skl::SkeletonNode*> queue;
	queue.push_back(pRoot);

	while (queue.size() > 0){
		skl::SkeletonNode* pNode = queue[queue.size() - 1];
		queue.pop_back();

		if (pNode->sId == id)
			return pNode;

		for (int i=0; i < pNode->nodes.size(); i++){
			skl::SkeletonNode* pSon = (skl::SkeletonNode*)pNode->nodes[i];
			queue.push_back(pSon);
		}
	}
}

inline void BonesMatrices::logAffine(){
	#ifdef LOG
		logg.logTransformation(0, currentAffine);
	#endif
}

inline void BonesMatrices::computeAffineTransformation(){
	Array2D<float> matrix3x3Rot = QuaternionToMatrix3x3(qRotation);
	for (int i=0; i < 3; i++)
		for (int j=0; j < 3; j++)
			currentAffine[i][j] = matrix3x3Rot[i][j];
	currentAffine[3][0] = 0.0f;
	currentAffine[3][1] = 0.0f;
	currentAffine[3][2] = 0.0f;
	currentAffine[3][3] = 1.0f;
	currentAffine[0][3] = vTranslation.x;
	currentAffine[1][3] = vTranslation.y;
	currentAffine[2][3] = vTranslation.z;
}

inline CVector4 BonesMatrices::getQuaternionFromAffine(){
	return Matrix3x3ToQuaternion(currentAffine);
}


inline CVector3 BonesMatrices::getTranslationFromAffine(){
	return CVector3(currentAffine[0][3], currentAffine[1][3], currentAffine[2][3]);
}

inline void copySNSkeletonNodeToSkl(SN::SkeletonNode * src, skl::SkeletonNode * dest){
	copySNSkeletonNodeToSklImpl(src, dest);
	//assignFathersForSkeletonTree(dest);

}

inline void copySNSkeletonNodeToSklImpl(SN::SkeletonNode * src, skl::SkeletonNode * dest){

	dest->id = src->id;
	//dest->sId = src->sId;
	dest->selected = false;
	dest->point = src->point;
	dest->nodes = std::vector<SN::SkeletonNode*>();
	//dest->father = src->father;
	dest->cyclic = src->cyclic;

	BonesMatrices bindPoseMatrices;

	bindPoseMatrices.qRotation = CVector4(0,1,0,0);
	bindPoseMatrices.vTranslation = src->point;
	bindPoseMatrices.computeAffineTransformation();

	BonesMatrices matrices;

	matrices.qRotation = CVector4(0,1,0,0);
	matrices.vTranslation = src->point;
	matrices.computeAffineTransformation();

	copyBonesMatrices(&bindPoseMatrices, &dest->bindPoseMatrices);
	copyBonesMatrices(&matrices, &dest->matrices);
	for (int i = 0; i < src->nodes.size(); i++) {
		skl::SkeletonNode * p = new skl::SkeletonNode();
		p->father = dest;
		dest->nodes.push_back(p);
		copySNSkeletonNodeToSkl((skl::SkeletonNode *)src->nodes[i], p);
	}

}

inline void copyBonesMatrices(BonesMatrices * in, BonesMatrices * out){
	out->qRotation = in->qRotation;
	out->vTranslation = in->vTranslation;
	out->currentAffine = in->currentAffine.copy();
}

#pragma package(smart_init)

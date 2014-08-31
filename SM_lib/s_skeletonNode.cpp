//---------------------------------------------------------------------------
#include "s_skeletonNode.h"

//---------------------------------------------------------------------------

void BonesMatrices::logAffine(){
	#ifdef LOG
		logg.logTransformation(0, currentAffine);
	#endif
}

void BonesMatrices::computeAffineTransformation(){
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

CVector4 BonesMatrices::getQuaternionFromAffine(){
	return Matrix3x3ToQuaternion(currentAffine);
}


CVector3 BonesMatrices::getTranslationFromAffine(){
	return CVector3(currentAffine[0][3], currentAffine[1][3], currentAffine[2][3]);
}
#pragma package(smart_init)

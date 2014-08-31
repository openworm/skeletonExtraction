#pragma once
#include <mmath\mmath.h>
#include <vector>
#include <boost/serialization/version.hpp>

using namespace std;
using namespace mmath;

class BonesMatrices {
public:
	CVector3 vTranslation;
	CVector4 qRotation;
	Array2D<float> currentAffine;
	BonesMatrices() {}
	void logAffine() {};
	void computeAffineTransformation() {};
	CVector4 getQuaternionFromAffine() {return CVector4();};
	CVector3 getTranslationFromAffine() {return CVector3();};

private:


	friend class boost::serialization::access;
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar  & BOOST_SERIALIZATION_NVP(vTranslation);
		ar  & BOOST_SERIALIZATION_NVP(qRotation);
	}

};

struct SkeletonNode {
	CVector3 point;
	bool selected;
	int id;
	string sId;
	SkeletonNode* father;
	BonesMatrices matrices;
	BonesMatrices bindPoseMatrices;
	vector<SkeletonNode*> nodes;
	bool cyclic;

	SkeletonNode(){
		selected = false;
		point = CVector3(FLT_MAX,FLT_MAX,FLT_MAX);
		father = NULL;
		cyclic = false;
	}
private:


	friend class boost::serialization::access;
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar  & BOOST_SERIALIZATION_NVP(id);
		ar  & BOOST_SERIALIZATION_NVP(selected);
		ar  & BOOST_SERIALIZATION_NVP(point);
		ar  & BOOST_SERIALIZATION_NVP(sId);
		ar  & BOOST_SERIALIZATION_NVP(cyclic);
		//ar  & BOOST_SERIALIZATION_NVP(father);
		ar  & BOOST_SERIALIZATION_NVP(matrices);
		ar  & BOOST_SERIALIZATION_NVP(bindPoseMatrices);
		ar  & BOOST_SERIALIZATION_NVP(nodes);
	}

};


BOOST_CLASS_VERSION(SkeletonNode, 0)
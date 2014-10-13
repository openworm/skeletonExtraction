#ifndef s_skeletonNodeH
#define s_skeletonNodeH

#include <windows.h>

//#include "m_math.h"
#include <mmath/mmath.h>
#include <SkeletonNode/SkeletonNode.h>
using namespace mmath;

#ifdef _LOG
	#include <logs/log.h>
#endif

//---------------------------------------------------------------------------
class BonesMatrices {
public:
	CVector3 vTranslation;
	CVector4 qRotation;
	Array2D<float> currentAffine;
	BonesMatrices();
	void logAffine();
	void computeAffineTransformation();
	CVector4 getQuaternionFromAffine();
	CVector3 getTranslationFromAffine();
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


namespace skl {
	class SkeletonNode : public SN::SkeletonNode {
		friend class ::boost::serialization::access;
	public:
		bool selected;
		std::string sId;

		BonesMatrices matrices;
		BonesMatrices bindPoseMatrices;

		SkeletonNode() : SN::SkeletonNode() {
			selected = false;
			sId = "";
		};
		SkeletonNode(SN::SkeletonNode *node);
		using SN::SkeletonNode::~SkeletonNode;

	protected:
		// When the class Archive corresponds to an output archive, the
		// & operator is defined similar to <<.  Likewise, when the class Archive
		// is a type of input archive the & operator is defined similar to >>.
		/*template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("BaseClass", boost::serialization::base_object<SN::SkeletonNode>(*this));
		}*/
		template<class Archive>
		void save(Archive & ar, const unsigned int version) const 
		{
			ar & boost::serialization::make_nvp("BaseClass", boost::serialization::base_object<SN::SkeletonNode>(*this));
			ar & BOOST_SERIALIZATION_NVP(selected);
			ar & BOOST_SERIALIZATION_NVP(sId);
			std::vector<SkeletonNode*> childs;
			for (int i = 0; i < nodes.size(); i++) {
				childs.push_back((SkeletonNode*)nodes[i]);
			}
			ar & BOOST_SERIALIZATION_NVP(childs);
		}
		template<class Archive>
		void load(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("BaseClass", boost::serialization::base_object<SN::SkeletonNode>(*this));
			ar & BOOST_SERIALIZATION_NVP(selected);
			ar & BOOST_SERIALIZATION_NVP(sId);
			std::vector<SkeletonNode*> childs;
			ar & BOOST_SERIALIZATION_NVP(childs);
			for (int i = 0; i < childs.size(); i++) {
				nodes.push_back((SN::SkeletonNode*)childs[i]);
			}
		}
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			_serialize = false;
			boost::serialization::split_member(ar, *this, version);
			_serialize = true;
		}
	};
}

skl::SkeletonNode * findNodeWithId(skl::SkeletonNode * pRoot, std::string id);

BOOST_CLASS_VERSION(skl::SkeletonNode, 0);

#include "s_skeletonNode.inl"

//---------------------------------------------------------------------------
#endif
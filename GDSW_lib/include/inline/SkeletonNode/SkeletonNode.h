#pragma once
#include <mmath\mmath.h>
#include <vector>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#pragma warning(push, 0)
#include <boost\archive\basic_xml_iarchive.hpp>
#include <boost\archive\xml_iarchive.hpp>
#include <boost\archive\xml_oarchive.hpp>
#pragma pop

namespace SN {
	class SkeletonNode
	{
		friend class ::boost::serialization::access;
	public:
		mmath::CVector3 point;
		int id;
		SkeletonNode *father;
		SkeletonNode *cyclic;
		std::vector<SkeletonNode*> nodes;
		bool _serialize;
		SkeletonNode() : point(mmath::CVector3()), id(0), father(NULL), cyclic(NULL), _serialize(true) {};
		SkeletonNode(float x, float y, float z, SkeletonNode *father = NULL) : point(mmath::CVector3(x,y,z)), id(0), father(father), cyclic(NULL), _serialize(true) {};
		SkeletonNode(float x, float y, float z, int id, SkeletonNode *father = NULL) : point(mmath::CVector3(x,y,z)), id(id), father(father), cyclic(NULL), _serialize(true) {};
		SkeletonNode(mmath::CVector3 point, SkeletonNode *father = NULL) : point(point), id(0), father(father), cyclic(NULL), _serialize(true) {};
		SkeletonNode(mmath::CVector3 point, int id, SkeletonNode *father = NULL) : point(point), id(id), father(father), cyclic(NULL), _serialize(true) {};
		~SkeletonNode(void) {
			for (int i = 0; i < nodes.size(); i++) {
				delete nodes[i];
			}
		}

		void addChild(SkeletonNode *node) {
			nodes.push_back(node);
			node->father = this;
		};
	protected:
		// When the class Archive corresponds to an output archive, the
		// & operator is defined similar to <<.  Likewise, when the class Archive
		// is a type of input archive the & operator is defined similar to >>.
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(id);
			ar & BOOST_SERIALIZATION_NVP(point);
			ar & BOOST_SERIALIZATION_NVP(father);
			ar & BOOST_SERIALIZATION_NVP(cyclic);
			if (_serialize) {
				ar & BOOST_SERIALIZATION_NVP(nodes);
			}
		}
	};
}

BOOST_CLASS_EXPORT_KEY(SN::SkeletonNode); 
BOOST_CLASS_VERSION(SN::SkeletonNode, 0)
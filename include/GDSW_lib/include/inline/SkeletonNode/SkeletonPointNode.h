#pragma once
#include "SkeletonGeneralNode.h"

namespace SNG {
	class PointNode : public virtual SkeletonNode
	{
		friend class ::boost::serialization::access;
	public:
		mmath::CVector3 point;

		PointNode() : SkeletonNode(), point(mmath::CVector3()) {
			nodeType = NodeType::PointNode;
		};
		~PointNode(void) { }

#pragma region Skeleton Querring
		virtual mmath::CVector3 GetPosition(int idx = 0) {
			return point;
		}
		virtual int GetElementCount() {
			return 0;
		}
		virtual mmath::CVector3 GetConnectivity(int idx = 0) {
			return mmath::CVector3(-1, -1, -1);
		}
#pragma endregion
	};
}

//BOOST_CLASS_EXPORT_GUID(SNG::PointNode, "SNG::SheetNode");
//BOOST_CLASS_EXPORT_KEY(SNG::PointNode); 
//BOOST_CLASS_VERSION(SNG::PointNode, 0)
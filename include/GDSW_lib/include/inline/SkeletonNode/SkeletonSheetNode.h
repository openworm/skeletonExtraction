#pragma once
#include "SkeletonGeneralNode.h"

namespace SNG {
	namespace {
		const int ELEMENT_SIZE = 3;
	}
	class SheetNode : public virtual SkeletonNode
	{
		friend class ::boost::serialization::access;
	public:
		std::vector<mmath::CVector3> points;
		std::vector<int> indices;

		SheetNode() : SkeletonNode() {
			nodeType = NodeType::SheetNode;
		};
		~SheetNode(void) { }		

#pragma region Skeleton Querring
		virtual mmath::CVector3 GetPosition(int idx = 0) {
			return points[idx];
		}
		virtual int GetElementCount() {
			return indices.size() / ELEMENT_SIZE;
		}
		virtual mmath::CVector3 GetConnectivity(int idx = 0) {
			idx *= ELEMENT_SIZE;
			mmath::CVector3 result;
			result.x = indices[idx + 0];
			result.y = indices[idx + 1];
			result.z = indices[idx + 2];

			return result;
		}
#pragma endregion
	};
}

//BOOST_CLASS_EXPORT_GUID(SNG::SheetNode, "SNG::SheetNode");
//BOOST_CLASS_EXPORT_KEY(SNG::SheetNode); 
//BOOST_CLASS_VERSION(SNG::SheetNode, 0)
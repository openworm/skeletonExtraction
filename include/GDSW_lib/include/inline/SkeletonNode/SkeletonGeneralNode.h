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

namespace SNG {
	typedef enum {
		BaseNode,
		PointNode,
		SheetNode
	} NodeType;

	class SkeletonNode
	{
		friend class ::boost::serialization::access;
	protected:
		SkeletonNode *father;
		std::vector<SkeletonNode*> nodes;
		std::vector<std::vector<mmath::CVector3> > connectionPointsVec;
		int id;
		NodeType nodeType;
	public:

		SkeletonNode() : nodeType(NodeType::BaseNode), id(0), father(NULL) {};
		~SkeletonNode(void) {
			for (int i = 0; i < nodes.size(); i++) {
				delete nodes[i];
			}
		}

#pragma region Skeleton Traversal
		int GetNumberOfNodes() {
			return nodes.size();
		}
		std::vector<mmath::CVector3> GetConnectionPoints(int nodeID) {
			return connectionPointsVec[nodeID];
		}
		SkeletonNode* GetNode(int nodeID) {
			return nodes[nodeID];
		}
		SkeletonNode* GetParent() {
			return father;
		}
		void SetParent(SkeletonNode *parent) {
			father = parent;
		}
#pragma endregion

#pragma region Skeleton Editing
		virtual void AddChild(SkeletonNode *node, std::vector<mmath::CVector3> &connectionPoints) {
			node->SetParent(this);
			nodes.push_back(node);
			connectionPointsVec.push_back(connectionPoints);
		}
#pragma endregion

#pragma region Skeleton Querring
		int GetID() {
			return id;
		}
		NodeType GetNodeType() {
			return nodeType;
		}
		virtual mmath::CVector3 GetPosition(int idx = 0) { return mmath::CVector3(); }
		virtual int GetElementCount() { return 0; }
		virtual mmath::CVector3 GetConnectivity(int idx = 0) { return mmath::CVector3(-1, -1, -1); }
#pragma endregion
	};
}

BOOST_CLASS_EXPORT_KEY(SNG::SkeletonNode); 
BOOST_CLASS_VERSION(SNG::SkeletonNode, 0)
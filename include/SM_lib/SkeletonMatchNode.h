#pragma once
#include <mmath\mmath.h>
#include <vector>
#include <boost/serialization/version.hpp>
#include <SkeletonNode\SkeletonNode.h>

using namespace std;
using namespace mmath;
using namespace SN;

class SkeletonMatchNode
{
	friend class boost::serialization::access;
public:
	CVector3 point;
	int id;
	int oldID;
	vector<float> dists;
	vector<int> betweenNodes;
	vector<vector<int> > oldIDs;
	vector<vector<CVector3> > positions;
	vector<SkeletonMatchNode*> nodes;
	SkeletonMatchNode* parent;
public:
	SkeletonMatchNode();
	SkeletonMatchNode(SkeletonNode* node, SkeletonMatchNode* _parent = NULL);
	SkeletonMatchNode(float x, float y, float z);
	SkeletonMatchNode(float x, float y, float z, int idx);
	~SkeletonMatchNode(void);

	void AddChild(SkeletonMatchNode *node);
	void RemoveChild(SkeletonMatchNode *node);
	void Trim();
protected:
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(id);
		ar & BOOST_SERIALIZATION_NVP(point);
		ar & BOOST_SERIALIZATION_NVP(nodes);
	}
};

BOOST_CLASS_VERSION(SkeletonMatchNode, 0)


SkeletonMatchNode* GetEnd(SkeletonMatchNode* node, int &between, float& dist);
SkeletonMatchNode* GetEnd(SkeletonMatchNode* node, int &between, float& dist, vector<CVector3> &pos, vector<int> &oldIDs);
int RecalculateIDs(SkeletonMatchNode* node);
int CountNodes(SkeletonMatchNode* node);
void SwapRoot(SkeletonMatchNode* root, SkeletonMatchNode* node);
SkeletonMatchNode* PrepareForTriming(SkeletonMatchNode* node);
void FixParents(SkeletonMatchNode* node);
#pragma once
#include <mmath\mmath.h>
#include "SkeletonGraph.h"
#include <vector>
#include <map>
#include <SkeletonNode\SkeletonNode.h>
#include <glm\glm.hpp>
#include "MatchingStruct.h"

using namespace std;
using namespace mmath;
using namespace SN;

class USkeletonNode
{
public:
	int id;
	int oldID;
	int count;
	CVector3 point;
	vector<CVector4> axisAngles;
	float parentDist;
	USkeletonNode* parent;
	vector<USkeletonNode*> nodes;

	USkeletonNode(void);
	USkeletonNode(int _id, CVector3 _point, USkeletonNode* _parent);
	USkeletonNode(int _id, CVector3 _point, float _parentDist, USkeletonNode* _parent);
	USkeletonNode(USkeletonNode* root, USkeletonNode* addRoot, float _parentDist);
	USkeletonNode(SkeletonGraphNode* N, int _id);
	USkeletonNode(SkeletonGraph* G, int _id = -1);
	USkeletonNode(SkeletonGraph* G, USkeletonNode* root, int gid, int skipId);
	USkeletonNode* SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root);
	USkeletonNode(SkeletonGraph* G, USkeletonNode* other, vector<int> mapping);
	USkeletonNode(SkeletonGraph* G, USkeletonNode* root, USkeletonNode* other, vector<int> mapping, int skipId);
	~USkeletonNode(void);

	void SetParent(USkeletonNode* node);
	void RemoveChild(USkeletonNode* node);
	bool ReplaceChild(USkeletonNode* child, USkeletonNode* node);
	SkeletonNode* ToSkeletonNode();
	void CalculateCorrespondingDoF(USkeletonNode* bind, float threshold, float axisThreshold);
	void CalculateCorrespondingDoF(USkeletonNode* bind, glm::mat4 M, float threshold, float axisThreshold);
};

struct DataStruct {
	USkeletonNode* root;
	int ignoreID;
	int selectID;
	int* id;

	DataStruct(USkeletonNode* _root, int _ignore, int _select, int* _id) : root(_root), ignoreID(_ignore), selectID(_select), id(_id) {};
};

struct NodeDist {
	USkeletonNode* node;
	float dist;

	NodeDist(USkeletonNode* _node, float _dist) : node(_node), dist(_dist) {};
};

USkeletonNode* SkipSameIds(USkeletonNode* node);

void AddSkeleton(USkeletonNode* oNode, USkeletonNode* aNode, vector<int> mapping, float lthreshold = 1);
void AddSkeleton(USkeletonNode* oNode, float oDist, USkeletonNode* aNode, float aDist, USkeletonNode* root, vector<int> mapping, float lthreshold);

void RecalculateIDs(USkeletonNode* node);
void RecalculateIDsAndExportOutput(USkeletonNode* node, vector<MatchingStruct>& output);

bool UniqueAxis(vector<CVector4>& axisAngles, CVector4 axis, float threshold);

void CleanUpCount(USkeletonNode* node);

void GetCloseDescendants(USkeletonNode* node, float threshold, vector<CVector3>& positions, vector<USkeletonNode*>& descendants, bool clear = false);

void Simplify(USkeletonNode* node, float threshold);

USkeletonNode* SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root, int fromID, int *_id);
USkeletonNode* SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root, int fromID, int *_id, vector<NodeDist>& out);

vector<USkeletonNode*> RecreateSkeletonsWithMatching(SkeletonGraph* A, SkeletonGraph* B, vector<int>& matching, map<int, MatchingSkeletonStruct>& o_map, float threshold);

void RecreateSkeletonsWithMatching(SkeletonGraph* A, DataStruct dA, SkeletonGraph* B, DataStruct dB, vector<int>& matching, map<int, MatchingSkeletonStruct>& o_map, float threshold);

void RecreateSkeleton(SkeletonGraph* G, DataStruct d);

void AddToMap(vector<NodeDist>& outA, vector<NodeDist>& outB, map<int, MatchingSkeletonStruct>& o_map, float threshold);

void MapOldIDs(USkeletonNode *root, map<int, int> &oldIDMap);
#pragma once
#include <mmath\mmath.h>
#include <vector>
#include "GraphEdge.h"

using namespace std;
using namespace mmath;

class SkeletonGraphNode {
public:
	int id;
	int oldID;
	CVector3 point;
	vector<SkeletonGraphNode*> neighborhood;
	vector<GraphEdge> edges;
public:
	SkeletonGraphNode(void);
	SkeletonGraphNode(int idx);
	~SkeletonGraphNode(void);

	void AddNeighborWithEdge(SkeletonGraphNode* neighbor, GraphEdge edge);
};


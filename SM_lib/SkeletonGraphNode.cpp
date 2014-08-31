#include "SkeletonGraphNode.h"


SkeletonGraphNode::SkeletonGraphNode(void) : id(0), point(CVector3())
{
}

SkeletonGraphNode::SkeletonGraphNode(int idx) : id(idx), point(CVector3())
{
}

SkeletonGraphNode::~SkeletonGraphNode(void)
{
}

void SkeletonGraphNode::AddNeighborWithEdge(SkeletonGraphNode* neighbor, GraphEdge edge) {
	neighborhood.push_back(neighbor);
	edges.push_back(edge);
}

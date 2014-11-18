#include "SkeletonGraph.h"
#include <deque>

SkeletonGraph::SkeletonGraph(void)
{
}


SkeletonGraph::~SkeletonGraph(void)
{
	FreeNodes();
}

//number of nodes as would return vector.size()
void SkeletonGraph::CreateGraphFromSkeleton(SkeletonMatchNode* root, int numOfNodes) {
	FreeNodes();
	nodes.reserve(numOfNodes);
	for (int i = 0; i < numOfNodes; i++) {
		SkeletonGraphNode* node = new SkeletonGraphNode(i);
		nodes.push_back(node);
	}

	deque<SkeletonMatchNode*> queue;
	queue.push_back(root);

	int id = 0;
	while (!queue.empty()) {
		SkeletonMatchNode* sklNode = queue.front();
		queue.pop_front();
		nodes[id]->point = sklNode->point;
		nodes[id]->oldID = sklNode->oldID;
		//edges are not oriented and parent would handle parent edge so we add only child edges
		for (int i = 0; i < sklNode->nodes.size(); i++) {
			GraphEdge ge = GraphEdge(sklNode->betweenNodes[i], sklNode->dists[i]);
			ge.oldIDs = sklNode->oldIDs[i];
			ge.positions = sklNode->positions[i];
			ge.fromId = sklNode->id;
			ge.toId = sklNode->nodes[i]->id;
			CreateEdge(sklNode->id, sklNode->nodes[i]->id, ge);
			queue.push_back(sklNode->nodes[i]);
		}
		id++;
	}
	CollectBranchNodes();
}

void SkeletonGraph::CollectBranchNodes() {
	branchNodes.clear();
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i]->neighborhood.size() > 2) {
			branchNodes.push_back(nodes[i]);
		}
	}
}

void SkeletonGraph::AddNode(SkeletonGraphNode* node) {
	nodes.push_back(node);
}

void SkeletonGraph::CreateEdge(int i, int j, GraphEdge edge) {
	nodes[i]->AddNeighborWithEdge(nodes[j], edge);
	nodes[j]->AddNeighborWithEdge(nodes[i], edge);
}

void SkeletonGraph::FreeNodes() {
	for (int i = 0; i < nodes.size(); i++) {
		delete nodes[i];
		nodes[i] = NULL;
	}
	nodes.clear();
}

bool SkeletonGraph::AreNeighbors(int i, int j, GraphEdge& edge) {
	for (int k = 0; k < nodes[i]->neighborhood.size(); k++) {
		if (nodes[i]->neighborhood[k]->id == j) {
			edge = nodes[i]->edges[k];
			return true;
		}
	}

	return false;
}

bool SkeletonGraph::AreNeighbors(int i, int j) {
	for (int k = 0; k < nodes[i]->neighborhood.size(); k++) {
		if (nodes[i]->neighborhood[k]->id == j) {
			return true;
		}
	}

	return false;
}
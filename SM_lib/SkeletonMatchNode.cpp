#include "SkeletonMatchNode.h"
#include <deque>

SkeletonMatchNode::SkeletonMatchNode (void) : point(CVector3()), id(0), parent(NULL) {
}

SkeletonMatchNode::SkeletonMatchNode (SkeletonNode* node, SkeletonMatchNode* _parent) : id(node->id), point(node->point), parent(parent) {
	oldID = node->id;
	for (int i = 0; i < node->nodes.size(); i++) {
		SkeletonMatchNode* newNode = new SkeletonMatchNode(node->nodes[i], this);
		nodes.push_back(newNode);
	}
}

SkeletonMatchNode::SkeletonMatchNode(float x, float y, float z) : point(CVector3(x, y, z)), id(0), parent(NULL) {
}

SkeletonMatchNode::SkeletonMatchNode(float x, float y, float z, int idx) : point(CVector3(x, y, z)), id(idx), parent(NULL) {
}

SkeletonMatchNode::~SkeletonMatchNode (void) {
	for (int i = 0; i < nodes.size(); i++) {
		delete nodes[i];
	}
}

void SkeletonMatchNode::AddChild(SkeletonMatchNode *node) {
	nodes.push_back(node);
}

void SkeletonMatchNode::RemoveChild(SkeletonMatchNode *node) {
	vector<SkeletonMatchNode *> temp;
	for (vector<SkeletonMatchNode *>::iterator it = nodes.begin(); it != nodes.end(); it++) {
		if ((*it) == node) {
			it = nodes.erase(it);
			return;
		}
	}
}

//trims the skeleton and stores distance and number of nodes in the original skeleton
void SkeletonMatchNode::Trim() {
	dists.clear();
	betweenNodes.clear();
	positions.clear();
	oldIDs.clear();

	for (int i = 0; i < nodes.size(); i++) {
		int between = 0;
		float dist = 0;
		vector<CVector3> pos;
		vector<int> ids;
		SkeletonMatchNode* node = GetEnd(nodes[i], between, dist, pos, ids);
		nodes[i] = node;
		betweenNodes.push_back(between);
		dists.push_back(dist);
		positions.push_back(pos);
		oldIDs.push_back(ids);
	}

	for (int i = 0; i < nodes.size(); i++) {
		nodes[i]->Trim();
	}
}

//finds the last link
SkeletonMatchNode* GetEnd(SkeletonMatchNode* node, int &between, float& dist) {
	vector<CVector3> temp;
	vector<int> temp2;
	return GetEnd(node, between, dist, temp, temp2);
}

SkeletonMatchNode* GetEnd(SkeletonMatchNode* node, int &between, float& dist, vector<CVector3> &pos, vector<int> &oldIDs) {
	SkeletonMatchNode* search = node;
	between = 0;
	dist = 0;
	pos.clear();
	oldIDs.clear();

	while (search->nodes.size() == 1) {
		between++;
		dist += Magnitude(search->point - search->nodes[0]->point);
		pos.push_back(search->point);
		oldIDs.push_back(search->oldID);
		search = search->nodes[0];
	}

	return search;
}

int RecalculateIDs(SkeletonMatchNode* node) {
	deque<SkeletonMatchNode*> queue;
	queue.push_back(node);
	int id = 0;

	while (!queue.empty()) {
		SkeletonMatchNode* aNode = queue.front();
		queue.pop_front();

		aNode->id = id;
		id++;

		for (int i = 0; i < aNode->nodes.size(); i++) {
			queue.push_back(aNode->nodes[i]);
		}
	}

	return id;
}

int CountNodes(SkeletonMatchNode* node) {
	deque<SkeletonMatchNode*> queue;
	queue.push_back(node);
	int nodes = 0;

	while (!queue.empty()) {
		SkeletonMatchNode* aNode = queue.front();
		queue.pop_front();

		nodes++;

		for (int i = 0; i < aNode->nodes.size(); i++) {
			queue.push_back(aNode->nodes[i]);
		}
	}

	return nodes;
}

void SwapRoot(SkeletonMatchNode* root, SkeletonMatchNode* node) {
	//node is the first that is BNP
	//everything going from root is OK
	//only route from node to root needs to be fixed
	SkeletonMatchNode *temp = node->parent;
	SkeletonMatchNode *parent = temp->parent;
	node->parent = NULL;
	temp->parent = node;
	temp->RemoveChild(node);
	node->AddChild(temp);

	while (temp != root) {
		temp->AddChild(parent);
		parent->RemoveChild(temp);
		SkeletonMatchNode *newParent = parent->parent;
		parent->parent = temp;
		temp = parent;
		parent = newParent;
	}
}

SkeletonMatchNode* PrepareForTriming(SkeletonMatchNode* node) {
	//this is already a leaf 
	if (node->nodes.size() <= 1) return node;
	int between;
	float dist;
	SkeletonMatchNode* newRoot = GetEnd(node->nodes[0], between, dist);
	SwapRoot(node, newRoot);
	return newRoot;
}

void FixParents(SkeletonMatchNode* node) {
	for (int i = 0; i < node->nodes.size(); i++) {
		node->nodes[i]->parent = node;
		FixParents(node->nodes[i]);
	}
}
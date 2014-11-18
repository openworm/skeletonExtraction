#include "USkeletonNode.h"
#include "Utility.h"
#include "m_math_additions.h"
#include <deque>
#include <set>

typedef pair<int, MatchingSkeletonStruct> o_map_pair;

using namespace std;
using namespace mmath;

USkeletonNode::USkeletonNode(void)
{
	count = 1;
}

USkeletonNode::USkeletonNode(int _id, CVector3 _point, USkeletonNode* _parent) {
	id = _id;
	count = 1;
	point = _point;
	parent = _parent;
	parentDist = Magnitude(point - parent->point);
	parent->nodes.push_back(this);
}

USkeletonNode::USkeletonNode(int _id, CVector3 _point, float _parentDist, USkeletonNode* _parent) {
	id = _id;
	count = 1;
	point = _point;
	parent = _parent;
	parentDist = _parentDist;
	//parent->nodes.push_back(this);
}

USkeletonNode::USkeletonNode(USkeletonNode* root, USkeletonNode* addRoot, float _parentDist) {
	id = root->id;
	count = 1;
	//point = addRoot->point;
	if (addRoot->parent != NULL && addRoot->parent->parent != NULL) {
		CVector3 u = Normalize(addRoot->parent->point - addRoot->parent->parent->point);
		CVector3 v = Normalize(addRoot->point - addRoot->parent->point);
		Quaternion q = QuaternionBetweenVectors(u, v);
		CVector3 w = Normalize(root->point - root->parent->point);
		w = QuaternionRotateVector(q, w);
		point = root->point + w * _parentDist;
	} else if (addRoot->parent != NULL) {
		CVector3 u = Normalize(addRoot->point - addRoot->parent->point);
		point = root->point + u * _parentDist;
	} else {
		point = root->point + Normalize(addRoot->point - root->point) * _parentDist;
	}
	parent = root;
	parentDist = _parentDist;
	parent->nodes.push_back(this);

	for (int i = 0; i < addRoot->nodes.size(); i++) {
		USkeletonNode* node = new USkeletonNode(this, addRoot->nodes[i], addRoot->nodes[i]->parentDist);
	}
}

/*USkeletonNode::USkeletonNode(USkeletonNode* root, USkeletonNode* addRoot) {
	id = root->id;
	point = addRoot->point;
	parent = root;
	parentDist = Magnitude(point - parent->point);
	parent->nodes.push_back(this);

	for (int i = 0; i < addRoot->nodes.size(); i++) {
		USkeletonNode* node = new USkeletonNode(this, addRoot->nodes[i]);
	}
}*/

USkeletonNode::USkeletonNode(SkeletonGraphNode* N, int _id) {
	id = _id;
	oldID = N->oldID;
	parent = NULL;
	point = N->point;
	parentDist = 0;
	count = 1;
}

USkeletonNode::USkeletonNode(SkeletonGraph* G, int _id) {
	if (_id == -1) _id = G->branchNodes[0]->id;
	parent = NULL;
	id = _id;
	point = G->nodes[_id]->point;
	parentDist = 0;
	count = 1;

	for (int i = 0; i < G->nodes[_id]->edges.size(); i++) {
		USkeletonNode* node = SkeletonNodesFromEdge(G->nodes[_id]->edges[i], this);
		new USkeletonNode(G, node, G->nodes[_id]->neighborhood[i]->id, id);
	}
}

USkeletonNode::USkeletonNode(SkeletonGraph* G, USkeletonNode* root, int gid, int skipId) {
	parent = root;
	root->nodes.push_back(this);
	id = gid;
	point = G->nodes[gid]->point;
	parentDist = Magnitude(point - parent->point);
	count = 1;

	for (int i = 0; i < G->nodes[gid]->edges.size(); i++) {
		if (G->nodes[gid]->neighborhood[i]->id == skipId) continue;
		USkeletonNode* node = SkeletonNodesFromEdge(G->nodes[gid]->edges[i], this);
		new USkeletonNode(G, node, G->nodes[gid]->neighborhood[i]->id, id);
	}
}

USkeletonNode* USkeletonNode::SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root) {
	bool reverse = (ge.fromId != root->id);
	int toId = reverse ? ge.fromId : ge.toId;
	USkeletonNode* croot = root;
	count = 1;

	for (int i = 0; i < ge.positions.size(); i++) {
		int idx = reverse ? (ge.positions.size() - i - 1) : i;
		USkeletonNode* node = new USkeletonNode();
		node->id = toId;
		node->point = ge.positions[idx];
		node->parent = croot;
		node->parentDist = Magnitude(node->point - croot->point);

		croot->nodes.push_back(node);

		croot = node;
	}

	return croot;
}

//other is intersection skeleton that has all nodes mapped to other nodes
USkeletonNode::USkeletonNode(SkeletonGraph* G, USkeletonNode* other, vector<int> mapping) {
	id = mapping[other->id];
	parent = NULL;
	parentDist = 0;
	point = G->nodes[id]->point;
	count = 1;

	vector<int> used;
	for (int i = 0; i < other->nodes.size(); i++) {
		USkeletonNode* ochild = other->nodes[i];
		int idx = mapping[ochild->id];
		GraphEdge ge;
		if (G->AreNeighbors(id, idx, ge)) {
			used.push_back(idx);
			USkeletonNode* node = SkeletonNodesFromEdge(ge, this);
			new USkeletonNode(G, node, SkipSameIds(other->nodes[i]), mapping, id);
		}
	}
	for (int i = 0; i < G->nodes[id]->neighborhood.size(); i++) {
		if (!inArray(used, G->nodes[id]->neighborhood[i]->id)) {
			USkeletonNode* node = SkeletonNodesFromEdge(G->nodes[id]->edges[i], this);
			new USkeletonNode(G, node, G->nodes[id]->neighborhood[i]->id, id);
		}
	}
}

USkeletonNode::USkeletonNode(SkeletonGraph* G, USkeletonNode* root, USkeletonNode* other, vector<int> mapping, int skipId) {
	id = mapping[other->id];
	parent = root;
	root->nodes.push_back(this);
	point = G->nodes[id]->point;
	parentDist = Magnitude(root->point - point);
	count = 1;

	vector<int> used;
	for (int i = 0; i < other->nodes.size(); i++) {
		USkeletonNode* ochild = other->nodes[i];
		int idx = mapping[ochild->id];
		GraphEdge ge;
		if (G->AreNeighbors(id, idx, ge)) {
			used.push_back(idx);
			USkeletonNode* node = SkeletonNodesFromEdge(ge, this);
			new USkeletonNode(G, node, SkipSameIds(other->nodes[i]), mapping, id);
		}
	}
	for (int i = 0; i < G->nodes[id]->neighborhood.size(); i++) {
		if (!inArray(used, G->nodes[id]->neighborhood[i]->id) && G->nodes[id]->neighborhood[i]->id != skipId) {
			USkeletonNode* node = SkeletonNodesFromEdge(G->nodes[id]->edges[i], this);
			new USkeletonNode(G, node, G->nodes[id]->neighborhood[i]->id, id);
		}
	}
}

USkeletonNode::~USkeletonNode(void)
{
	for (int i = 0; i < nodes.size(); i++) {
		delete nodes[i];
	}
}

void USkeletonNode::SetParent(USkeletonNode* node) {
	parent->RemoveChild(this);
	parent = node;
	parent->nodes.push_back(this);
	parentDist = Magnitude(point - parent->point);
}

void USkeletonNode::RemoveChild(USkeletonNode* node) {
	for (vector<USkeletonNode *>::iterator it = nodes.begin(); it != nodes.end(); it++) {
		if ((*it) == node) {
			it = nodes.erase(it);
			return;
		}
	}
}

bool USkeletonNode::ReplaceChild(USkeletonNode* child, USkeletonNode* node) {
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i] == child) {
			nodes[i] = node;
			return true;
		}
	}

	return false;
}

SkeletonNode* USkeletonNode::ToSkeletonNode() {
	SkeletonNode* skl = new SkeletonNode();
	skl->id = id;
	skl->point = point;
	//skl->cyclic = count;

	for (int i = 0; i < nodes.size(); i++) {
		SkeletonNode* nskl = nodes[i]->ToSkeletonNode();
		nskl->father = skl;
		skl->nodes.push_back(nskl);
	}

	return skl;
}

void USkeletonNode::CalculateCorrespondingDoF(USkeletonNode* bind, float threshold, float axisThreshold) {
	//bind->axisAngles.push_back(CVector4(0, 0, 0, 1));
	for (int i = 0; i < nodes.size(); i++) {
		if (i < bind->nodes.size()) {
			nodes[i]->CalculateCorrespondingDoF(bind->nodes[i], glm::mat4(1.0), threshold, axisThreshold);
		}
	}
}

void USkeletonNode::CalculateCorrespondingDoF(USkeletonNode* bind, glm::mat4 M, float threshold, float axisThreshold) {
	//have to be the same skeleton just posed differently
	//this is in bind position
	CVector4 axisAngle = CVector4(0, 0, 0, 1);

	//rotation from bind to this skeleton
	CVector3 bPoint = TransformCPoint(bind->point, M);
	CVector3 bpPoint = TransformCPoint(bind->parent->point, M);
	CVector3 u = Normalize(bPoint - bpPoint);
	CVector3 v = Normalize(point - parent->point);

	Quaternion q = QuaternionBetweenVectors(u, v);
	axisAngle = QuaternionToAxisAngle(q);
	axisAngle.s = axisAngle.s*180.0f/M_PI;

	M = AddRotation(M, axisAngle, parent->point);
	//if (axisAngle.s*180.0f/M_PI > threshold) {
	if (axisAngle.s > threshold) {
		if (UniqueAxis(bind->axisAngles, axisAngle, axisThreshold)) {
			//AddRotation(M, axisAngle, parent->point);
			bind->axisAngles.push_back(axisAngle);
		}
	} else {
		axisAngle = CVector4(0, 0, 0, 1);
	}
	//bind->axisAngles.push_back(axisAngle);

	for (int i = 0; i < nodes.size(); i++) {
		if (i < bind->nodes.size()) {
			nodes[i]->CalculateCorrespondingDoF(bind->nodes[i], M, threshold, axisThreshold);
		}
	}
}

USkeletonNode* SkipSameIds(USkeletonNode* node) {
	int id = node->id;
	while (node->nodes.size() == 1 && node->nodes[0]->id == id) {
		node = node->nodes[0];
	}

	return node;
}

void AddSkeleton(USkeletonNode* oNode, USkeletonNode* aNode, vector<int> mapping, float lthreshold) {
	//add for root
	oNode->count++;
	for (int i = 0; i < aNode->nodes.size(); i++) {
		if (i < oNode->nodes.size()) {
			AddSkeleton(oNode->nodes[i], oNode->nodes[i]->parentDist, aNode->nodes[i], aNode->nodes[i]->parentDist, oNode, mapping, lthreshold);
		} else {
			AddSkeleton(NULL, 0, aNode->nodes[i], aNode->nodes[i]->parentDist, oNode, mapping, lthreshold);
		}
	}
}

void AddSkeleton(USkeletonNode* oNode, float oDist, USkeletonNode* aNode, float aDist, USkeletonNode* root, vector<int> mapping, float lthreshold) {
	//no more to add
	if (aNode == NULL) return;
	//only nodes to add remain
	if (oNode == NULL) {
		USkeletonNode* node = new USkeletonNode(root, aNode, aDist);
		return;
	}
	//add closer node
	//if within threshold don't add and skip
	if (fabs(oDist - aDist) < lthreshold) {
		//move both
		//oNode->point = oNode->point * (float)oNode->count;
		//oNode->point = oNode->point + aNode->point;
		aNode->count++;

		vector<CVector3> positions;
		vector<USkeletonNode*> descendants;
		GetCloseDescendants(aNode, lthreshold, positions, descendants);

		/*for (int i = 0; i < positions.size(); i++) {
			oNode->point = oNode->point + positions[i];
			oNode->count++;
		}
		oNode->point = oNode->point / (float)oNode->count;

		for (int i = 0; i < oNode->nodes.size(); i++) {
			oNode->nodes[i]->parentDist = Length(oNode->point - oNode->nodes[i]->point);
		}*/

		for (int i = 0; i < descendants.size(); i++) {
			float dist = Length(aNode->point - descendants[i]->point);
			if (i < oNode->nodes.size()) {
				AddSkeleton(oNode->nodes[i], oNode->nodes[i]->parentDist, descendants[i], dist, oNode, mapping, lthreshold);
			} else {
				AddSkeleton(NULL, 0, descendants[i], dist, oNode, mapping, lthreshold);
			}
		}

		/*aNode->count++;
		//oNode->count++;
		for (int i = 0; i < aNode->nodes.size(); i++) {
			if (i < oNode->nodes.size()) {
				AddSkeleton(oNode->nodes[i], oNode->nodes[i]->parentDist, aNode->nodes[i], aNode->nodes[i]->parentDist, oNode, mapping, lthreshold);
			} else {
				AddSkeleton(NULL, 0, aNode->nodes[i], aNode->nodes[i]->parentDist, oNode, mapping, lthreshold);
			}
		}*/
	} else if (oDist < aDist) {//if oNode is closer add just skip
		//move oNode
		if (oNode->nodes.size() > 0) {
			AddSkeleton(oNode->nodes[0], oNode->nodes[0]->parentDist, aNode, aDist - oDist, oNode, mapping, lthreshold);
		} else {
			AddSkeleton(NULL, 0, aNode, aDist - oDist, oNode, mapping, lthreshold);
		}
	} else {//if aNode is closer add new node
		int idx = oNode->id;
		CVector3 dir = Normalize(oNode->point - root->point);
		//USkeletonNode* node = new USkeletonNode(idx, aNode->point, aDist, root);
		USkeletonNode* node = new USkeletonNode(idx, root->point + dir*aDist, aDist, root);
		root->ReplaceChild(oNode, node);
		node->nodes.push_back(oNode);
		oNode->parent = node;
		oNode->parentDist -= aDist;
		//move aNode
		if (aNode->nodes.size() == 0) {
			AddSkeleton(oNode, oDist - aDist, NULL, 0, node, mapping, lthreshold);
		} else {
			AddSkeleton(oNode, oDist - aDist, aNode->nodes[0], aNode->nodes[0]->parentDist, node, mapping, lthreshold);
			for (int i = 1; i < aNode->nodes.size(); i++) {
				AddSkeleton(NULL, 0, aNode->nodes[i], aNode->nodes[i]->parentDist, node, mapping, lthreshold);
			}
		}
	}
}

void RecalculateIDs(USkeletonNode* node) {
	deque<USkeletonNode*> queue;
	queue.push_back(node);

	int id = 0;
	while (!queue.empty()) {
		USkeletonNode* aNode = queue.front();
		queue.pop_front();

		aNode->id = id;
		id++;

		for (int i = 0; i < aNode->nodes.size(); i++) {
			queue.push_back(aNode->nodes[i]);
		}
	}
}

void RecalculateIDsAndExportOutput(USkeletonNode* node, vector<MatchingStruct>& output) {
	deque<USkeletonNode*> queue;
	queue.push_back(node);

	int id = 0;
	while (!queue.empty()) {
		USkeletonNode* aNode = queue.front();
		queue.pop_front();

		output.push_back(MatchingStruct(aNode->id, aNode->count, aNode->axisAngles.size()));
		aNode->id = id;
		id++;

		for (int i = 0; i < aNode->nodes.size(); i++) {
			queue.push_back(aNode->nodes[i]);
		}
	}
}

bool UniqueAxis(vector<CVector4>& axisAngles, CVector4 axis, float threshold) {
	CVector3 u(axis.i, axis.j, axis.k);

	for (int i = 0; i < axisAngles.size(); i++) {
		CVector3 v(axisAngles[i].i, axisAngles[i].j, axisAngles[i].k);
		float dotProd = fabs(Dot(u, v));
		if (dotProd > threshold) {
			return false;
		}
	}

	return true;
}

void CleanUpCount(USkeletonNode* node) {
	deque<USkeletonNode*> queue;
	queue.push_back(node);

	while (!queue.empty()) {
		USkeletonNode* aNode = queue.front();
		queue.pop_front();

		aNode->count = 0;

		for (int i = 0; i < aNode->nodes.size(); i++) {
			queue.push_back(aNode->nodes[i]);
		}
	}
}

void GetCloseDescendants(USkeletonNode* node, float threshold, vector<CVector3>& positions, vector<USkeletonNode*>& descendants, bool clear) {
	positions.clear();
	descendants.clear();

	for (int i = 0; i < node->nodes.size(); i++) {
		USkeletonNode* aNode = node->nodes[i];
		float dist = aNode->parentDist;
		while (dist < threshold && aNode->nodes.size() == 1) {
			aNode = aNode->nodes[0];
			positions.push_back(aNode->point);
			if (clear) {
				aNode->nodes.clear();
				delete aNode;
			}
		}
		descendants.push_back(aNode);
	}
}

/*void GetCloseDescendants(USkeletonNode* node, float threshold, vector<CVector3>& positions, vector<USkeletonNode*>& descendants, bool clear) {
	positions.clear();
	descendants.clear();

	deque<USkeletonNode*> queue;
	deque<float> distqueue;
	for (int i = 0; i < node->nodes.size(); i++) {
		queue.push_back(node->nodes[i]);
		distqueue.push_back(0);
	}

	while (!queue.empty()) {
		USkeletonNode* aNode = queue.front();
		float dist = distqueue.front();
		queue.pop_front();
		distqueue.pop_front();

		dist += aNode->parentDist;
		if (dist < threshold) {
			positions.push_back(aNode->point);

			for (int i = 0; i < aNode->nodes.size(); i++) {
				queue.push_back(aNode->nodes[i]);
				distqueue.push_back(dist);
			}
			if (clear) {
				aNode->nodes.clear();
				delete aNode;
			}
		} else {
			descendants.push_back(aNode);
		}
	}
}*/

void Simplify(USkeletonNode* node, float threshold) {
	vector<CVector3> positions;
	vector<USkeletonNode*> descendants;
	GetCloseDescendants(node, threshold, positions, descendants, true);

	node->point = node->point * (float)node->count;
	for (int i = 0; i < positions.size(); i++) {
		node->point = node->point + positions[i];
		node->count++;
	}
	node->point = node->point / (float)node->count;

	node->nodes.clear();
	for (int i = 0; i < descendants.size(); i++) {
		descendants[i]->parent = node;
		descendants[i]->parentDist = Length(descendants[i]->point - node->point);
		node->nodes.push_back(descendants[i]);
	}

	for (int i = 0; i < node->nodes.size(); i++) {
		Simplify(node->nodes[i], threshold);
	}
}

USkeletonNode* SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root, int fromID, int *_id) {
	bool reverse = (ge.fromId != fromID);
	USkeletonNode* croot = root;

	for (int i = 0; i < ge.positions.size(); i++) {
		int idx = reverse ? (ge.positions.size() - i - 1) : i;
		USkeletonNode* node = new USkeletonNode();
		node->id = *_id;
		node->oldID = ge.oldIDs[idx];
		(*_id)++;
		node->point = ge.positions[idx];
		node->parent = croot;
		node->parentDist = Magnitude(node->point - croot->point);

		croot->nodes.push_back(node);

		croot = node;
	}

	return croot;
}

USkeletonNode* SkeletonNodesFromEdge(GraphEdge ge, USkeletonNode* root, int fromID, int *_id, vector<NodeDist>& out) {
	bool reverse = (ge.fromId != fromID);
	USkeletonNode* croot = root;

	float dist = 0;
	for (int i = 0; i < ge.positions.size(); i++) {
		int idx = reverse ? (ge.positions.size() - i - 1) : i;
		USkeletonNode* node = new USkeletonNode();
		node->id = *_id;
		node->oldID = ge.oldIDs[idx];
		(*_id)++;
		node->point = ge.positions[idx];
		node->parent = croot;
		node->parentDist = Magnitude(node->point - croot->point);
		dist += node->parentDist;

		croot->nodes.push_back(node);

		out.push_back(NodeDist(node, dist));

		croot = node;
	}

	return croot;
}

vector<USkeletonNode*> RecreateSkeletonsWithMatching(SkeletonGraph* A, SkeletonGraph* B, vector<int>& matching, map<int, MatchingSkeletonStruct>& o_map, float threshold) {
	//find first matched node preferring branch nodes
	bool selected = false;
	int idx = -1;
	for (int i = 0; i < A->branchNodes.size(); i++) {
		if (matching[A->branchNodes[i]->id] != -1) {
			selected = true;
			idx = A->branchNodes[i]->id;
			break;
		}
	}
	if (!selected) {
		for (int i = 0; i < A->nodes.size(); i++) {
			if (matching[A->nodes[i]->id] != -1) {
				selected = true;
				idx = A->nodes[i]->id;
				break;
			}
		}
	}
	if (!selected) return vector<USkeletonNode*>(); //empty matching
	//crate new USkeletonNode
	int idA = 0;
	int idB = 0;

	USkeletonNode* sklA = new USkeletonNode(A->nodes[idx], idA);
	USkeletonNode* sklB = new USkeletonNode(B->nodes[matching[idx]], idB);

	idA++;
	idB++;

	//o_map.insert(o_map_pair(sklA->id, MatchingSkeletonStruct(sklB->id)));
	//o_map[sklA->id] = MatchingSkeletonStruct(sklB->id);
	o_map[sklA->oldID] = MatchingSkeletonStruct(sklB->oldID);
	//for each matched node form A match with B
	//unmatched nodes are just added
	set<int> matched;
	for (int i = 0; i < A->nodes[idx]->neighborhood.size(); i++) {
		int sA = A->nodes[idx]->neighborhood[i]->id;
		if (matching[sA] == -1) {
			RecreateSkeleton(A, DataStruct(sklA, idx, sA, &idA));
		} else {
			int sB = B->nodes[matching[sA]]->id;
			matched.insert(sB);
			RecreateSkeletonsWithMatching(A, DataStruct(sklA, idx, sA, &idA), B, DataStruct(sklB, matching[idx], sB, &idB), matching, o_map, threshold);
		}
	}

	for (int i = 0; i < B->nodes[matching[idx]]->neighborhood.size(); i++) {
		if (matched.find(B->nodes[matching[idx]]->neighborhood[i]->id) == matched.end()) {
			int sB = B->nodes[matching[idx]]->neighborhood[i]->id;
			RecreateSkeleton(B, DataStruct(sklB, matching[idx], sB, &idB));
		}
	}

	vector<USkeletonNode*> result;
	result.push_back(sklA);
	result.push_back(sklB);
	return result;
}

void RecreateSkeletonsWithMatching(SkeletonGraph* A, DataStruct dA, SkeletonGraph* B, DataStruct dB, vector<int>& matching, map<int, MatchingSkeletonStruct>& o_map, float threshold) {
	//create links stored in edges
	GraphEdge eA;
	A->AreNeighbors(dA.ignoreID, dA.selectID, eA);
	vector<NodeDist> outA;
	USkeletonNode* rootA = SkeletonNodesFromEdge(eA, dA.root, dA.ignoreID, dA.id, outA);

	GraphEdge eB;
	B->AreNeighbors(dB.ignoreID, dB.selectID, eB);
	vector<NodeDist> outB;
	USkeletonNode* rootB = SkeletonNodesFromEdge(eB, dB.root, dB.ignoreID, dB.id, outB);
	//add matched to the end
	USkeletonNode* sklA = new USkeletonNode(A->nodes[dA.selectID], *dA.id);
	USkeletonNode* sklB = new USkeletonNode(B->nodes[dB.selectID], *dB.id);

	(*dA.id)++;
	(*dB.id)++;

	sklA->parent = rootA;
	rootA->nodes.push_back(sklA);
	sklB->parent = rootB;
	rootB->nodes.push_back(sklB);

	float mdA = eA.dist + Magnitude(rootA->point - sklA->point);
	float mdB = eB.dist + Magnitude(rootB->point - sklB->point);
	outA.push_back(NodeDist(sklA, mdA));
	outB.push_back(NodeDist(sklB, mdB));
	//normalize distances
	for (int i = 0; i < outA.size(); i++) {
		outA[i].dist = outA[i].dist / mdA;
	}
	for (int i = 0; i < outB.size(); i++) {
		outB[i].dist = outB[i].dist / mdB;
	}

	//add to output map
	AddToMap(outA, outB, o_map, threshold);
	//proceed like before matched together, unmatched just add
	set<int> matched;
	matched.insert(dB.ignoreID);
	for (int i = 0; i < A->nodes[dA.selectID]->neighborhood.size(); i++) {
		int sA = A->nodes[dA.selectID]->neighborhood[i]->id;
		if (sA == dA.ignoreID) continue;

		if (matching[sA] == -1) {
			RecreateSkeleton(A, DataStruct(sklA, dA.selectID, sA, dA.id));
		} else {
			int sB = B->nodes[matching[sA]]->id;
			matched.insert(sB);
			RecreateSkeletonsWithMatching(A, DataStruct(sklA, dA.selectID, sA, dA.id), B, DataStruct(sklB, dB.selectID, sB, dB.id), matching, o_map, threshold);
		}
	}

	for (int i = 0; i < B->nodes[dB.selectID]->neighborhood.size(); i++) {
		if (matched.find(B->nodes[dB.selectID]->neighborhood[i]->id) == matched.end()) {
			int sB = B->nodes[dB.selectID]->neighborhood[i]->id;
			RecreateSkeleton(B, DataStruct(sklB, dB.selectID, sB, dB.id));
		}
	}
}

void RecreateSkeleton(SkeletonGraph* G, DataStruct d) {
	GraphEdge e;
	G->AreNeighbors(d.ignoreID, d.selectID, e);
	USkeletonNode* root = SkeletonNodesFromEdge(e, d.root, d.ignoreID, d.id);
	USkeletonNode* node = new USkeletonNode(G->nodes[d.selectID], *d.id);

	node->parent = root;
	root->nodes.push_back(node);

	(*d.id)++;

	for (int i = 0; i < G->nodes[d.selectID]->neighborhood.size(); i++) {
		if (G->nodes[d.selectID]->neighborhood[i]->id != d.ignoreID) {
			RecreateSkeleton(G, DataStruct(node, G->nodes[d.selectID]->id, G->nodes[d.selectID]->neighborhood[i]->id, d.id));
			//RecreateSkeleton(G, DataStruct(node, d.selectID, G->nodes[d.selectID]->neighborhood[i]->id, d.id));
		}
	}
}

void AddToMap(vector<NodeDist>& outA, vector<NodeDist>& outB, map<int, MatchingSkeletonStruct>& o_map, float threshold) {
	int idx = 0;
	for (int i = 0; i < outB.size(); i++) {
		if (i == outB.size() - 1) {
			idx = outA.size() - 1;
			o_map[outA[idx].node->oldID].matched.push_back(outB[i].node->oldID);
		} else if (outB[i].dist <= outA[idx].dist + threshold || idx == outA.size() - 1) {
			//o_map[outA[idx].node->id].matched.push_back(outB[i].node->id);
			o_map[outA[idx].node->oldID].matched.push_back(outB[i].node->oldID);
		} else {
			idx++;
			if (idx >= outA.size()) idx = outA.size() - 1;
			i--;
		}
	}
}

void MapOldIDs(USkeletonNode *root, map<int, int> &oldIDMap) {
	deque<USkeletonNode*> queue;
	queue.push_back(root);

	while (!queue.empty()) {
		USkeletonNode* node = queue.front();
		queue.pop_front();

		oldIDMap[node->id] = node->oldID;

		for (int i = 0; i < node->nodes.size(); i++) {
			queue.push_back(node->nodes[i]);
		}
	}
}
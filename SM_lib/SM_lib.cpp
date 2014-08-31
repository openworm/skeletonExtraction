#include "SM_lib.h"
#include <string>
#include "SkeletonMatchNode.h"
#include "GraphMatcher.h"
#include "USkeletonNode.h"
#include <mmath\mmath.h>

using namespace std;
using namespace mmath;
using namespace SN;

float AvarageLength(USkeletonNode* root) {
	float length = 0;
	float nodes = 0;
	vector<USkeletonNode* > stack;
	stack.push_back(root);

	while (!stack.empty()) {
		USkeletonNode* node = stack.back();
		stack.pop_back();

		length += node->parentDist;
		nodes += 1;

		for (int i = 0; i < node->nodes.size(); i++) {
			stack.push_back(node->nodes[i]);
		}
	}

	return length/nodes;
}

std::map<int, MatchingSkeletonStruct> MatchSkeletons(SkeletonNode* skl1, SkeletonNode* skl2, float threshold) {
	SkeletonMatchNode *sklA = new SkeletonMatchNode(skl1);
	SkeletonMatchNode *sklB = new SkeletonMatchNode(skl2);

	FixParents(sklA);
	sklA = PrepareForTriming(sklA);
	sklA->Trim();
	FixParents(sklB);
	sklB = PrepareForTriming(sklB);
	sklB->Trim();

	int nodesA = RecalculateIDs(sklA);
	int nodesB = RecalculateIDs(sklB);

	SkeletonGraph* A = new SkeletonGraph();
	A->CreateGraphFromSkeleton(sklA, nodesA);
	SkeletonGraph* B = new SkeletonGraph();
	B->CreateGraphFromSkeleton(sklB, nodesB);

	GraphMatcher gm;
	gm.ignore = -1;
	gm.MatchGraphs(A, B);
	gm.SortFoundMatchings();

	std::map<int, MatchingSkeletonStruct> solution;

	if (gm.bestMatchings.size() > 0) {
		vector<int> matching = gm.bestMatchings[0];
		vector<USkeletonNode*> outSkls = RecreateSkeletonsWithMatching(A, B, matching, solution, threshold);

		delete outSkls[0];
		delete outSkls[1];
	}

	delete sklA;
	delete sklB;
	delete A;
	delete B;

	return solution;
}

vector<SkeletonNode* > MatchSkeletons(vector<SkeletonNode *> skeletons, vector<MatchingStruct>& output, float thresholdPercent, float angleThreshold, float axisAngleThreshold, bool symmetric) {
	axisAngleThreshold = axisAngleThreshold * M_PI / 180.0;
	axisAngleThreshold = cos(axisAngleThreshold);

	vector<SkeletonMatchNode *> skls;
	vector<int> numOfNodes;
	for (int i = 0; i < skeletons.size(); i++) {
		SkeletonMatchNode* skl = new SkeletonMatchNode(skeletons[i]);
		FixParents(skl);
		int nodes = CountNodes(skl);
		skl = PrepareForTriming(skl);
		skl->Trim();
		skls.push_back(skl);
		numOfNodes.push_back(nodes);
	}

	vector<SkeletonGraph* > G;
	for (int i = 0; i < skls.size(); i++) {
		int nodes = RecalculateIDs(skls[i]);
		SkeletonGraph* A = new SkeletonGraph();
		A->CreateGraphFromSkeleton(skls[i], nodes);
		G.push_back(A);
	}

	int smalestID = 0;
	int snodes = 0;
	for (int i = 0; i < G.size(); i++) {
		if (i == 0 || snodes > G[i]->nodes.size() || (snodes == G[i]->nodes.size() && numOfNodes[smalestID] > numOfNodes[i])) {
			snodes = G[i]->nodes.size();
			smalestID = i;
		}
	}
	vector<vector<int> > mappings;
	vector<vector<int> > mappingsSym;
	for (int i = 0; i < G.size(); i++) {
		GraphMatcher gm;
		gm.MatchGraphs(G[smalestID], G[i]);
		gm.SortFoundMatchings();
		mappings.push_back(gm.bestMatchings[0]);
		mappingsSym.push_back(gm.bestMatchings.back());
	}

	USkeletonNode* uroot = new USkeletonNode(G[smalestID]);
	vector<USkeletonNode* > skelets;
	float threshold = AvarageLength(uroot) * thresholdPercent;
	//CleanUpCount(uroot);
	//Simplify(uroot, threshold);
	for (int i = 0; i < G.size(); i++) {
		USkeletonNode* toAdd = new USkeletonNode(G[i], uroot, mappings[i]);
		//Simplify(toAdd, threshold);
		//AddSkeleton(uroot, toAdd, uroot, mappings[i]);
		AddSkeleton(uroot, toAdd, mappings[i], threshold);
		skelets.push_back(toAdd);
		if (symmetric) {
			USkeletonNode* toAddS = new USkeletonNode(G[i], uroot, mappingsSym[i]);
			AddSkeleton(uroot, toAddS, mappingsSym[i], threshold);
			delete toAddS;
		}
	}

	vector<SkeletonNode* > result;
	//result.push_back(uroot->ToSkeletonNode());
	result.push_back(NULL);
	for (int i = 0; i < skelets.size(); i++) {
		//result.push_back(skelets[i]->ToSkeletonNode());
		AddSkeleton(skelets[i], uroot, mappings[i], threshold);
		skelets[i]->CalculateCorrespondingDoF(uroot, angleThreshold, axisAngleThreshold);
		RecalculateIDs(skelets[i]);
		result.push_back(skelets[i]->ToSkeletonNode());
	}
	RecalculateIDsAndExportOutput(uroot, output);
	result[0] = uroot->ToSkeletonNode();
	output[0].matched = skelets.size();

	for (int i = 0; i < skls.size(); i++) {
		delete skls[i];
	}
	for (int i = 0; i < G.size(); i++) {
		delete G[i];
	}
	for (int i = 0; i < skelets.size(); i++) {
		delete skelets[i];
	}
	delete uroot;

	return result;
}
#include <algorithm>
#include "GraphMatcher.h"
#include "ValueMatchingStruct.h"

GraphMatcher::GraphMatcher(void)
{
	comparator = new GraphNodeMatch();
	evaluator = new GraphMatchingEvaluator();
	own = true;
}

GraphMatcher::GraphMatcher(GraphNodeMatch* _comparator, GraphMatchingEvaluator* _evaluator) {
	comparator = _comparator;
	evaluator = _evaluator;
	own = false;
}

GraphMatcher::~GraphMatcher(void)
{
	if (own) {
		delete comparator;
		delete evaluator;
	}
}

void GraphMatcher::MatchGraphs(SkeletonGraph* match, SkeletonGraph* to, float _threshold) {
	if (match->nodes.size() <= to->nodes.size()) {
		A = match;
		B = to;
	} else {
		A = to;
		B = match;
	}
	threshold = _threshold;
	ignore = 0;
	GenerateMatchings();
	solutions = matchingScore.size();
}

void GraphMatcher::GenerateMatchings() {
	vector<int> matching;
	matching.reserve(A->nodes.size());
	used.clear();
	matchingScore.clear();
	bestMatchings.clear();
	Backtrack(0, matching);
}

void GraphMatcher::Backtrack(int num, vector<int>& matching) {
	if (matching.size() == A->nodes.size()) {
		FinishMatching(matching);
	} else {
		for (int numb = 0; numb <= B->nodes.size(); numb++) {
			int i = numb - 1;
			if (CanMatch(num, i, matching)) {
				matching.push_back(i);
				used.push_back(i);
				if (i == -1) ignore--;
				Backtrack(num + 1, matching);
				if (i == -1) ignore++;
				used.pop_back();
				matching.pop_back();
			}
		}
	}
}

bool GraphMatcher::CanMatch(int a, int b, vector<int>& matching) {
	if (b == -1) {
		if (ignore < 0 || ignore > 0)
			return true;
		else
			return false;
	} else {
		if (!In(b, used) && comparator->CanMatch(A, B, a, b, matching)) {
			return true;
		}
	}

	return false;
}

void GraphMatcher::FinishMatching(vector<int>& proposedMatching) {
	/*//proposed matching only matches branch nodes
	//we need to create matching for leaf nodes
	//empty matching
	vector<int> finalMatching;
	finalMatching.reserve(A->nodes.size());
	for (int i = 0; i < A->nodes.size(); i++) {
		finalMatching.push_back(-1);
	}
	//first add branch nodes
	for (int i = 0; i < proposedMatching.size(); i++) {
		finalMatching[A->branchNodes[i]->id] = proposedMatching[i];
	}
	//then add all leaf combinations
	vector<int> tempUsed = used;
	for (int i = 0; i < A->branchNodes.size(); i++) {
		for (int j = 0; j < A->branchNodes[i]->neighborhood.size(); j++) {
			SkeletonGraphNode* node = A->branchNodes[i]->neighborhood[j];
			//we need to match leafs
			if (node->neighborhood.size() == 1) {
				int idx = MatchLeaf(node->id, finalMatching[A->branchNodes[i]->id], finalMatching);
				finalMatching[node->id] = idx;
			}
		}
	}
	//for now
	float error = ValueMatching(finalMatching);
	if (bestMatchings.size() < solutions || error < matchingScore.back()) {
		//bestScore = error;
		//bestMatching = finalMatching;
		InsertOrdered(error, finalMatching);
		if (bestMatchings.size() > solutions) {
			bestMatchings.resize(solutions);
		}
	}
	//and restore used
	used = tempUsed;*/
	/*float error = 0;
	if (evaluator->EvaluateMatching(A, B, proposedMatching, error)) {
		if (bestMatchings.size() < solutions || error < matchingScore.back()) {
			InsertOrdered(error, proposedMatching);
			if (bestMatchings.size() > solutions) {
				bestMatchings.resize(solutions);
			}
		}
	}*/
	float error = 0;
	if (evaluator->EvaluateMatching(A, B, proposedMatching, error)) {
		if (bestMatchings.size() == 0 || error <= matchingScore[0] + threshold) {
			InsertOrderedAndMoveThreshold(error, proposedMatching);
		}
	}
}

void GraphMatcher::InsertOrdered(float num, vector<int>& matching) {
	if (matchingScore.size() == 0 || num >= matchingScore.back()) {
		matchingScore.push_back(num);
		bestMatchings.push_back(matching);
		return;
	}
	vector<vector<int> >::iterator it_2 = bestMatchings.begin();
	for (vector<float>::iterator it = matchingScore.begin(); it != matchingScore.end(); it++) {
		if ((*it) > num) {
			matchingScore.insert(it, num);
			bestMatchings.insert(it_2, matching);
			break;
		}

		it_2++;
	}
}

void GraphMatcher::InsertOrderedAndMoveThreshold(float num, vector<int>& matching) {
	if (matchingScore.size() == 0 || num >= matchingScore.back()) {
		matchingScore.push_back(num);
		bestMatchings.push_back(matching);
		return;
	}
	//new found is the best matching and sets a new threshold
	if (matchingScore[0] > num) {
		vector<float> tempScore;
		vector<vector<int> > tempMatching;
		tempScore.push_back(num);
		tempMatching.push_back(matching);
		for (int i = 0; i < matchingScore.size(); i++) {
			if (matchingScore[i] <= num + threshold) {
				tempScore.push_back(matchingScore[i]);
				tempMatching.push_back(bestMatchings[i]);
			} else {
				break;
			}
		}
		matchingScore = tempScore;
		bestMatchings = tempMatching;
		return;
	}
	vector<vector<int> >::iterator it_2 = bestMatchings.begin();
	for (vector<float>::iterator it = matchingScore.begin(); it != matchingScore.end(); it++) {
		if ((*it) > num) {
			matchingScore.insert(it, num);
			bestMatchings.insert(it_2, matching);
			break;
		}

		it_2++;
	}
}

//get ID in B for node from A
int GraphMatcher::MatchLeaf(int a, int b, vector<int>& matching) {
	for (int i = 0; i < B->nodes[b]->neighborhood.size(); i++) {
		SkeletonGraphNode* node = B->nodes[b]->neighborhood[i];
		//node was not used
		if (!In(node->id, used)) {
			used.push_back(node->id);
			return node->id;
		}
	}

	return -1;
}

bool GraphMatcher::ValueMatching(vector<int>& matching, float& error) {
	error = 0;

	for (int i = 0; i < A->nodes.size(); i++) {
		SkeletonGraphNode* node = A->nodes[i];
		if (matching[i] == -1) {
			//if the node is not mapped add all his neighbors as error
			for (int j = 0; j < node->edges.size(); j++) {
				error += node->edges[j].nodes;
			}
		} else {
			//error are only the missing ones
			//check if neighbors are neighbors in matching and add error
			for (int j = 0; j < node->neighborhood.size(); j++) {
				int id = node->neighborhood[j]->id;
				if (matching[id] == -1) {
					//penalize for missing neighbor if needed (missing node is penalized anyway)
				} else {
					GraphEdge edge;
					if (B->AreNeighbors(matching[node->id], matching[id], edge)) {
						error += abs(edge.nodes - node->edges[j].nodes);
					} else {//wrong topology == invalid matching
						return false;
					}
				}
			}
		}
	}

	return true;
}

void GraphMatcher::SortFoundMatchings() {
	vector<ValueMatchingStruct> toSort;
	for (int i = 0; i < matchingScore.size(); i++) {
		float value = EvaluateForSorting(bestMatchings[i]);
		toSort.push_back(ValueMatchingStruct(value, bestMatchings[i]));
	}

	sort(toSort.begin(), toSort.end());

	matchingScore.clear();
	bestMatchings.clear();
	for (int i = 0; i < toSort.size(); i++) {
		matchingScore.push_back(toSort[i].value);
		bestMatchings.push_back(toSort[i].matching);
	}
}

float GraphMatcher::EvaluateForSorting(vector<int>& matching) {
	float dist = 0;

	for (int i = 0; i < A->nodes.size(); i++) {
		//this is leaf so we check distance
		if (A->nodes[i]->neighborhood.size() == 1) {
			if (matching[i] != -1) {
				dist += Magnitude(A->nodes[i]->point - B->nodes[matching[i]]->point);
			}
		}
	}

	return dist;
}

bool In(int num, vector<int> array) {
	for (int i = 0; i < array.size(); i++) {
		if (num == array[i]) return true;
	}

	return false;
}

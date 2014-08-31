#pragma once
#include "SkeletonGraph.h"

class GraphNodeMatch
{
public:
	GraphNodeMatch(void);
	~GraphNodeMatch(void);

	bool CanMatch(SkeletonGraph* A, SkeletonGraph* B, int a, int b, vector<int>& matching) {
		bool good = true;
		//if we are matching leaf or both have same number of neighbors
		good &= (A->nodes[a]->neighborhood.size() == 1 || A->nodes[a]->neighborhood.size() <= B->nodes[b]->neighborhood.size());
		//check if each set neighbor is neighbor in the other graph too
		for (int i = 0; i < A->nodes[a]->neighborhood.size(); i++) {
			SkeletonGraphNode* neighbor = A->nodes[a]->neighborhood[i];
			//neighbor was already set
			if (neighbor->id < a) {
				//neighbor was set to a valid node
				if (matching[neighbor->id] != -1) {
					good &= B->AreNeighbors(b, matching[neighbor->id]);
				}
			}
		}

		return good;
	}
};


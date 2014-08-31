#pragma once
#include "SkeletonGraph.h"

class GraphMatchingEvaluator
{
public:
	GraphMatchingEvaluator(void);
	~GraphMatchingEvaluator(void);

	bool EvaluateMatching(SkeletonGraph* A, SkeletonGraph* B, vector<int>& matching, float& error) {
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
};


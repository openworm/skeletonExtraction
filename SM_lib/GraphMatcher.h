#pragma once
#include "SkeletonGraph.h"
#include "GraphNodeMatch.h"
#include "GraphMatchingEvaluator.h"

class GraphMatcher
{
private:
	bool own;
public:
	SkeletonGraph* A;
	SkeletonGraph* B;
	float threshold;
	int solutions;
	int ignore;
	GraphNodeMatch* comparator;
	GraphMatchingEvaluator* evaluator;

	vector<float> matchingScore;
	vector<vector<int> > bestMatchings;
	vector<int> used;
public:
	GraphMatcher(void);
	GraphMatcher(GraphNodeMatch* _comparator, GraphMatchingEvaluator* _evaluator);
	~GraphMatcher(void);
	
	void MatchGraphs(SkeletonGraph* match, SkeletonGraph* to, float _threshold = 0);
	void GenerateMatchings();
	void Backtrack(int num, vector<int>& matching);
	bool CanMatch(int a, int b, vector<int>& matching);
	int MatchLeaf(int a, int b, vector<int>& matching);
	void FinishMatching(vector<int>& proposedMatching);
	bool ValueMatching(vector<int>& matching, float& error);
	void InsertOrdered(float num, vector<int>& matching);
	void InsertOrderedAndMoveThreshold(float num, vector<int>& matching);
	void SortFoundMatchings();
	float EvaluateForSorting(vector<int>& matching);
};

bool In(int num, vector<int> array);


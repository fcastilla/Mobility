#pragma once

#include "gurobi_c++.h"
#include "Data.h"
#include "Variable.h"
#include "Constraint.h"

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

using namespace std;

class SubproblemSolver;
class Node;
class Route;

class Solver
{
public:
	Solver(ProblemData *d);
	~Solver();

	//Model object and enviroment
	GRBEnv *env;
	GRBModel *model;

	//Variables
	VariableHash vHash;

	//Constraints
	ConstraintHash cHash;
	
	int solve();
private:
	ProblemData *data;	
	SubproblemSolver *spSolver;
	double bigM;

	//Global 
	double zInc;

	//Statistics
	int routeCounter;
	int totalNodes;
	int exploredNodes;
	Node* bestSolution;

	//Methods
	void buildProblemNetwork();
	void buildInitialModel();
	void buildDWM();
	void collapseSubproblemVertices();
	
	int BaP(Node *node);
	int getFeasibleSolution();
	int solveLPByColumnGeneration(Node *node);
};

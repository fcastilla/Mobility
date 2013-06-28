#pragma once

#include "gurobi_c++.h"
#include "Data.h"
#include "Variable.h"
#include "Constraint.h"
#include "Solution.h"
#include "GlobalParameters.h"

#include <sstream>
#include <time.h>
#include <set>

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
	GlobalParameters *parameters;
	SubproblemSolver *spSolver;
	set<Solution*> solutions;

	stringstream separator;

	double bigM;
	int numDuals;

	//Global 
	double ZInc;
	double gap;
	bool isInt;

	//Statistics Root node	
	double rootLB;
	double rootTime;
	double rootRoutes;

	//General statistics
	int numVars;
	double mipLB;
	int totalRoutes;
	int totalNodes;
	double totalTime;
	clock_t tStart, tEnd;

	//Methods
	void buildProblemNetwork();
	void buildInitialModel();
	void buildDWMForExtendedFormulations();
	
	int BaP(Node *node);
	int solveLPByColumnGeneration(Node *node, int treeSize);
};

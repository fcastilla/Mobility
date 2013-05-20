#pragma once

#include "gurobi_c++.h"
#include "Data.h"
#include "Variable.h"
#include "Constraint.h"

using namespace std;

class SubproblemSolver;
class Route;

class LPSolver
{
public:
	LPSolver(ProblemData *d);
	~LPSolver();

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
	int exploredNodes;

	//Subproblem related
	vector<SubproblemSolver*> spSolvers;

	//Methods
	void buildProblemNetwork();
	void buildInitialModel();
	void fixByRC();	
	void collapseSubproblemVertices();
	
	int BaP();
	int solveCurrentLPByColumnGeneration();
};

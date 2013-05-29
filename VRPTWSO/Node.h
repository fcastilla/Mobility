#pragma once

#include "gurobi_c++.h"
#include "Variable.h"
#include "Constraint.h"
#include "GlobalParameters.h"

#include <map>
#include <valarray>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

using namespace std;

class Route;
class Solution;

class Node
{
public:
	Node(int c, int e);
	Node(const Node &node);
	~Node();
	
	int solve();
	int solStatus;
	
	VariableHash vHash;
	ConstraintHash cHash;	
	
	int cDual, eDual;	
	void updatePi();
	double getMaxPiDifference();

	//Get Methods
	GRBModel *getModel(){ return model; }
	int getNodeId(){ return nodeId; }	
	double getZLP(){ return Zlp; }
	double getVarLB(Variable v);
	int getRouteCount(){ return routeCount; }
	double getDualVal(int j, int t, int e);

	double getArcReducedCost(int j, int t, int e, double cost);
	double getRouteUseReducedCost(int eqType);


	//Set Methods
	void setNodeId(int id){ nodeId = id; }
	void setModel(GRBModel *m){	model = new GRBModel(*m); }
	void setVHash(VariableHash hash){ vHash = hash; }
	void setCHash(ConstraintHash hash){ cHash = hash; }
	
	//Other Methods 
	const Variable getMostFractional();
	bool isIntegerSolution(){ return isInteger; }

	bool addColumn(Route *r);
	bool addBranchConstraint(Variable v, double rhs);
	int fixVarsByReducedCost(double maxRC);
	int cleanNode(int maxRoutes);
	void printSolution();

	double verifyRouteCost(Route *r);
	Solution *getSolution();

private:
	GRBModel *model;
	GlobalParameters *parameters;
	Solution *solution;
		
	int nodeId;
	double Zlp;
	bool isInteger;
	int routeCount;

	void updateVariables(int status);
	
	//dual stabilization	
	double alpha;
	void initializePi();
	void getCurrentPi();
	void calculateAlphaPi();

	valarray<double> alphaPi_e;
	valarray<double> feasiblePi_e; //for explicit master constraints
	valarray<double> currentPi_e; //for explicit master constraints
	valarray<double> alphaPi_c;
	valarray<double> feasiblePi_c; //for convex constraints
	valarray<double> currentPi_c; //for convex constraints
};
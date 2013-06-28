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
	Node(int num);
	Node(const Node &node);
	~Node();
	
	int solve();
	int solStatus;
	
	VariableHash vHash;
	ConstraintHash cHash;	
		
	void updatePi();	
	void toggleAlpha();
	double getMaxPiDifference();

	//Get Methods
	GRBModel *getModel(){ return model; }
	int getNodeId(){ return nodeId; }	
	double getZLP(){ return Zlp; }
	int getRouteCount(){ return routeCount; }
	int getTreeLevel() const { return treeLevel; }

	double getArcReducedCost(int s, int d, int sTime, int dTime, int e, double cost);
	double getRouteUseReducedCost(int eqType);

	double getNumDuals() const { return numDuals; }
	void unrelaxModel();

	//Set Methods
	void setNodeId(int id){ nodeId = id; }
	void setTreeLevel(int level){ treeLevel = level; }
	void setModel(GRBModel *m){	model = new GRBModel(*m); }
	void setVHash(VariableHash hash){ vHash = hash; }
	void setCHash(ConstraintHash hash){ cHash = hash; }
	
	//Other Methods 
	const Variable getMostFractional();
	bool isIntegerSolution(){ return isInteger; }

	bool addColumns(vector<Route*> &routes, int &rCounter);
	bool addColumn(Route *r);
	bool addBranchConstraint(Variable v, double rhs);
	int fixVarsByReducedCost(double maxRC);
	int cleanNode(int maxRoutes);
	void printSolution();

	void setNumDuals(int num){ numDuals = num; }

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
	int treeLevel;

	//dual multipliers
	double alpha, validAlpha, defaultAlpha;
	int numDuals;
	void initializePi();
	void calculateAlphaPi();
	void getDualMultipliers();

	/*vector<valarray<double>> currentPi;
	vector<valarray<double>> feasiblePi;
	vector<valarray<double*/

	valarray<double> currentPi;
	valarray<double> alphaPi;
	valarray<double> feasiblePi;

	void updateVariables(int status);

};
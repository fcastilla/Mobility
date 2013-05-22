#pragma once

#include "gurobi_c++.h"
#include "Variable.h"
#include "Constraint.h"

using namespace std;

class Route;

class Node
{
public:
	Node() : Zlp(1e13), nodeId(-1){};
	Node(const Node &node);
	~Node();
	
	int solve();
	int solStatus;
	
	VariableHash vHash;
	ConstraintHash cHash;

	//Get Methods
	int getNodeId(){ return nodeId; }	
	double getZLP(){ return Zlp; }
	double getVarLB(const Variable &v);
	double getArcReducedCost(const Variable &v);
	double getRouteReducedCost(int eqType);

	//Set Methods
	void setModel(GRBModel *m){ model = m; }
	void setVHash(VariableHash hash){ vHash = hash; }
	void setCHash(ConstraintHash hash){ cHash = hash; }
	
	//Other Methods 
	const Variable &getMostFractional();
	bool isIntegerSolution(){ return isInteger; }

	bool addColumn(Route *r);
	bool addBranchConstraint(const Variable &v, double rhs);
	int fixVarsByReducedCost(double maxRC);
	int cleanNode(int maxRoutes);
	void printSolution();

	double verifyRouteCost(Route *r);

private:
	GRBModel *model;
	vector<Route*> routes;
		
	int nodeId;
	double Zlp;
	bool isInteger;

	void updateVariables();
};
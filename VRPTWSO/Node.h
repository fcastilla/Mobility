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

	//Set Methods
	void setModel(GRBModel *m){ model = m; }
	void setVHash(VariableHash hash){ vHash = hash; }
	void setCHash(ConstraintHash hash){ cHash = hash; }
	
	//Other Methods 
	const Variable &getMostFractional();
	double getZLP(){ return Zlp; }
	bool isIntegerSolution(){ return isInteger; }

	bool addColumn(Route *r);
	bool addBranchConstraint(const Variable &v, double rhs);
	int fixVarsByReducedCost(double maxRC);
	int cleanNode(int maxRoutes);

private:
	GRBModel *model;
	vector<Route*> routes;
	VariableHash vHash;
	ConstraintHash cHash;
		
	int nodeId;
	double Zlp;
	bool isInteger;

	void updateVariables();
};
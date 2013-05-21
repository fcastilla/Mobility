#pragma once

#include "gurobi_c++.h"
#include "Variable.h"
#include "Constraint.h"

class Node
{
public:
	Node(){};
	Node(const Node &node);
	~Node();



private:
	GRBModel *model;


}
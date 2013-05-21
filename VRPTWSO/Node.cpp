#include "Node.h"
#include "Bucket.h"
#include "Route.h"

#include <cmath>
#include <algorithm>
#include <vector>

Node::Node(const Node &other) : vHash(other.vHash), cHash(other.cHash), 
	routes(other.routes), Zlp(1e13), nodeId(-1), solStatus(GRB_LOADED)
{
	model = new GRBModel(*other.model);
}

int Node::solve()
{
	model->optimize();
	solStatus = model->get(GRB_IntAttr_Status);
	
	if(solStatus != GRB_OPTIMAL)
		return solStatus;

	Zlp = model->get(GRB_DoubleAttr_ObjVal);
	updateVariables();
	return solStatus;
}

void Node::updateVariables()
{
	if(solStatus != GRB_OPTIMAL) return;

	isInteger = true;

	double val;
	GRBVar var;
	Variable v;
	VariableHash::iterator vit = vHash.begin();

	for(; vit != vHash.end(); vit++){
		v = vit->first;
		var = model->getVarByName(v.toString());

		if(var.get(GRB_IntAttr_VBasis) != GRB_BASIC)
			v.rank ++;

		val = var.get(GRB_DoubleAttr_X);
		v.setValue(val);
		v.fractionality = abs(v.getValue() - 0.5);

		if(floor(val) < val) isInteger = false;
	}
}

const Variable& Node::getMostFractional()
{
	Variable mostFractional, v;
	VariableHash::iterator vit = vHash.begin();
	mostFractional.fractionality = 1;
	for(; vit != vHash.end(); vit++){
		v = vit->first;
		if(v.fractionality < mostFractional.fractionality)
			mostFractional = v;
	}

	return mostFractional;
}

bool Node::addBranchConstraint(const Variable& v, double rhs)
{
	Variable myVar = v;
	GRBVar var = model->getVarByName(myVar.toString());
	GRBLinExpr expr = 0;
	expr += var;

	model->addConstr(expr == rhs, "Branch_" + myVar.toString());
	model->update();
}

bool Node::addColumn(Route *route)
{	
	Variable v;

	Constraint c1, c2;
	ConstraintHash::iterator cit1, cit2;

	int routeNumber = route->getRouteNumber();
	int eqType = route->getEquipmentType();

	//Create lambda variable
	v.reset();
	v.setType(V_LAMBDA);
	v.setRouteNum(routeNumber);
	v.setEquipmentTipe(eqType);

	if(vHash.find(v) != vHash.end()){
		std::cout << "Adding Column " << v.toString() << ": Lambda variable already existed! " << std::endl;
		return false;
	}

	vHash[v] = true;
	GRBVar lambda = model->addVar(0.0,1.0,0.0,GRB_CONTINUOUS,v.toString());
	model->update(); //TODO: verify if update is needed at this point.

	//Add column (Card constraints)
	c1.reset();
	c1.setType(C_CARD);
	c1.setEquipmentType(eqType);

	if(cHash.find(c1) == cHash.end()){
		std::cout << "Adding Column " << v.toString() << ": Cardinality constraint didn't exist." << std::endl;
		return false;
	}
	GRBConstr cardConstr = model->getConstrByName(c1.toString());
	model->chgCoeff(cardConstr,lambda,1.0);

	//Add column (Explicit Master constraints)
	Edge *myEdge;
	GRBConstr explicitConstr;
	vector<Edge*>::iterator eit = route->edges.begin();
	for(; eit != route->edges.end(); eit++){
		myEdge = (*eit);

		//Explicit constraints
		c2.reset();
		c2.setType(C_EXPLICIT);
		c2.setStartJob(myEdge->getStartJob());
		c2.setEndJob(myEdge->getEndJob());
		c2.setTime(myEdge->getTime());
		c2.setEquipmentType(eqType);

		if(cHash.find(c2) == cHash.end()){
			std::cout << "Adding Column " << v.toString() << ": Explicit master constraint didn't exist." << std::endl;
			return false;
		}

		explicitConstr = model->getConstrByName(c2.toString());
		model->chgCoeff(explicitConstr, lambda, -1.0);
	}

	//Update the model to include new column
	model->update();
	return true;
}

int Node::fixVarsByReducedCost(double maxRC)
{
	double epsilon = 0.000001;
	int deletedVars = 0;
	double rc = 0.0;

	maxRC += epsilon;
	
	GRBVar var;
	Variable v;
	VariableHash::iterator vit;

	vit = vHash.begin();
	for(; vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getType() == V_X || v.getType() == V_Y){
			var = model->getVarByName(v.toString());
			rc = var.get(GRB_DoubleAttr_RC);
			if(rc >= maxRC){
				model->remove(var);
				vHash.erase(vit);
				deletedVars++;
			}
		}
	}

	return deletedVars;
}

int Node::cleanNode(int maxRoutes)
{
	int cont = 0;
	vector<Variable> lambdas;
	vector<Variable>::iterator it;

	GRBVar lambda;
	Variable v;
	VariableHash::iterator vit = vHash.begin();

	for(; vit != vHash.end(); vit++){
		v = vit->first;

		if(v.getType() == V_LAMBDA){
			lambda = model->getVarByName(v.toString());
			if(lambda.get(GRB_IntAttr_VBasis) == GRB_BASIC){
				cont++;
			}else{
				lambdas.push_back(v);
			}
		}
	}

	sort(lambdas.begin(),lambdas.end(),varRankComparator);

	it = lambdas.begin();
	for(; it != lambdas.end(); it++){
		v = (*it);

		if(cont < maxRoutes){
			cont++;
		}else{
			vit = vHash.find(v);
			vHash.erase(vit);			
			lambda = model->getVarByName(v.toString());
			model->remove(lambda);
			cont++;
		}
	}

	return cont;
}

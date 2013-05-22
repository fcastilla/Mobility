#include "Node.h"
#include "Bucket.h"
#include "Route.h"

#include <map>
#include <hash_map>
#include <cmath>
#include <algorithm>
#include <vector>

Node::Node(const Node &other) : vHash(other.vHash), cHash(other.cHash), 
	routes(other.routes), Zlp(1e13), nodeId(-1), solStatus(GRB_LOADED)
{
	model = new GRBModel(*other.model);
}

Node::~Node()
{
	vHash.clear();
	cHash.clear();
	routes.clear();
	delete model;
}

int Node::solve()
{
	model->write("modelo.lp");
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
			vit->first.increaseRank();

		val = var.get(GRB_DoubleAttr_X);
		vit->first.setValue(val);
		vit->first.setFractionality(fabs(v.getValue() - 0.5));

		if(floor(val) < val) isInteger = false;
	}
}

const Variable& Node::getMostFractional()
{
	Variable mostFractional, v;
	VariableHash::iterator vit = vHash.begin();
	for(; vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getFractionality() < mostFractional.getFractionality())
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

	return true;
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
	GRBVar lambda = model->addVar(0.0,GRB_INFINITY,0.0,GRB_CONTINUOUS,v.toString());
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

	model->update();
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

	VariableRankComparator varRankComparator;
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

	model->update();
	return cont;
}

double Node::getVarLB(const Variable &v)
{
	GRBVar myVar;
	VariableHash::iterator vit = vHash.find(v);
	if(vit != vHash.end()){
		Variable var = vit->first;
		myVar = model->getVarByName(var.toString());
		return myVar.get(GRB_DoubleAttr_LB);
	}

	return -1;
}

double Node::getArcReducedCost(const Variable &v)
{
	Variable var;
	VariableHash::iterator vit = vHash.find(v);
	if(vit != vHash.end()){ //Variable exist in current model
		var = vit->first;
		GRBVar myVar = model->getVarByName(var.toString());
		double ub = myVar.get(GRB_DoubleAttr_UB);
		if(ub == 0){ //se a variavel esta fixada a 0
			return 1e13;
		}
	}
	
	Constraint c;
	c.setType(C_EXPLICIT);
	c.setStartJob(v.getStartJob());
	c.setEndJob(v.getEndJob());
	c.setTime(v.getTime());
	c.setEquipmentType(v.getEquipmentType());
	
	ConstraintHash::iterator cit = cHash.find(c);
	if(cit != cHash.end()){ //constraint exist
		GRBConstr myConstr = model->getConstrByName(c.toString());
		return myConstr.get(GRB_DoubleAttr_Pi) * -1; //return shadow price
	}

	return 1e13;
}

double Node::getRouteReducedCost(int eqType)
{
	Constraint c;
	c.setType(C_CARD);
	c.setEquipmentType(eqType);

	ConstraintHash::iterator cit = cHash.find(c);
	if(cit != cHash.end()){
		GRBConstr myConstr = model->getConstrByName(c.toString());
		return myConstr.get(GRB_DoubleAttr_Pi); //return shadow price
	}

	return 1e13;
}

void Node::printSolution()
{
	Variable v;
	
	cout << "//------------------------------------------------//" << endl;
	cout << "//----------------NEW SOLUTION--------------------//" << endl;
	cout << "//------------------------------------------------//" << endl;
	
	cout << "Node id: " << nodeId << " - ObjValue: " << Zlp << endl;

	VariableHash::iterator vit = vHash.begin();
	for(; vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getValue() > 0.00001)
			cout << v.toString() << " = " << v.getValue() << endl;
	}
	
	cout << "//------------------------------------------------//" << endl;
}

double Node::verifyRouteCost(Route *r)
{
	double cost = 0;

	Variable v;
	Edge *e;
	vector<Edge*>::iterator eit = r->edges.begin();
	for(; eit != r->edges.end(); eit++){
		e = (*eit);
		v.reset();
		v.setType(V_X);
		v.setStartJob(e->getStartJob());
		v.setEndJob(e->getEndJob());
		v.setTime(e->getTime());
		v.setEquipmentTipe(r->getEquipmentType());

		cost += getArcReducedCost(v);
	}

	return cost;
}

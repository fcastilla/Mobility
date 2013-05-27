#include "Node.h"
#include "Bucket.h"
#include "Route.h"
#include "Solution.h"

#include <map>
#include <hash_map>
#include <cmath>
#include <algorithm>
#include <vector>

Node::Node(int c, int e) : Zlp(1e13), nodeId(-1), routeCount(0)
{
	parameters = GlobalParameters::getInstance();
	solution = nullptr;
	cDual = c;
	eDual = e;
	initializePi();
}

Node::Node(const Node &other) : vHash(other.vHash), cHash(other.cHash), Zlp(1e13), 
	nodeId(-1), solStatus(GRB_LOADED), routeCount(0)
{
	model = new GRBModel(*other.model);
	parameters = GlobalParameters::getInstance();
	solution = nullptr;
	cDual = other.cDual;
	eDual = other.eDual;
	initializePi();
}

Node::~Node()
{
	vHash.clear();
	cHash.clear();
	delete model;
}

int Node::solve()
{
	//model->write("modelo.lp");
	model->optimize();
	solStatus = model->get(GRB_IntAttr_Status);
	
	if(solStatus != GRB_OPTIMAL && solStatus != GRB_TIME_LIMIT && solStatus != GRB_SOLUTION_LIMIT)
		return solStatus;

	Zlp = model->get(GRB_DoubleAttr_ObjVal);
	updateVariables(solStatus);
	if(parameters->useDualStabilization()){
		getCurrentPi();
		calculateAlphaPi();
	}
	return solStatus;
}

void Node::initializePi()
{
	alpha = 0.05;

	alphaPi_c = valarray<double>(cDual);
	feasiblePi_c = valarray<double>(cDual);
	currentPi_c = valarray<double>(eDual);

	alphaPi_e = valarray<double>(eDual);
	currentPi_e = valarray<double>(eDual);
	feasiblePi_e = valarray<double>(eDual);
}

void Node::getCurrentPi()
{
	double pi;

	GRBConstr myConstr;

	Constraint c;
	ConstraintHash::iterator cit = cHash.begin();

	while(cit != cHash.end()){
		c = cit->first;
		if(c.getType() == C_CARD){
			myConstr = model->getConstrByName(c.toString());			
			pi = myConstr.get(GRB_DoubleAttr_Pi); 
			currentPi_c[c.getId()] = pi;
		}else if(c.getType() == C_EXPLICIT){			
			myConstr = model->getConstrByName(c.toString());			
			pi = myConstr.get(GRB_DoubleAttr_Pi); 
			currentPi_e[c.getId()] = pi;
		}
		cit++;
	}
}

void Node::calculateAlphaPi()
{
	double alphaComp = 1 - alpha;

	alphaPi_c = (alphaComp * feasiblePi_c) + (alpha * currentPi_c);
	alphaPi_e = (alphaComp * feasiblePi_e) + (alpha * currentPi_e);
}

void Node::updatePi()
{
	feasiblePi_c = alphaPi_c;
	feasiblePi_e = alphaPi_e;
}

double Node::getMaxPiDifference()
{
	double maxDiff;
	valarray<double> diff1 = abs(feasiblePi_e - currentPi_e);
	valarray<double> diff2 = abs(feasiblePi_c - currentPi_c);

	maxDiff = max(diff1.max(),diff2.max());

	return maxDiff;
}

void Node::updateVariables(int status)
{
	isInteger = true;

	double val, rc;

	GRBVar var;
	VariableHash::iterator vit = vHash.begin();
	VariableHash::iterator evit = vHash.end();

	while(vit != evit){
		var = model->getVarByName(vit->first.toString());

		if(status == GRB_OPTIMAL && var.get(GRB_DoubleAttr_X) != 0){
			vit->first.increaseRank();
		}		

		if(!model->get(GRB_IntAttr_IsMIP)){
			rc = var.get(GRB_DoubleAttr_RC);			
			vit->first.setReducedCost(rc);
		}

		val = var.get(GRB_DoubleAttr_X);
		vit->first.setValue(val);
		vit->first.setFractionality(fabs(val - 0.5));

		if(val - floor(val) > parameters->getEpsilon()){
			isInteger = false;
		}

		vit++;
	}
}

const Variable Node::getMostFractional()
{	
	vector<Variable> yFractionalVars = vector<Variable>();
	vector<Variable> xFractionalVars = vector<Variable>();
	
	Variable v;
	VariableHash::iterator vit;

	//Try to get any fractional variables
	for(vit = vHash.begin(); vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getType() == V_Y){
			//Verify has a fractional value
			if(v.getFractionality() != 0.5)
				yFractionalVars.push_back(v);
		}else if(v.getType() == V_X){
			if(v.getFractionality() != 0.5)
				xFractionalVars.push_back(v);
		}
	}

	VariableFractionalityComparator comp;

	//See if there are any fractional Y vars
	if(yFractionalVars.size() > 0){
		std::sort(yFractionalVars.begin(),yFractionalVars.end(),comp);
		return yFractionalVars[0];
	}else if(xFractionalVars.size() > 0){
		std::sort(xFractionalVars.begin(),xFractionalVars.end(),comp);
		return xFractionalVars[0];
	}

	cout << "ATENTION: No fractional variable found!!! Still not integer solution????" << endl;
	return v;
}

bool Node::addBranchConstraint(Variable v, double rhs)
{
	if(vHash.find(v) != vHash.end()){
		GRBVar var = model->getVarByName(v.toString());
		GRBLinExpr expr = 0;
		expr += var;

		model->addConstr(expr == rhs, "Branch_" + v.toString());
		model->update();
		return true;
	}

	return false;
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
	v.setRouteNumber(routeNumber);
	v.setEquipmentTipe(eqType);

	if(vHash.find(v) != vHash.end()){
		std::cout << "Adding Column " << v.toString() << ": Lambda variable already existed! " << std::endl;
		return false;
	}

	vHash[v] = true;
	GRBVar lambda = model->addVar(0.0,GRB_INFINITY,0.0,GRB_CONTINUOUS,v.toString());
	model->update(); 

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
	while(eit != route->edges.end()){
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

		eit++;
	}

	this->routeCount++;

	//Update the model to include new column
	model->update();
	return true;
}

int Node::fixVarsByReducedCost(double maxRC)
{
	double epsilon = 1e-5;
	int deletedVars = 0;
	double rc = 0.0;
	
	GRBVar var;
	VariableHash::iterator vit = vHash.begin();

	while(vit != vHash.end()){
		if(vit->first.getType() == V_X || vit->first.getType() == V_Y){
			if(vit->first.getReducedCost() > maxRC){
				var = model->getVarByName(vit->first.toString());
				model->remove(var);
				vHash.erase(vit++);
				deletedVars++;
			}else{
				vit++;
			}
		}else{
			vit++;
		}
	}

	model->update();
	return deletedVars;
}

int Node::cleanNode(int maxRoutes)
{
	int cont = 0;
	vector<Variable> lambdas;

	GRBVar lambda;

	VariableHash::iterator vit = vHash.begin();
	VariableHash::iterator evit = vHash.end();

	for(; vit != evit; vit++){
		if(vit->first.getType() == V_LAMBDA){
			lambda = model->getVarByName(vit->first.toString());
			if(lambda.get(GRB_IntAttr_VBasis) != GRB_BASIC){
				lambdas.push_back(vit->first);
			}
		}
	}

	VariableRankComparator varRankComparator;
	sort(lambdas.begin(),lambdas.end(),varRankComparator);
		
	vector<Variable>::iterator it = lambdas.begin();

	while(it != lambdas.end()){
		if(cont < maxRoutes){
			cont++;
		}else{
			vit = vHash.find(*it);
			vHash.erase(vit);			
			lambda = model->getVarByName((*it).toString());
			model->remove(lambda);
			cont++;
		}
		it++;
	}

	lambdas.clear();
	model->update();
	return cont;
}

double Node::getVarLB(Variable v)
{
	GRBVar myVar;
	VariableHash::iterator vit = vHash.find(v);
	if(vit != vHash.end()){
		myVar = model->getVarByName(v.toString());
		return myVar.get(GRB_DoubleAttr_LB);
	}

	return -1;
}

double Node::getArcReducedCost(int sJob, int dJob, int time, int eqType)
{
	Constraint c;
	c.setType(C_EXPLICIT);
	c.setStartJob(sJob);
	c.setEndJob(dJob);
	c.setTime(time);
	c.setEquipmentType(eqType);
	
	ConstraintHash::iterator cit = cHash.find(c);
	if(cit != cHash.end()){ 
		if(parameters->useDualStabilization()){
			return alphaPi_e[cit->first.getId()];
		}else{
			GRBConstr myConstr = model->getConstrByName(c.toString());
			return myConstr.get(GRB_DoubleAttr_Pi);
		}
	}

	return 1e13;
}

double Node::getRouteUseReducedCost(int eqType)
{
	Constraint c;
	c.setType(C_CARD);
	c.setEquipmentType(eqType);

	ConstraintHash::iterator cit = cHash.find(c);
	if(cit != cHash.end()){
		if(parameters->useDualStabilization()){
			return alphaPi_e[cit->first.getId()];
		}else{
			GRBConstr myConstr = model->getConstrByName(c.toString());
			return myConstr.get(GRB_DoubleAttr_Pi);
		}
	}

	return 1e13;
}

double Node::verifyRouteCost(Route *r)
{
	double cost = 0;

	Edge *e;
	vector<Edge*>::iterator eit = r->edges.begin();
	while(eit != r->edges.end()){
		e = (*eit);
		cost += getArcReducedCost(e->getStartJob(),e->getEndJob(),e->getTime(),r->getEquipmentType());

		eit++;
	}

	return cost;
}

Solution* Node::getSolution()
{
	if(solution != nullptr)
		delete solution;

	solution = new Solution();

	//Iterate through lambda variables
	VariableHash::iterator vit = vHash.begin();
	for(; vit != vHash.end(); vit++){
		if(vit->first.getType() == V_X){
			if(vit->first.getValue() > parameters->getEpsilon()){ //lambda is in current solution
				solution->addEdge(vit->first.getStartJob(), vit->first.getEndJob(), vit->first.getTime(), vit->first.getArrivalTime(), vit->first.getEquipmentType());			
			}
		}
	}
	solution->setSolutionValue(Zlp);

	return solution;
}

void Node::printSolution()
{
	Variable v;
	
	cout << "//------------------------------------------------//" << endl;
	cout << "//----------------NEW SOLUTION--------------------//" << endl;
	cout << "//------------------------------------------------//" << endl;
	
	cout << "Node id: " << nodeId << " - ObjValue: " << Zlp << endl;

	VariableHash::iterator vit = vHash.begin();
	VariableHash::iterator eit = vHash.end();

	for(; vit != eit; vit++){
		v = vit->first;
		if(v.getType() == V_W) continue;
		if(v.getValue() > 0.00001)
			cout << v.toString() << " = " << v.getValue() << endl;
	}
	
	cout << "//------------------------------------------------//" << endl;
}

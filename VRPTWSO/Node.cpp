#include "Node.h"
#include "Bucket.h"
#include "Route.h"
#include "Solution.h"

#include <map>
#include <hash_map>
#include <cmath>
#include <algorithm>
#include <vector>

Node::Node(int c, int e) : Zlp(1e13), nodeId(-1), routeCount(0), treeLevel(0)
{
	parameters = GlobalParameters::getInstance();
	solution = nullptr;
	cDual = c;
	eDual = e;
	initializePi();
}

Node::Node(const Node &other) : vHash(other.vHash), cHash(other.cHash), Zlp(1e13), 
	nodeId(-1), solStatus(GRB_LOADED), routeCount(0), treeLevel(other.getTreeLevel()+1)
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
	model->optimize();
	solStatus = model->get(GRB_IntAttr_Status);
	
	if(solStatus != GRB_OPTIMAL && solStatus != GRB_TIME_LIMIT && solStatus != GRB_SOLUTION_LIMIT)
		return solStatus;

	Zlp = model->get(GRB_DoubleAttr_ObjVal);
	updateVariables(solStatus);
	if(parameters->useDualStabilization() && model->get(GRB_IntAttr_IsMIP) == 0){
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
		if(c.getType() == C_OVF_FLOW_INIT){
			myConstr = model->getConstrByName(c.toString());			
			pi = myConstr.get(GRB_DoubleAttr_Pi); 
			currentPi_c[c.getId()] = pi;
		}else if(c.getType() == C_SYNCH){			
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
	alphaPi_c = feasiblePi_c;
	alphaPi_e = feasiblePi_e;
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
		//vit->first.setFractionality(fabs(val - 0.5));

		if(val - floor(val) > parameters->getEpsilon()){
			isInteger = false;
		}

		vit++;
	}
}

const Variable Node::getMostFractional()
{	
	VariableFractionalityComparator comp;

	vector<Variable> yFractionalVars = vector<Variable>();
	vector<Variable> xFractionalVars = vector<Variable>();
	
	Variable v, x, mostFractional;
	VariableHash::iterator vit, vit2;

	//Get y fractional variables
	for(vit = vHash.begin(); vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getType() == V_Y){
			//Verify has a fractional value
			if(v.getFractionality() != 0.5)
				yFractionalVars.push_back(v);
		}
	}
		
	//See if there are any fractional Y vars
	if(yFractionalVars.size() > 0){
		std::sort(yFractionalVars.begin(),yFractionalVars.end(),comp);
		return yFractionalVars[0];
	}

	//IF THERE ARE NO Y FRACTIONAL VARIABLES, BRANCH ON ORIGINAL X VARS

	//Set the arc values, based on the lambdas variables
	Edge *e;
	VariableHash xHash;
	for(vit = vHash.begin(); vit != vHash.end(); vit++){
		v = vit->first;
		if(v.getType() == V_LAMBDA){
			//Iterate through lambda route edges
			vector<Edge*>::iterator itEdge = v.getRoute()->edges.begin();
			while(itEdge != v.getRoute()->edges.end()){
				e = (*itEdge);
				x.reset();
				x.setType(V_X);
				x.setStartJob(e->getStartJob());
				x.setEndJob(e->getEndJob());
				x.setTime(e->getTime());
				x.setEquipmentTipe(v.getEquipmentType());

				vit2 = xHash.find(x);
				if(vit2 != xHash.end()){
					vit2->first.setValue(vit2->first.getValue() + v.getValue());
				}else{
					x.setValue(v.getValue());
					xHash[x] = true;
				}
				itEdge++;
			}
		}
	}

	//Get most fractional arc
	for(vit = xHash.begin(); vit != xHash.end(); vit++){
		x = vit->first;
		//Verify x has a fractional value
		if(x.getFractionality() != 0.5)
			xFractionalVars.push_back(x);
	}

	//See if there are any fractional x vars
	if(xFractionalVars.size() > 0){
		std::sort(xFractionalVars.begin(),xFractionalVars.end(),comp);
		return xFractionalVars[0];
	}

	cout << "ATENTION: No fractional variable found!!! Still not integer solution????" << endl;
	return mostFractional;
}

bool Node::addBranchConstraint(Variable v, double rhs)
{
	if(v.getType() == V_Y){
		if(vHash.find(v) != vHash.end()){
			GRBVar var = model->getVarByName(v.toString());
			GRBLinExpr expr = 0;
			expr += var;

			model->addConstr(expr == rhs, "Branch_" + v.toString());
			model->update();
			return true;
		}
	}else if(v.getType() == V_X){
		GRBVar var;
		GRBLinExpr expr = 0;
		//Iterate through lambdas and pick the ones which route pass through x
		Variable lambda;
		VariableHash::iterator vit = vHash.begin();
		for(; vit != vHash.end(); vit++){
			if(vit->first.getType() == V_LAMBDA){
				lambda = vit->first;
				if(lambda.getEquipmentType() != v.getEquipmentType()) continue; //both vars must be of the same equipment type.
				if(lambda.getRoute()->findArc(v.getStartJob(),v.getEndJob(),v.getTime())){
					var = model->getVarByName(lambda.toString());
					expr += var;
				}					
			}
		}
		model->addConstr(expr == rhs, "Branch_" + v.toString());
		model->update();
		return true;
	}else if(v.getType() == V_ERROR){
		return false;
	}
	
	return false;
}

bool Node::addColumns(vector<Route*> &routes, int &rCounter)
{
	Route *myRoute;
	vector<Route*>::iterator rit, eit;

	//Verify Routes
	rit = routes.begin();
	eit = routes.end();
	for(; rit != eit; rit++){
		myRoute = (*rit);
		//Verify
		verifyRouteCost(myRoute);
	}

	//Add columns	
	rit = routes.begin();
	eit = routes.end();
	for(; rit != eit; rit++){
		myRoute = (*rit);
		myRoute->setRouteNumber(rCounter++);
		//cout << myRoute->toString() << endl;

		if(!addColumn(myRoute)){
			cout << "Error: column " << myRoute->getRouteNumber() << "," 
				<< myRoute->getEquipmentType() << " already existed in node " << nodeId << endl;
			getchar();
			exit(0);
		}
	}
	
	return true;
}

bool Node::addColumn(Route *route)
{	
	Variable v;
	VariableHash::iterator vit;

	Constraint c, c1, c2;
	ConstraintHash::iterator cit1, cit2;

	GRBConstr cons;
	GRBVar var;

	int routeNumber = route->getRouteNumber();
	int eqType = route->getEquipmentType();
	int s,d,t,a;

	//cout << "Adding route " << routeNumber << endl;

	//Iterate through route arcs and create variables
	int contVar = 0;
	Edge *myEdge;
	vector<Edge*>::iterator itEdge = route->edges.begin();
	while(itEdge != route->edges.end()){
		myEdge = (*itEdge);
		s = myEdge->getStartJob();
		d = myEdge->getEndJob();
		t = myEdge->getTime();

		if(s == d){ //waiting arc
			//create w variable
			v.reset();
			v.setType(V_W);
			v.setStartJob(s);
			v.setTime(t);
			v.setEquipmentTipe(eqType);

			if(vHash.find(v) == vHash.end()){
				vHash[v] = true;
				model->addVar(0.0,1.0,0.0,GRB_CONTINUOUS,v.toString());
				contVar++;
			}
		}else{ //transition arc
			//create x variable
			v.reset();
			v.setType(V_X);
			v.setStartJob(s);
			v.setEndJob(d);
			v.setTime(t);
			v.setEquipmentTipe(eqType);

			if(vHash.find(v) == vHash.end()){
				vHash[v] = true;
				model->addVar(0.0,1.0,myEdge->getCost(),GRB_CONTINUOUS,v.toString());
				contVar++;
			}
		}	
		itEdge++;
	}	

	//if(contVar == 0) goto error;

	model->update();

	////Create rows
	//itEdge = route->edges.begin();
	//while(itEdge != route->edges.end()){
	//	myEdge = (*itEdge);
	//	s = myEdge->getStartJob();
	//	d = myEdge->getEndJob();
	//	t = myEdge->getTime();

	//	if(s!= 0){
	//		c.reset();
	//		c.setType(C_OVF_FLOW);
	//		c.setStartJob(s);
	//		c.setTime(t);
	//		c.setEquipmentType(eqType);

	//		if(cHash.find(c) == cHash.end()){
	//			GRBLinExpr expr = 0;
	//			cHash[c] = true;
	//			model->addConstr(expr, GRB_EQUAL, 0.0, c.toString());
	//		}
	//	}
	//	itEdge++;
	//}
	//model->update();
	//model->write("Test.lp");

	//Add new variables to constraints
	//x vars
	itEdge = route->edges.begin();
	while(itEdge != route->edges.end()){
		myEdge = (*itEdge);
		s = myEdge->getStartJob();
		d = myEdge->getEndJob();
		t = myEdge->getTime();
		a = myEdge->getArriveTime();

		if(s != d){
			//Get Variable
			v.reset();
			v.setType(V_X);
			v.setStartJob(s);
			v.setEndJob(d);
			v.setTime(t);
			v.setEquipmentTipe(eqType);

			if(vHash.find(v) == vHash.end()) goto error;

			var = model->getVarByName(v.toString());

			//Synchronization constraints
			if(s != 0){
				c.reset();
				c.setType(C_SYNCH);
				c.setStartJob(s);
				c.setTime(t);
				c.setEquipmentType(eqType);

				if(cHash.find(c) == cHash.end()) goto error;

				cons = model->getConstrByName(c.toString());
				model->chgCoeff(cons,var,1.0);
			}

			//Flow init constraints
			if(s == 0 && t == 0){
				c.reset();
				c.setType(C_OVF_FLOW_INIT);
				c.setEquipmentType(eqType);

				if(cHash.find(c) == cHash.end()) goto error;

				cons = model->getConstrByName(c.toString());
				model->chgCoeff(cons,var,1.0);
			}

			//FLOW SOURCE
			if(s != 0){
				c.reset();
				c.setType(C_OVF_FLOW);
				c.setStartJob(s);
				c.setTime(t);
				c.setEquipmentType(eqType);

				if(cHash.find(c) == cHash.end()) goto error;
					
				cons = model->getConstrByName(c.toString());
				model->chgCoeff(cons,var,1.0);
			}

			//FLOW DESTINATION
			if(d != 0){
				c.reset();
				c.setType(C_OVF_FLOW);
				c.setStartJob(d);
				c.setTime(a);
				c.setEquipmentType(eqType);

				if(cHash.find(c) == cHash.end()) goto error;
					
				cons = model->getConstrByName(c.toString());
				model->chgCoeff(cons,var,-1.0);
			}
		
		}else{ //wait arc
			//Get Variable
			v.reset();
			v.setType(V_W);
			v.setStartJob(s);
			v.setTime(t);
			v.setEquipmentTipe(eqType);

			if(vHash.find(v) == vHash.end()) goto error;

			var = model->getVarByName(v.toString());	

			//FLOW SOURCE
			c.reset();
			c.setType(C_OVF_FLOW);
			c.setStartJob(s);
			c.setTime(t);
			c.setEquipmentType(eqType);

			if(cHash.find(c) == cHash.end()) goto error;
					
			cons = model->getConstrByName(c.toString());
			model->chgCoeff(cons,var,1.0);
			
			//FLOW DESTINATION
			c.reset();
			c.setType(C_OVF_FLOW);
			c.setStartJob(s);
			c.setTime(t + 1);
			c.setEquipmentType(eqType);

			if(cHash.find(c) == cHash.end()) goto error;
					
			cons = model->getConstrByName(c.toString());
			model->chgCoeff(cons,var,-1.0);
		}

		itEdge++;
	}

	this->routeCount++;

	//Update the model to include new column
	model->update();
	//model->write("modelo_CGExtended.lp");
	return true;

error:
	cout << "Atention: problem adding route: " << routeNumber << endl;
	exit(0);
	return false;
}

//bool Node::addColumn(Route *route)
//{	
//	Variable v;
//
//	Constraint c1, c2;
//	ConstraintHash::iterator cit1, cit2;
//
//	int routeNumber = route->getRouteNumber();
//	int eqType = route->getEquipmentType();
//
//	//Create lambda variable
//	v.reset();
//	v.setType(V_LAMBDA);
//	v.setRouteNumber(routeNumber);
//	v.setEquipmentTipe(eqType);
//	v.setRoute(route);
//
//	if(vHash.find(v) != vHash.end()){
//		std::cout << "Adding Column " << v.toString() << ": Lambda variable already existed! " << std::endl;
//		return false;
//	}
//
//	vHash[v] = true;
//	GRBVar lambda = model->addVar(0.0,1.0,route->getCost(),GRB_CONTINUOUS,v.toString()); //adds to the objective function
//	model->update(); 
//
//	double routeRC = route->getCost();
//
//	//Add column (Conv constraints)
//	c1.reset();
//	c1.setType(C_CONV);
//	c1.setEquipmentType(eqType);
//
//	if(cHash.find(c1) == cHash.end()){
//		std::cout << "Adding Column " << v.toString() << ": Cardinality constraint didn't exist." << std::endl;
//		return false;
//	}
//	GRBConstr cardConstr = model->getConstrByName(c1.toString());
//	model->chgCoeff(cardConstr,lambda,1.0);
//
//	//Add column (Synch constraints)
//	Edge *myEdge;
//	GRBConstr convConstr;
//	vector<Edge*>::iterator eit = route->edges.begin();
//	while(eit != route->edges.end()){
//		myEdge = (*eit);
//
//		if(myEdge->getStartJob() != 0){;
//
//			//synchronization constraints
//			c2.reset();
//			c2.setType(C_SYNCH);
//			c2.setStartJob(myEdge->getStartJob());
//			c2.setTime(myEdge->getTime());
//			c2.setEquipmentType(eqType);
//
//			if(cHash.find(c2) == cHash.end()){
//				std::cout << "Adding Column " << v.toString() << ": synch constraint didn't exist." << std::endl;
//				return false;
//			}
//
//			convConstr = model->getConstrByName(c2.toString());
//			model->chgCoeff(convConstr, lambda, 1.0);
//		}
//
//		eit++;
//	}
//
//	this->routeCount++;
//
//	//Update the model to include new column
//	model->update();
//	return true;
//}

int Node::fixVarsByReducedCost(double maxRC)
{
	double epsilon = 1e-5;
	int deletedVars = 0;
	double rc = 0.0;
	
	GRBVar var;
	VariableHash::iterator vit = vHash.begin();

	while(vit != vHash.end()){
		if(vit->first.getType() == V_Y){
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

	//model->update();
	return deletedVars;
}

int Node::cleanNode(int maxRoutes)
{
	int cont = 0;
	int contDel = 0;
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
	
	Variable v;
	vector<Variable>::iterator it = lambdas.begin();

	while(it != lambdas.end()){
		if(cont < maxRoutes){
			cont++;
		}else{
			v = *it;
			vit = vHash.find(v);
			vHash.erase(vit);			
			lambda = model->getVarByName(v.toString());
			model->remove(lambda);
			contDel++;
		}
		it++;
	}

	lambdas.clear();
	model->update();
	return contDel;
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

double Node::getArcReducedCost(int j, int t, int e, double cost)
{
	Constraint c;
	//c.setType(C_SYNCH);
	c.setType(C_OVF_FLOW);
	c.setStartJob(j);
	c.setTime(t);
	c.setEquipmentType(e);	
	
	double dualVal = 0.0;
	if(j != 0){
		ConstraintHash::iterator cit = cHash.find(c);
		if(cit != cHash.end()){ 
			if(parameters->useDualStabilization()){
				dualVal = alphaPi_e[cit->first.getId()];
			}else{
				GRBConstr myConstr = model->getConstrByName(c.toString());
				dualVal = myConstr.get(GRB_DoubleAttr_Pi);
			}
		}
	}else{
		dualVal = getRouteUseReducedCost(e);
	}

	return cost - dualVal;
}

double Node::getDualVal(int j, int t, int e)
{
	double val = 0.0;

	Constraint c;
	//c.setType(C_SYNCH);
	c.setType(C_OVF_FLOW);
	c.setStartJob(j);
	c.setTime(t);
	c.setEquipmentType(e);

	if(j != 0){
		ConstraintHash::iterator cit = cHash.find(c);
		if(cit != cHash.end()){ 
			GRBConstr myConstr = model->getConstrByName(c.toString());
			val = myConstr.get(GRB_DoubleAttr_Pi);
		}
	}else{
		val = getRouteUseReducedCost(e);
	}

	return val;
}

double Node::getRouteUseReducedCost(int eqType)
{
	Constraint c;
	//c.setType(C_CONV);
	c.setType(C_OVF_FLOW_INIT);
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

double Node::verifyRouteCost(Route *route)
{
	Variable v;

	Constraint c1, c2;
	ConstraintHash::iterator cit1, cit2;

	int routeNumber = route->getRouteNumber();
	int eqType = route->getEquipmentType();

	//Create lambda variable
	double routeRC = route->getCost();

	//Add column (Conv constraints)
	c1.reset();
	//c1.setType(C_CONV);
	c1.setType(C_OVF_FLOW_INIT);
	c1.setEquipmentType(eqType);

	if(cHash.find(c1) == cHash.end()){
		std::cout << "Adding Column " << v.toString() << ": Cardinality constraint didn't exist." << std::endl;
		return false;
	}

	if(parameters->useDualStabilization()){
		routeRC -= alphaPi_c[c1.getId()];	
	}else{
		GRBConstr cardConstr = model->getConstrByName(c1.toString());
		routeRC -= cardConstr.get(GRB_DoubleAttr_Pi);
	}

	//Add column (Synch constraints)
	Edge *myEdge;
	GRBConstr convConstr;
	vector<Edge*>::iterator eit = route->edges.begin();
	while(eit != route->edges.end()){
		myEdge = (*eit);

		if(myEdge->getStartJob() != 0 && myEdge->getStartJob() != myEdge->getEndJob()){;

			//synchronization constraints
			c2.reset();
			//c2.setType(C_SYNCH);
			c2.setType(C_OVF_FLOW);
			c2.setStartJob(myEdge->getStartJob());
			c2.setTime(myEdge->getTime());
			c2.setEquipmentType(eqType);

			if(cHash.find(c2) == cHash.end()){
				std::cout << "Adding Column " << v.toString() << ": synch constraint didn't exist." << std::endl;
				return false;
			}
			
			if(parameters->useDualStabilization()){
				routeRC -= alphaPi_e[c2.getId()];	
			}else{
				convConstr = model->getConstrByName(c2.toString());
				routeRC -= convConstr.get(GRB_DoubleAttr_Pi);
			}
		}

		eit++;
	}

	if(route->getReducedCost() - routeRC > parameters->getEpsilon()){
		cout << "ATENCAO: CUSTOS REDUZIDOS NÃO BATEM" << endl;
		cout << " Custo reduzido do subproblema: " << route->getReducedCost() << endl;
		cout << " Custo reduzido verificado da rota: " << routeRC << endl;
	}

	return true;
}

Solution* Node::getSolution()
{
	if(solution != nullptr)
		delete solution;

	solution = new Solution();

	//Iterate through lambda variables
	VariableHash::iterator vit = vHash.begin();
	for(; vit != vHash.end(); vit++){
		if(vit->first.getType() == V_LAMBDA){
			if(vit->first.getValue() > parameters->getEpsilon()){ //lambda is in current solution
				solution->addRoute(vit->first.getRoute());
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
		//if(v.getType() != V_Y) continue;
		if(v.getValue() > 0.00001)
			cout << v.toString() << " = " << v.getValue() << endl;
	}
	
	cout << "//------------------------------------------------//" << endl;
}

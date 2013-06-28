#include "Node.h"
#include "Bucket.h"
#include "Route.h"
#include "Solution.h"

#include <map>
#include <hash_map>
#include <cmath>
#include <algorithm>
#include <vector>

Node::Node(int num) : Zlp(1e13), nodeId(-1), routeCount(0), treeLevel(0), numDuals(num)
{
	parameters = GlobalParameters::getInstance();
	solution = nullptr;	
	initializePi();
}

Node::Node(const Node &other) : vHash(other.vHash), cHash(other.cHash), Zlp(1e13), 
	nodeId(-1), solStatus(GRB_LOADED), routeCount(0), treeLevel(other.getTreeLevel()+1)
{
	model = new GRBModel(*other.model);
	parameters = GlobalParameters::getInstance();
	solution = nullptr;
	numDuals = other.getNumDuals();
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
	getDualMultipliers();
	//calculateAlphaPi();
	updateVariables(solStatus);
	return solStatus;
}

void Node::initializePi()
{
	defaultAlpha = 0.7;
	validAlpha = defaultAlpha;
	alpha = (parameters->useDualStabilization())? validAlpha : 1;

	alphaPi = valarray<double>(numDuals);
	feasiblePi = valarray<double>(numDuals);
	currentPi = valarray<double>(numDuals);
}

void Node::toggleAlpha()
{
	if(validAlpha == defaultAlpha)
		validAlpha = 1;
	else
		validAlpha = defaultAlpha;
	
	cout << endl << "Current alpha value: " << validAlpha;
}

void Node::getDualMultipliers()
{
	Constraint c;
	ConstraintHash::iterator cit = cHash.begin();

	double val = 0.0;
	while(cit != cHash.end()){
		c = cit->first;
		val = 0.0;
		if(c.getType() == C_SYNCH || c.getType() == C_OVF_FLOW || c.getType() == C_OVF_FLOW_INIT){
			GRBConstr myConstr = model->getConstrByName(c.toString());
			val = myConstr.get(GRB_DoubleAttr_Pi);
			//currentPi[c.getId()] = val;
			alphaPi[c.getId()] = val;
		}
		cit++;
	}
}

void Node::calculateAlphaPi(){
	double alphaComp = 1 - alpha;
	
	if(parameters->useDualStabilization())
		alphaPi = (alphaComp * feasiblePi) + (alpha * currentPi);
	else
		alphaPi = currentPi;
}

void Node::updatePi()
{
	feasiblePi = alphaPi;
}

double Node::getMaxPiDifference()
{
	double maxDiff;
	valarray<double> diff1 = abs(feasiblePi - currentPi);

	maxDiff = diff1.max();

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
	if(v.getType() == V_Y || v.getType() == V_X){
		if(vHash.find(v) != vHash.end()){
			GRBVar var = model->getVarByName(v.toString());
			GRBLinExpr expr = 0;
			expr += var;

			model->addConstr(expr == rhs, "Branch_" + v.toString());
			model->update();
			return true;
		}
	}
	
	return false;
}

bool Node::addColumns(vector<Route*> &routes, int &rCounter)
{
	Route *myRoute;
	vector<Route*>::iterator rit, eit;

	//Verify Routes
	//rit = routes.begin();
	//eit = routes.end();
	//for(; rit != eit; rit++){
	//	myRoute = (*rit);
	//	//Verify
	//	verifyRouteCost(myRoute);
	//}

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

	//Iterate through route arcs and create variables
	int contVar = 0;
	Edge *myEdge;
	vector<Edge*>::iterator itEdge = route->edges.begin();
	while(itEdge != route->edges.end()){
		myEdge = (*itEdge);
		s = myEdge->getStartJob();
		d = myEdge->getEndJob();
		t = myEdge->getTime();

		if(s != d){ //transition arc
			//create x variable
			v.reset();
			v.setType(V_X);
			v.setStartJob(s);
			v.setEndJob(d);
			v.setTime(t);
			v.setEquipmentTipe(eqType);

			/*if(vHash.find(v) != vHash.end()){
				model->getVarByName(v.toString()).set(GRB_DoubleAttr_UB, GRB_INFINITY);
			}*/

			if(vHash.find(v) == vHash.end()){
				vHash[v] = true;
				model->addVar(0.0,GRB_INFINITY,myEdge->getCost(),GRB_CONTINUOUS,v.toString());
				contVar++;
			}
		}	
		itEdge++;
	}	

	model->update();

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
		
		}

		itEdge++;
	}

	//this->routeCount++;

	//Update the model to include new column
	model->update();
	//model->write("modelo_CGExtended.lp");
	//getchar();
	return true;

error:
	cout << "Atention: problem adding route: " << routeNumber << endl;
	exit(0);
	return false;
}

int Node::fixVarsByReducedCost(double maxRC)
{
	double epsilon = 1e-5;
	int deletedVars = 0;
	double rc = 0.0;
	
	GRBVar var;
	VariableHash::iterator vit = vHash.begin();

	while(vit != vHash.end()){
		if(vit->first.getReducedCost() > maxRC){
			var = model->getVarByName(vit->first.toString());
			model->remove(var);
			vHash.erase(vit++);
			deletedVars++;
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

double Node::getArcReducedCost(int s, int d, int sTime, int dTime, int e, double cost)
{
	Constraint cSynch, cFlowIn, cFlowOut, cDepot;
	
	double rc = cost;
	if(s != 0){
		//Synch
		cSynch.reset();
		cSynch.setType(C_SYNCH);
		cSynch.setStartJob(s);
		cSynch.setTime(sTime);
		cSynch.setEquipmentType(e);

		ConstraintHash::iterator citSynch = cHash.find(cSynch);
		if(citSynch != cHash.end()){
			rc -= alphaPi[citSynch->first.getId()];
		}

		//FlowOut 
		cFlowOut.reset();		
		cFlowOut.setType(C_OVF_FLOW);
		cFlowOut.setStartJob(s);
		cFlowOut.setTime(sTime);
		cFlowOut.setEquipmentType(e);

		ConstraintHash::iterator citFlowOut = cHash.find(cFlowOut);

		if(citFlowOut != cHash.end()){
			rc -= alphaPi[citFlowOut->first.getId()];
		}

		//Flow In
		if(d != 0){ //not for depot ending
			cFlowIn.reset();		
			cFlowIn.setType(C_OVF_FLOW);
			cFlowIn.setStartJob(d);
			cFlowIn.setTime(dTime);
			cFlowIn.setEquipmentType(e);

			ConstraintHash::iterator citFlowIn = cHash.find(cFlowIn);

			if(citFlowIn != cHash.end()){
				rc += alphaPi[citFlowIn->first.getId()];
			}	
		}		
	}else{		
		//Out of depot constraint
		cDepot.reset();
		cDepot.setType(C_OVF_FLOW_INIT);
		cDepot.setEquipmentType(e);

		//FlowIn
		cFlowIn.reset();		
		cFlowIn.setType(C_OVF_FLOW);
		cFlowIn.setStartJob(d);
		cFlowIn.setTime(dTime);
		cFlowIn.setEquipmentType(e);

		ConstraintHash::iterator citDepot = cHash.find(cDepot);
		ConstraintHash::iterator citFlowIn = cHash.find(cFlowIn);

		if(citDepot != cHash.end() && citFlowIn != cHash.end()){
			rc -= alphaPi[citDepot->first.getId()];
			rc += alphaPi[citFlowIn->first.getId()];
			//rc = 0;
		}
	}
	
	return rc;
}

double Node::verifyRouteCost(Route *route)
{
	Variable v;

	Constraint cSynch, cDepot, cFlowOut, cFlowIn;
	ConstraintHash::iterator citSynch, citDepot, citFlowOut, citFlowIn;

	vector<Edge*>::iterator itEdge;
	vector<Edge*>::iterator eitEdge;

	int s, d, sTime, dTime, cost;

	int routeNumber = route->getRouteNumber();
	int eqType = route->getEquipmentType();

	Edge *myEdge;
	itEdge = route->edges.begin();
	eitEdge = route->edges.end();

	double rc = 0.0;
	while(itEdge != eitEdge){
		myEdge = *itEdge;
		s = myEdge->getStartJob();
		d = myEdge->getEndJob();
		sTime = myEdge->getTime();
		dTime = myEdge->getArriveTime();

		//Flow init constraints
		cDepot.reset();
		cDepot.setType(C_OVF_FLOW_INIT);
		cDepot.setEquipmentType(eqType);
		
		citDepot = cHash.find(cDepot);

		//Flow In
		cFlowIn.reset();
		cFlowIn.setType(C_OVF_FLOW);
		cFlowIn.setStartJob(d);
		cFlowIn.setTime(dTime);
		cFlowIn.setEquipmentType(eqType);

		citFlowIn = cHash.find(cFlowIn);

		//Synch Constraint
		cSynch.reset();
		cSynch.setType(C_SYNCH);
		cSynch.setStartJob(s);
		cSynch.setTime(sTime);
		cSynch.setEquipmentType(eqType);

		citSynch = cHash.find(cSynch);

		//Flow Out
		cFlowOut.reset();
		cFlowOut.setType(C_OVF_FLOW);
		cFlowOut.setStartJob(s);
		cFlowOut.setTime(sTime);
		cFlowOut.setEquipmentType(eqType);

		citFlowOut = cHash.find(cFlowOut);
				
		rc += myEdge->getCost();

		if(s == d){
			rc += 0;
		}else if(s == 0){
			rc -= alphaPi[citDepot->first.getId()];
			rc += alphaPi[citFlowIn->first.getId()];
		}else{
			rc -= alphaPi[citSynch->first.getId()];
			rc -= alphaPi[citFlowOut->first.getId()];
			if(d != 0)
				rc += alphaPi[citFlowIn->first.getId()];
		}

		itEdge++;
	}
		
	if(route->getReducedCost() - rc < parameters->getEpsilon()){
		return true;
	}

	cout << "WRONG REDUCED COSTS!!!" << endl;
	cout << endl << endl << "RouteRC: " << route->getReducedCost() << " | Verified RC: " << rc << endl;
	
	itEdge = route->edges.begin();
	eitEdge = route->edges.end();
	while(itEdge != eitEdge){
		myEdge = *itEdge;
		s = myEdge->getStartJob();
		d = myEdge->getEndJob();
		sTime = myEdge->getTime();
		dTime = myEdge->getArriveTime();

		v.reset();
		v.setType(V_X);
		v.setStartJob(s);
		v.setEndJob(d);
		v.setTime(sTime);
		v.setEquipmentTipe(eqType);		

		//Flow init constraints
		cDepot.reset();
		cDepot.setType(C_OVF_FLOW_INIT);
		cDepot.setEquipmentType(eqType);
		
		citDepot = cHash.find(cDepot);

		//Flow In
		cFlowIn.reset();
		cFlowIn.setType(C_OVF_FLOW);
		cFlowIn.setStartJob(d);
		cFlowIn.setTime(dTime);
		cFlowIn.setEquipmentType(eqType);

		citFlowIn = cHash.find(cFlowIn);

		//Synch Constraint
		cSynch.reset();
		cSynch.setType(C_SYNCH);
		cSynch.setStartJob(s);
		cSynch.setTime(sTime);
		cSynch.setEquipmentType(eqType);

		citSynch = cHash.find(cSynch);

		//Flow Out
		cFlowOut.reset();
		cFlowOut.setType(C_OVF_FLOW);
		cFlowOut.setStartJob(s);
		cFlowOut.setTime(sTime);
		cFlowOut.setEquipmentType(eqType);

		citFlowOut = cHash.find(cFlowOut);

		cout << endl << v.toString();	

		if(vHash.find(v) != vHash.end()){
			GRBVar myVar = model->getVarByName(v.toString());
			GRBColumn myCol = model->getCol(myVar);

			int varStat = myVar.get(GRB_IntAttr_VBasis);
			string varStatus = (varStat == 0)? "Basic" : (varStat == -1)? "Nonnasic at rootLB" : "Nonbasic at UB";
			cout << " | Status: " << varStatus;
			cout << " | Coefficients: ";

			for(int i=0; i<myCol.size(); i++){
				cout << " # " << myCol.getConstr(i).get(GRB_StringAttr_ConstrName) << " = ";
				cout << myCol.getCoeff(i);
			}
		}
			
		cout << " | Arc_" << s << "," << d << "," << sTime << "," << eqType << "_RC: " << getArcReducedCost(s,d,sTime,dTime,eqType,myEdge->getCost());			 
				
		//Print duals
		if(s == 0){ //comming from depot
			cout << " | S_" << s << "_" << d << "=" << myEdge->getCost();
			cout << " | Dual of " << cDepot.toString() << "=" << alphaPi[citDepot->first.getId()];
			cout << " | Dual of " << cFlowIn.toString() << "=" << alphaPi[citFlowIn->first.getId()];
		}else{										
			cout << " | Dual of " << cSynch.toString() << "=" << alphaPi[citSynch->first.getId()];
			cout << " | Dual of " << cFlowOut.toString() << "=" << alphaPi[citFlowOut->first.getId()];
			if(d!=0)
				cout << " | Dual of " << cFlowIn.toString() << "=" << alphaPi[citFlowIn->first.getId()];
		}
		cout << endl << endl << "----------------------------" << endl;				
		itEdge++;
	}
	printSolution();
	//model->write("RC_error.lp");
	
	getchar();
	return false;
}

void Node::unrelaxModel()
{
	Variable v;
	VariableHash::iterator vit = vHash.begin();

	while(vit != vHash.end()){
		v = vit->first;
		GRBVar var = model->getVarByName(v.toString());
		var.set(GRB_CharAttr_VType,GRB_INTEGER);
		vit++;
	}
	model->update();
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
		if(v.getType() == V_W) continue;
		if(v.getValue() > 0.00001)
			cout << v.toString() << " = " << v.getValue() << endl;
	}
	
	cout << "//------------------------------------------------//" << endl;
}

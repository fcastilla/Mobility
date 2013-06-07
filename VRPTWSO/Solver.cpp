#include "Solver.h"
#include "SubproblemSolver.h"
#include "Bucket.h"
#include "Route.h"
#include "Node.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <queue>

//#define GUROBI_TEST
#define CG_EXTENDED

string itos(int i) {stringstream s; s << i; return s.str(); }

string getFileSeparator(){
	#ifdef _WIN32
		return "\\";
	#else
		return "/";
	#endif
}

string getSpTypeName(int type){
	switch(type){
		case QROUTE:
			return "q-Route";
			break;
		case QROUTE_NOLOOP:
			return "2Cycle";
			break;
		default:
			return "N/A";
	}
}

#define METHOD_MIP = 1;

Solver::Solver(ProblemData *d, int type) : data(d), spType(type)
{
	//Get global parameters
	parameters = GlobalParameters::getInstance();

	//For printing
	separator << setw(180) << setfill('-') << "-" << setfill (' ') << endl;

	//Create Gurobi enviroment and model
	env = new GRBEnv();
	model = new GRBModel(*env);	

	//Objective function value of auxiliary variables
	bigM = parameters->getBigM();
	
	//Initialize subproblem solver vector
	SubproblemType spMethod = static_cast<SubproblemType>(type);
	spSolver = new SubproblemSolver(data, spMethod);	

	//Solutions set
	solutions = set<Solution*>();
}

Solver::~Solver()
{
	//Destroy subproblem solver
	delete spSolver;

	//Destroy variable hash map
	vHash.clear();

	//Destroy constraint hash map
	cHash.clear();
	
	//Destroy Gurobi Enviroment
	delete env;
}

int Solver::solve()
{
	int status = GRB_INPROGRESS;
	eDualVars = 0;
	cDualVars = 0;

	//Global parameters
	lb = 1e13;
	ZInc = 1e13;
	gap = 1e13;
	totalNodes = 0;
	exploredNodes = 0;
	routeCounter = 0;
	maxTreeHeight = 0;

	//Build problem graph representation
	buildProblemNetwork();

	//Build the initial model
	buildInitialModel();

	//Get a first integer feasible solution (ZInc)
	GRBEnv myEnv = model->getEnv();
	myEnv.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);

	
#ifdef GUROBI_TEST

	//myEnv.set(GRB_DoubleParam_NodeLimit,1);
	myEnv.set(GRB_DoubleParam_TimeLimit, 300);
	myEnv.set(GRB_IntParam_Method, 1);

	Node *tempNode = new Node(cDualVars, eDualVars);
	tempNode->setModel(model);
	tempNode->setVHash(vHash);
	tempNode->setCHash(cHash);

	//TESTE GUROBI
	//model->optimize();
	tempNode->solve();
	status = tempNode->getModel()->get(GRB_IntAttr_Status);
	double objVal = tempNode->getModel()->get(GRB_DoubleAttr_ObjVal);
	double bound = tempNode->getModel()->get(GRB_DoubleAttr_ObjBound);
	double expNode = tempNode->getModel()->get(GRB_DoubleAttr_NodeCount);
	double totalTime = tempNode->getModel()->get(GRB_DoubleAttr_Runtime);
	double mipGap = tempNode->getModel()->get(GRB_DoubleAttr_MIPGap);

	tempNode->printSolution();

	std::ifstream myFile("statisticsGurobi.csv");
	bool isEmpty = myFile.peek() == std::ifstream::traits_type::eof();
	myFile.close();

	string fileName = data->getFileName();
	unsigned pos = 0;

	while(pos = fileName.find(getFileSeparator()) != std::string::npos){
		fileName.erase(0, pos);
	}

	//Save statistics
	ofstream statFile;
	statFile.open("statisticsGurobi.csv", ios::out | ios::app);

	//Header
	if(isEmpty)
		statFile << "File_Name,LB,UB,Gap,Nodes,Time" << endl;
	else
		statFile << endl;

	statFile << fileName << ",";
	statFile << bound << ","; 
	statFile << objVal << ","; 
	statFile << mipGap << ",";
	statFile << expNode << ",";
	statFile << totalTime;

	//getchar();

#endif
#ifdef CG_EXTENDED
	myEnv.set(GRB_IntParam_MIPFocus, 1);
	//myEnv.set(GRB_DoubleParam_TimeLimit, 1200);

	//Disable gurobi output
	//myEnv.set(GRB_IntParam_OutputFlag,0);	
	myEnv.set(GRB_DoubleParam_TimeLimit, GRB_INFINITY);

	buildDWMForExtendedFormulations();

	Node *rootNode = new Node(cDualVars,eDualVars);
	rootNode->setModel(model);
	rootNode->setVHash(vHash);
	rootNode->setCHash(cHash);
	rootNode->setNodeId(0);
	rootNode->setTreeLevel(0);

	//At this point, root node has its own copy of the model
	delete model;
	vHash.clear();
	cHash.clear();

	//Solve DWM model by CG
	tStart = clock();
	if(parameters->solveByMIP()){
		status = BaP(rootNode);
	}else{
		status = solveLPByColumnGeneration(rootNode,0);
	}
	tEnd = clock();

	////--------------------------------------
	////SAVE OUTPUT
	////--------------------------------------	
	string fileName = data->getFileName();
	unsigned pos = 0;

	while(pos = fileName.find(getFileSeparator()) != std::string::npos){
		fileName.erase(0, pos);
	}

	//--------------------------------------
	//SOLUTION FILE
	//--------------------------------------
	ofstream solFile;
	solFile.open("solutions.txt", ios::out | ios::app);

	//Begin of new solution
	solFile << setw(80) << setfill('-') << "-" << setfill (' ') << endl;

	//Instance name	
	solFile << "#Instance: " << endl << fileName << endl;

	Solution *mySolution;
	if(parameters->solveByMIP()){		
		if(solutions.size() > 0){
			mySolution = (*solutions.begin());
			solFile << *mySolution;
		}else{
			solFile << "No solution found. Final status was: " << status << endl;
		}
	}else{
		mySolution = rootNode->getSolution();
		solFile << *mySolution;
	}
	
	//End of new solution
	solFile << setw(80) << setfill('-') << "-" << setfill (' ') << endl;
	solFile.close();
	//--------------------------------------

	//--------------------------------------
	//STATISTICS FILE	
	//--------------------------------------
	//Verify if statistics file is empty
	std::ifstream myFile("statistics.csv");
	bool isEmpty = myFile.peek() == std::ifstream::traits_type::eof();
	myFile.close();

	//Save statistics
	ofstream statFile;
	statFile.open("statistics.csv", ios::out | ios::app);

	//Header
	if(isEmpty)
		statFile << "File_Name,LB,UB,Gap,SP_Method,Exp_Nodes,Max_Tree_Height,Total_Routes,Integer,Time" << endl;
	else
		statFile << endl;

	statFile << fileName << ","; //file Name
	statFile << lb << ",";
		
	if(ZInc < 1e13)
		statFile << ZInc << ",";
	else
		statFile << ",";

	if(gap < 1e13)
		statFile << gap << "%,";
	else
		statFile << ",";

	statFile << getSpTypeName(spType) << ",";
	statFile << exploredNodes << ",";
	statFile << maxTreeHeight << ",";
	statFile << routeCounter << ",";
	if(isInt){
		statFile << "Yes,";
	}else{
		statFile << "No,";
	}
	statFile << (double)(tEnd - tStart)/CLOCKS_PER_SEC << "s" ; //Time

	statFile.close();
	//--------------------------------------

#else
	myEnv.set(GRB_IntParam_MIPFocus, 1);
	//myEnv.set(GRB_DoubleParam_TimeLimit, 1200);

	//Disable gurobi output
	myEnv.set(GRB_IntParam_OutputFlag,0);	
	myEnv.set(GRB_DoubleParam_TimeLimit, GRB_INFINITY);

	//Build DWM model
	buildDWM();
	
	Node *rootNode = new Node(cDualVars,eDualVars);
	rootNode->setModel(model);
	rootNode->setVHash(vHash);
	rootNode->setCHash(cHash);
	rootNode->setNodeId(0);
	rootNode->setTreeLevel(0);

	//At this point, root node has its own copy of the model
	delete model;
	vHash.clear();
	cHash.clear();

	//Solve DWM model by CG
	tStart = clock();
	if(parameters->solveByMIP()){
		status = BaP(rootNode);
	}else{
		status = solveLPByColumnGeneration(rootNode,0);
	}
	tEnd = clock();

	////--------------------------------------
	////SAVE OUTPUT
	////--------------------------------------	
	string fileName = data->getFileName();
	unsigned pos = 0;

	while(pos = fileName.find(getFileSeparator()) != std::string::npos){
		fileName.erase(0, pos);
	}

	//--------------------------------------
	//SOLUTION FILE
	//--------------------------------------
	ofstream solFile;
	solFile.open("solutions.txt", ios::out | ios::app);

	//Begin of new solution
	solFile << setw(80) << setfill('-') << "-" << setfill (' ') << endl;

	//Instance name	
	solFile << "#Instance: " << endl << fileName << endl;

	Solution *mySolution;
	if(parameters->solveByMIP()){		
		if(solutions.size() > 0){
			mySolution = (*solutions.begin());
			solFile << *mySolution;
		}else{
			solFile << "No solution found. Final status was: " << status << endl;
		}
	}else{
		mySolution = rootNode->getSolution();
		solFile << *mySolution;
	}
	
	//End of new solution
	solFile << setw(80) << setfill('-') << "-" << setfill (' ') << endl;
	solFile.close();
	//--------------------------------------

	//--------------------------------------
	//STATISTICS FILE	
	//--------------------------------------
	//Verify if statistics file is empty
	std::ifstream myFile("statistics.csv");
	bool isEmpty = myFile.peek() == std::ifstream::traits_type::eof();
	myFile.close();

	//Save statistics
	ofstream statFile;
	statFile.open("statistics.csv", ios::out | ios::app);

	//Header
	if(isEmpty)
		statFile << "File_Name,LB,UB,Gap,SP_Method,Exp_Nodes,Max_Tree_Height,Total_Routes,Integer,Time" << endl;
	else
		statFile << endl;

	statFile << fileName << ","; //file Name
	statFile << lb << ",";
		
	if(ZInc < 1e13)
		statFile << ZInc << ",";
	else
		statFile << ",";

	if(gap < 1e13)
		statFile << gap << "%,";
	else
		statFile << ",";

	statFile << getSpTypeName(spType) << ",";
	statFile << exploredNodes << ",";
	statFile << maxTreeHeight << ",";
	statFile << routeCounter << ",";
	if(isInt){
		statFile << "Yes,";
	}else{
		statFile << "No,";
	}
	statFile << (double)(tEnd - tStart)/CLOCKS_PER_SEC << "s" ; //Time

	statFile.close();
#endif

	return status;
}

void Solver::buildInitialModel()
{
	Job *job;
	Equipment *e;
	int tInit, tEnd;
	int cont;
	int contVars = 0;
	int contCons = 0;

	cout << "*******************" << endl;
	cout << "Creating ovf model." << endl;
	cout << "*******************" << endl;

	//----------------
	//CREATE VARIABLES
	//----------------
	Variable v, y, x, w;

	cont = 0;
	cout << "Creating y vars." << endl;
	//yVars
	double coeff = 0.0;
	for(int j=1; j < data->numJobs; j++){ //job 0 does not have an Y var
		job = data->jobs[j];
		tInit = job->getFirstStartTimePeriod();
		tEnd = job->getLastStartTimePeriod();

		for(int t=tInit; t <= tEnd; t++){
			y.reset();
			y.setType(V_Y);
			y.setStartJob(j);
			y.setTime(t);

			//coeff = (double)((data->minDistance/(data->horizonLength)) * t);

			vHash[y] = true;
			model->addVar(0.0,1.0,coeff,GRB_INTEGER,y.toString());
			cont++;
		}
	}
	cout << "Total y Vars created: " << cont << endl;
	contVars += cont;

	//xVars and wVars
	int contw = 0;
	cont = 0;
	cout << "Creating x and w vars." << endl;
	queue<Vertex*> myQueue;
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		e = data->equipments[eqType];

		//Make a BFS over the problem network
		Vertex *o, *d;
		vector<vector<bool>> visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

		o = data->problemNetwork[0][0]; //depot
		myQueue.push(o);

		while(myQueue.size() > 0){
			o = myQueue.front();			
			myQueue.pop();

			int oJob = o->getJob();
			int oTime = o->getTime();

			int sourceReq = data->jobs[oJob]->getEquipmentTypeRequired(eqType);

			if(!visited[oJob][oTime]){
				visited[oJob][oTime] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::iterator it = o->getAdjacenceList(eqType).begin();
				vector<Vertex*>::iterator eit = o->getAdjacenceList(eqType).end();
				for(; it != eit; it++){
					d = (*it);
					int dJob = d->getJob();
					int dTime = d->getTime();

					int destinationReq = data->jobs[dJob]->getEquipmentTypeRequired(eqType);
					double transitionTime = e->getNotRoundedTransitionTime(oJob,dJob);

					//CREATE VARS
					if(oJob != dJob){ //not waiting					
						//create xVar
						x.reset();
						x.setType(V_X);
						x.setStartJob(o->getJob());
						x.setEndJob(d->getJob());
						x.setTime(o->getTime());
						x.setArrivalTime(d->getTime());
						x.setEquipmentTipe(eqType);

						if(vHash.find(x) == vHash.end()){
							vHash[x] = true;
							model->addVar(0.0,1.0,transitionTime,GRB_INTEGER, x.toString());
							data->addEdge(o->getJob(), d->getJob(), o->getTime(), d->getTime(),transitionTime);
							cont++;
						}					
					}else{
						w.reset();
						w.setType(V_W);
						w.setStartJob(o->getJob());
						w.setTime(o->getTime());
						w.setEquipmentTipe(eqType);

						if(vHash.find(w) == vHash.end()){
							vHash[w] = true;
							model->addVar(0.0,1.0,0.0,GRB_INTEGER,w.toString());							
							data->addEdge(o->getJob(), o->getJob(), o->getTime(), o->getTime() + 1,0.0);
							contw++;
						}
					}
					myQueue.push(d);
				}
			}
		}
		visited.clear();
	}
	
	contVars += cont + contw;

	model->update();
	cout << "Total x Vars created: " << cont << endl;
	cout << "Total w Vars created: " << contw << endl;
	cout << "Total Variables: " << contVars << endl;
	//----------------------

	//----------------------
	//CREATE CONSTRAINTS
	//----------------------
	GRBVar var1, var2;
	Constraint c1, c2;

	cont = 0;
	cout << "Creating cover constraints." << endl;
	//Cover Constraints
	for(int j=1; j < data->numJobs; j++){
		GRBLinExpr expr = 0;

		c1.reset();
		c1.setType(C_COVER);
		c1.setStartJob(j);

		if(cHash.find(c1) == cHash.end()){
			for(int t=0; t <= data->horizonLength; t++){
				y.reset();
				y.setType(V_Y);
				y.setStartJob(j);
				y.setTime(t);

				if(vHash.find(y) != vHash.end()){
					var1 = model->getVarByName(y.toString());
					expr += var1;
				}
			}
			cHash[c1] = true;
			model->addConstr(expr == 1, c1.toString());
			cont ++;
		}
	}
	contCons += cont;
	cout << "Total cover constraints created: " << cont << endl;

	cout << "Creating synchronization constraints." << endl;
	cont = 0;
	//Synchronization Constraints
	for(int j=1; j < data->numJobs; j++){
		job = data->jobs[j];
		tInit = job->getFirstStartTimePeriod();
		tEnd = job->getLastStartTimePeriod();
		for(int t=tInit; t <= tEnd; t++){
			//Get y variable
			y.reset();
			y.setType(V_Y);
			y.setStartJob(j);
			y.setTime(t);

			if(vHash.find(y) == vHash.end()) continue; //job cannot be attended at time period t, so go to t+1
			var1 = model->getVarByName(y.toString());

			for(int eqType=0; eqType < data->numEquipments; eqType++){
				//Verify that job j requires eqType
				if(job->getEquipmentTypeRequired(eqType) <= 0) continue;				

				c1.reset();
				c1.setType(C_SYNCH);
				c1.setStartJob(j);
				c1.setTime(t);
				c1.setEquipmentType(eqType);
				c1.setId(cont);

				if(cHash.find(c1) == cHash.end()){
					GRBLinExpr expr = 0;				
					expr -= var1;

					for(int i=0; i < data->numJobs; i++){
						//Get x Variable
						x.reset();
						x.setType(V_X);
						x.setStartJob(j);
						x.setEndJob(i);
						x.setTime(t);
						x.setEquipmentTipe(eqType);

						if(vHash.find(x) != vHash.end()){
							var2 = model->getVarByName(x.toString());
							expr += var2;
						}
					}

					cHash[c1] = true;
					model->addConstr(expr == 0, c1.toString());
					cont ++;
				}
			}
		}
	}
	eDualVars = cont;
	contCons += cont;
	cout << "Total synchronization constraints created: " << cont << endl;

	cont = 0;
	cout << "Creating flow init constraints." << endl;
	//Fow Init constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		e = data->equipments[eqType];
		
		c1.reset();
		c1.setType(C_OVF_FLOW_INIT);
		c1.setEquipmentType(eqType);

		if(cHash.find(c1) != cHash.end()) continue;

		GRBLinExpr expr = 0;

		for(int j=1; j < data->numJobs; j++){
			x.reset();
			x.setType(V_X);
			x.setStartJob(0);
			x.setEndJob(j);
			x.setTime(0);
			x.setEquipmentTipe(eqType);

			if(vHash.find(x) != vHash.end()){
				var1 = model->getVarByName(x.toString());
				expr += var1;				
			}
		}

		cHash[c1] = true;
		model->addConstr(expr == e->getNumMachines(), c1.toString());
		cont ++;
	}
	contCons += cont;
	cout << "Total flow init constraints created: " << cont << endl;

	cont = 0;
	cout << "Creating flow constraints." << endl;
	vector<Vertex*>::iterator it, eit;
	//Create Flow constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		e = data->equipments[eqType];
		for(int j=1; j<data->numJobs; j++){
			job = data->jobs[j];

			if(job->getEquipmentTypeRequired(eqType) <= 0) continue;

			tInit = min(job->getFirstStartTimePeriod(),(int)e->getTransitionTime(0,j));
			tEnd = job->getLastStartTimePeriod();

			for(int t = tInit; t <= tEnd; t++){
				c1.reset();
				c1.setType(C_OVF_FLOW);
				c1.setStartJob(j);
				c1.setTime(t);
				c1.setEquipmentType(eqType);

				if(cHash.find(c1) == cHash.end()){
					GRBLinExpr expr = 0;

					//Leaving variables (+)
					it = data->problemNetwork[j][t]->getAdjacenceList(eqType).begin();
					eit = data->problemNetwork[j][t]->getAdjacenceList(eqType).end();

					for(; it != eit; it++){
						int i = (*it)->getJob();
						
						if(i != j){ //transition
							//Get x variable
							v.reset();
							v.setType(V_X);
							v.setStartJob(j);
							v.setEndJob(i);
							v.setTime(t);
							v.setEquipmentTipe(eqType);

							if(vHash.find(v) != vHash.end()){
								var1 = model->getVarByName(v.toString());
								expr += var1;
							}
						}else{ //waiting
							//Get w variable
							v.reset();
							v.setType(V_W);
							v.setStartJob(j);
							v.setTime(t);
							v.setEquipmentTipe(eqType);

							if(vHash.find(v) != vHash.end()){
								var1 = model->getVarByName(v.toString());
								expr += var1;
							}
						}
					}

					//Entering variaveis (-)
					it = data->problemNetwork[j][t]->getIncidenceList(eqType).begin();
					eit = data->problemNetwork[j][t]->getIncidenceList(eqType).end();

					for(; it != eit; it++){
						int i = (*it)->getJob();
						int t1 = (*it)->getTime();

						if(i != j){ //Transition
							//Get x variable
							v.reset();
							v.setType(V_X);
							v.setStartJob(i);
							v.setEndJob(j);
							v.setTime(t1);
							v.setEquipmentTipe(eqType);

							if(vHash.find(v) != vHash.end()){
								var1 = model->getVarByName(v.toString());
								expr -= var1;
							}
						}else{ //Waiting
							//Get w Variable
							v.reset();
							v.setType(V_W);
							v.setStartJob(j);
							v.setTime(t1);
							v.setEquipmentTipe(eqType);

							if(vHash.find(v) != vHash.end()){
								var1 = model->getVarByName(v.toString());
								expr -= var1;
							}
						}
					}

					cHash[c1] = true;
					model->addConstr(expr == 0, c1.toString());
					cont ++; 
				}
			}
		}
	}
	contCons += cont;
	cout << "Total flow constraints created: " << cont << endl;
	cout << "Total constraints in OVF: " << contCons << endl;

	model->update();
	//----------------------
	
	model->write("modelo_OVF.lp");
}

void Solver::buildDWM()
{		
	int cont;
	int contVars = 0;
	int contCons = 0;

	GRBVar var1, var2;
	GRBConstr cons;

	Constraint c;
	ConstraintHash::iterator cit;

	Variable v, b, f;

	cout << "****************************" << endl;
	cout << "Creating dwm model." << endl;
	cout << "****************************" << endl;

	//Relax integer variables
	
	cout << "Deleting w, x (ovf) variables, relaxing y variaveis." << endl;
	//Delete w variables
	VariableHash::iterator vit = vHash.begin();
	while(vit != vHash.end()){
		v = vit->first;
		var1 = model->getVarByName(v.toString());

		if(v.getType() == V_W){ 
			model->remove(var1);
			vit = vHash.erase(vit);
		}else if(v.getType() == V_X){			 
			model->remove(var1);
			vit = vHash.erase(vit); 
		}else if(v.getType() == V_Y){
			//Relax integer variable
			var1.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
			vit++;
		}		
	}

	cont = 0;
	cout << "Creating b auxilaty variables." << endl;
	//Create bAux vars
	for(int j=0; j< data->numJobs; j++){
		b.reset();
		b.setType(V_BAUX);
		b.setStartJob(j);

		if(vHash.find(b) == vHash.end()){
			vHash[b] = true;
			model->addVar(0.0,1.0,bigM,GRB_CONTINUOUS,b.toString());
			cont ++;
		}
	}
	contVars += cont;
	cout << "Total b aux variables created: " << cont << endl;

	
	cont = 0;
	cout << "Creating f auxiliary variables." << endl;
	//fAuxVar
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		vHash[f] = true;
		model->addVar(0.0,e->getNumMachines(),bigM,GRB_CONTINUOUS, f.toString());
		cont ++;
	}
	contVars += cont;
	cout << "Total f aux variables created: " << cont << endl;
	cout << "Total auxiliary variables created in dwm: " << contVars << endl;

	model->update();
	//----------------------

	//-----------------------------
	//ERASE SUBPROBLEM CONSTRAINTS
	//-----------------------------	
	cout << "Erasing subproblem constraints (flow)." << endl;
	cit = cHash.begin();
	while(cit != cHash.end()){
		c = cit->first;

		if(c.getType() == C_OVF_FLOW || c.getType() == C_OVF_FLOW_INIT){
			cons = model->getConstrByName(c.toString());
			cHash.erase(cit++);
			model->remove(cons);
		}else{
			cit++;
		}
	}
	model->update();
	//----------------------

	//----------------------
	//CREATE CONSTRAINTS
	//----------------------
	cont = 0;
	cout << "Creating convexity constraints." << endl;
	//Route number constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];
		
		GRBLinExpr expr = 0;

		c.reset();
		c.setType(C_CONV);
		c.setEquipmentType(eqType);
		c.setId(cont);

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		if(cHash.find(c) == cHash.end() && vHash.find(f) != vHash.end()){
			var1 = model->getVarByName(f.toString());
			expr += var1;
			cHash[c] = true;
			model->addConstr(expr == e->getNumMachines(), c.toString());
			cont++;
		}
	}
	cDualVars = cont;
	contCons += cont;
	cout << "Convexity constraints created: " << cont << endl;
	

	//Add auxiliary variables to cover constraints
	cont = 0;
	cout << "Adding b aux vaiables to cover constraints." << endl;
	for(int j=0; j<data->numJobs; j++){
		c.reset();
		c.setType(C_COVER);
		c.setStartJob(j);

		v.reset();
		v.setType(V_BAUX);
		v.setStartJob(j);

		if(cHash.find(c) != cHash.end() && vHash.find(v) != vHash.end()){
			var1 = model->getVarByName(v.toString());
			cons = model->getConstrByName(c.toString());
			model->chgCoeff(cons,var1,1.0);
			cont++;
		}			
	}
	cout << "B aux vars created: " << cont << endl;
	cout << "Total constraints specific to the DWM: " << contCons << endl;

	model->update();
	//----------------------

	model->write("modelo_DWM.lp");
	getchar();
}

void Solver::buildDWMForExtendedFormulations()
{		
	int cont;
	int contVars = 0;
	int contCons = 0;

	GRBVar var1, var2;
	GRBConstr cons;

	Constraint c;
	ConstraintHash::iterator cit;
	Variable v, b, f;

	cout << "****************************" << endl;
	cout << "Creating extended model." << endl;
	cout << "****************************" << endl;

	//Relax integer variables
	
	cout << "deleting x and w (ovf) variables, and y variaveis." << endl;
	//Delete w variables
	VariableHash::iterator vit = vHash.begin();
	while(vit != vHash.end()){
		v = vit->first;
		var1 = model->getVarByName(v.toString());

		if(v.getType() == V_W){ 
			model->remove(var1);
			vit = vHash.erase(vit);
			/*var1.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
			vit++;*/
		}else if(v.getType() == V_X){			 
			model->remove(var1);
			vit = vHash.erase(vit); 
		}else if(v.getType() == V_Y){
			//Relax integer variable
			var1.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
			vit++;
		}		
	}

	cont = 0;
	cout << "Creating b auxilaty variables." << endl;
	//Create bAux vars
	for(int j=1; j< data->numJobs; j++){
		b.reset();
		b.setType(V_BAUX);
		b.setStartJob(j);

		if(vHash.find(b) == vHash.end()){
			vHash[b] = true;
			model->addVar(0.0,1.0,bigM,GRB_CONTINUOUS,b.toString());
			cont ++;
		}
	}
	contVars += cont;
	cout << "Total b aux variables created: " << cont << endl;

	
	cont = 0;
	cout << "Creating f auxiliary variables." << endl;
	//fAuxVar
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		vHash[f] = true;
		model->addVar(0.0,e->getNumMachines(),bigM,GRB_CONTINUOUS, f.toString());
		cont ++;
	}
	contVars += cont;
	cout << "Total f aux variables created: " << cont << endl;
	cout << "Total auxiliary variables created in dwm: " << contVars << endl;

	model->update();
	//----------------------

	//-----------------------------
	//ERASE flow CONSTRAINTS
	//-----------------------------	
	/*cout << "Erasing flow conservation constraints." << endl;
	cit = cHash.begin();
	while(cit != cHash.end()){
		c = cit->first;

		if(c.getType() == C_OVF_FLOW){
			cons = model->getConstrByName(c.toString());
			cHash.erase(cit++);
			model->remove(cons);
		}else{
			cit++;
		}
	}
	model->update();*/
	//----------------------

	//Add auxiliary variables to constraints
	cont = 0;
	cout << "Adding faux vars to flowinit constraints." << endl;
	//Route number constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		c.reset();
		c.setType(C_OVF_FLOW_INIT);
		c.setEquipmentType(eqType);

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		if(cHash.find(c) != cHash.end() && vHash.find(f) != vHash.end()){
			var1 = model->getVarByName(f.toString());
			cons = model->getConstrByName(c.toString());
			model->chgCoeff(cons,var1,1.0);
			cont++;
		}
	}
	cDualVars = cont;
	contCons += cont;
	cout << "Convexity constraints created: " << cont << endl;
	

	//Add auxiliary variables to cover constraints
	cont = 0;
	cout << "Adding b aux vaiables to cover constraints." << endl;
	for(int j=0; j<data->numJobs; j++){
		c.reset();
		c.setType(C_COVER);
		c.setStartJob(j);

		v.reset();
		v.setType(V_BAUX);
		v.setStartJob(j);

		if(cHash.find(c) != cHash.end() && vHash.find(v) != vHash.end()){
			var1 = model->getVarByName(v.toString());
			cons = model->getConstrByName(c.toString());
			model->chgCoeff(cons,var1,1.0);
			cont++;
		}			
	}
	cout << "B aux vars created: " << cont << endl;
	cout << "Total constraints specific to the DWM: " << contCons << endl;


	model->update();
	//----------------------

	//model->write("modelo_CGExtended.lp");
	//getchar();
}

int Solver::solveLPByColumnGeneration(Node *node, int treeSize)
{
	int status = GRB_INPROGRESS;
	double Zlp = 1e13;
	
	int rCount;
	double minRouteCost = 0.0;

	double lagrangeanBound = 0.0;
	int fixatedVars;
	int totalFixatedVars = 0;
	bool tryToFixVars = true;

	int iteration = 0;
	int printFrequency = (parameters->getPrintLevel() > 0)? 1 : 5;

	//Reset subproblem solver and collapse vertices
	spSolver->reset();
	Route *myRoute;
	vector<Route*> generatedRoutes = vector<Route*>();
	vector<Route*>::iterator rit, eit;
	//spSolver->collapseVertices();

	//Dynamically solve the model by column generation.
	bool end = false;
	while(!end){
		stringstream output;

		clock_t currentTime = clock();
		double timeSpent = (double)(currentTime - tStart)/CLOCKS_PER_SEC;
		if(timeSpent > parameters->getTimeLimit())
			return GRB_INFEASIBLE;

		//Solve current model
		iteration ++;
		clock_t gurobiStart, gurobiEnd;
		
		/*node->getModel()->write("Test.lp");
		getchar();*/
		node->getModel()->write("Test.lp");
		gurobiStart = clock();
		status = node->solve();
		node->printSolution();
		getchar();
		gurobiEnd = clock();

		cout << endl <<  separator.str();
		cout << endl << "Solver elapsed time: " << (double)(gurobiEnd - gurobiStart)/CLOCKS_PER_SEC;

		if(status == GRB_OPTIMAL){
			int sw = 0;		
			
			rCount = 0;
			fixatedVars = 0;
			double Zlp = node->getZLP();
			lagrangeanBound = Zlp;

			cout << endl << "Starting route generation..."; 

			clock_t spInit, spEnd;
			spInit = clock();
			double minRC = 1e13;
			//Generate routes for each equipment type
			for(int eqType = 0; eqType < data->numEquipments; eqType++){
				Equipment *e = data->equipments[eqType];	
				minRouteCost = 0.0;								
				
				spSolver->solve(node, eqType, 10);
				if(spSolver->isInfeasible()){
					return GRB_INFEASIBLE;
				}

				//Verify that at least 1 route was generated
				if(spSolver->routes.size() == 0) continue;

				//Imprimir las rotas para o subproblema de este tipo de equipamento
				vector<Route*>::iterator itRoute = spSolver->routes.begin();
				vector<Route*>::iterator eitRoute = spSolver->routes.end();

				//if(parameters->getPrintLevel() > 0){
					cout << endl << endl << "Routes Generated for Equipment Type: " << eqType;
					while(itRoute != eitRoute){
						Route *spRoute = (*itRoute);					
						cout << endl << spRoute->toString();
						itRoute++;
					}
				//}

				//Append routes to generated routes vector
				minRouteCost = spSolver->routes[0]->getReducedCost();
				minRC = min(minRC,minRouteCost);
				generatedRoutes.insert(generatedRoutes.end(),spSolver->routes.begin(),spSolver->routes.end());
				
				cout << endl << "Minimum reduced cost found for Equipment Type " << eqType << ": " << minRouteCost;
				
				lagrangeanBound -= (e->getNumMachines() * minRouteCost);
			}
			getchar();
			spEnd = clock();

			double spTimeSpent = (double)(spEnd - spInit)/CLOCKS_PER_SEC;
			//if(parameters->getPrintLevel() > 0)
				cout << endl << "Consumed time generating routes: " << spTimeSpent << "s";

			//If no routes where generated, the current lp solution is optimal
			if(generatedRoutes.size() == 0){
				if(parameters->useDualStabilization()){
					if(node->getMaxPiDifference() <= parameters->getEpsilon()){
						end = true;
					}else{
						cout << "No route found. Updating Pi" << endl;
						node->updatePi();
						continue;
					}
				}else{
					end = true;
				}
			}else{
				//fix variables by reduced cost.
				/*if(!parameters->useDualStabilization()){
					if(iteration % 10 == 0 || tryToFixVars){
						fixatedVars = node->fixVarsByReducedCost(lagrangeanBound);
						totalFixatedVars += fixatedVars;
						if(fixatedVars)
							tryToFixVars = true;
						else
							tryToFixVars = false;
					}
				}*/
				
				//Add routes to the model
				node->addColumns(generatedRoutes, routeCounter);
				rCount += generatedRoutes.size();
					
				//node->getModel()->write("Test.lp");
				//getchar();
				generatedRoutes.clear();
			}

			if(iteration % printFrequency == 0 || end){
				output << left;
				output << "| " << "Id: " << setw(4) << node->getNodeId() << " Unexp: " << setw(4) << treeSize << " Iter: " << setw(5) << iteration;
				output << "| " << "Zlp: " << setw(7) << Zlp;
				
				if(lb < 1e13)
					output << " LB: " << setw(7) << lb;
				else
					output << " LB: " << setw(7) << "--";

				if(ZInc < 1e13)
					output << " UB: " << setw(7) << ZInc;
				else
					output << " UB: " << setw(7) << "--";

				if(gap < 1e13)
					output << " Gap: " << setprecision(3) << gap << "%" ;
				else
					output << " Gap: " << setw(7) << "--";

				output << "| " << "Routes: " << setw(5) << rCount << "Total: " << setw(5) << routeCounter << " MinRC: " << setw(10) << minRC;
				output << "| " << "LagBound: " << setw(10) << lagrangeanBound << " Fix: " << setw(4) << fixatedVars << " TFix: " << setw(5) << totalFixatedVars;
				output << "| " << "Time: " << setw(5)  << (double)(clock() - tStart)/CLOCKS_PER_SEC << "s | ";

				cout << endl << output.str() << endl;
			}
		}else{ //INFEASIBLE
			//node->getModel()->write("modelo_inf.lp");
			cout << "Infeasible" << endl;
			return GRB_INFEASIBLE;
		}
	}

	if(parameters->solveByMIP() == false)
		isInt = node->isIntegerSolution();

	if(node->getNodeId() == 0)
		lb = node->getZLP();

	if(parameters->getPrintLevel() > 0)
		node->printSolution();


	return status;
}

int Solver::BaP(Node *node)
{	
	int status = GRB_INPROGRESS;
	exploredNodes = 0;
	
	vector<Node*> myStack = vector<Node*>();
	myStack.push_back(node);
	Node *currentNode;

	cout << separator.str();
	cout << "Starting branch and price algorithm. Initial Incumbent: " << ZInc << endl;
	cout << separator.str();

	while(myStack.size() > 0){
		clock_t currentTime = clock();
		double timeSpent = (double)(currentTime - tStart)/CLOCKS_PER_SEC;
		if(timeSpent > parameters->getTimeLimit())
			break;

		currentNode = myStack.back();
		myStack.pop_back();

		cout << separator.str();
		if(parameters->getPrintLevel() > 0)
			cout << "Starting column generation on node " << exploredNodes << endl;

		currentNode->setNodeId(exploredNodes);
		status = solveLPByColumnGeneration(currentNode, myStack.size());
		exploredNodes ++;

		if(exploredNodes == 1)
			currentNode->printSolution();

		if(parameters->getPrintLevel() > 0)
			cout << "Column generation on node " << exploredNodes << " completed." << endl;
			cout << separator.str();

		if(status != GRB_OPTIMAL){
			cout << "Node " << exploredNodes << " INFEASIBLE. " << endl;
			delete currentNode;
			continue;
		}else{
			double Zlp = currentNode->getZLP();
			if(currentNode->isIntegerSolution()){
				if(Zlp < ZInc || solutions.size() == 0){
					Solution *s = currentNode->getSolution();
					
					cout << separator.str();
					cout << "NEW INCUMBENT FOUND: " << endl;
					cout << s->toString() << endl;
					cout << separator.str();

					solutions.insert(s);
					ZInc = Zlp;
					gap = (double)(ZInc / lb);

					if(solutions.size() == parameters->getNumSolutions())
						break; //quit bap with current solutions.
				}
				continue;
			}else if(ceil(Zlp) >= ZInc && exploredNodes > 1){
				cout << "Node " << exploredNodes << " PRUNED BY BOUND. " << ceil(Zlp) << " > " << ZInc << endl;
				delete currentNode;
				continue;
			}
		}

		//Fix by reduced costs.
		int fix = currentNode->fixVarsByReducedCost(ZInc - currentNode->getZLP());
		cout << fix << " variables eliminated by reduced cost." << endl;
		
		//Node cleaning
		int cleaned = currentNode->cleanNode(300);
		cout << cleaned << " routes eliminated in node clening." << endl;

		//Get branching candidate
		Variable branchV = currentNode->getMostFractional();
		cout << "Branching on variable: " << branchV.toString() << endl;

		//Add two nodes to the stack
		Node *nodeIzq = new Node(*currentNode);
		Node *nodeDer = new Node(*currentNode);

		bool addBranch1 = nodeIzq->addBranchConstraint(branchV, 0.0);
		bool addBranch2 = nodeDer->addBranchConstraint(branchV, 1.0);

		if(!addBranch1 || !addBranch2){
			cout << "ATENTION: error adding branch constraint on variable: " << branchV.toString() << endl;
			break;
		}

		myStack.push_back(nodeIzq);
		myStack.push_back(nodeDer);

		//update tree height
		maxTreeHeight = max(nodeIzq->getTreeLevel(), maxTreeHeight);
		
		//parent node not needed anymore
		delete currentNode;
	}

	if(solutions.size() > 0){
		cout << separator.str();
		cout << "BEST SOLUTION FOUND: " << endl;
		cout << (*solutions.begin())->toString() << endl;
		cout << separator.str();
		isInt = true;
	}else{
		cout << "ATENTION: No solution found =( " << endl;
	}

	return status;
}

void Solver::buildProblemNetwork()
{
	Vertex *s, *d;
	queue<Vertex*> myQueue;

	cout << "Creating problem network." << endl;

	s = new Vertex(data->numEquipments);	
	s->setJob(0);
	s->setTime(0);

	data->problemNetwork[0][0] = s;
	data->vertexSet.insert(s);
	myQueue.push(s);

	while(myQueue.size() > 0){
		s = myQueue.front();
		Job *sJob = data->jobs[s->getJob()];
		
		for(int eqType=0; eqType < data->numEquipments; eqType++){
			Equipment *e = data->equipments[eqType];

			//Verify equipment requirement of job s
			if(s->getJob() != 0 && !sJob->getEquipmentTypeRequired(eqType)) continue;	

			//Verify that vertex s belongs to the time window of sJob (sJob can be attended at this time)
			if(s->getTime() >= sJob->getFirstStartTimePeriod() && s->getTime() <= sJob->getLastStartTimePeriod()){
				//create adjacence list for vertex s (other Jobs)
				for(int j=1; j<data->numJobs; j++){
					Job *jJob = data->jobs[j];

					//Verify is not the same job
					if(s->getJob() == j) continue;

					//Verify equipment requirement of job j
					if(!jJob->getEquipmentTypeRequired(eqType)) continue;			
				
					//Verify time window compatibility
					int arriveTime = s->getTime() + sJob->getServiceTime() + (int)e->getTransitionTime(s->getJob(),j);
					if(arriveTime > jJob->getLastStartTimePeriod()) continue;

					if(data->problemNetwork[j][arriveTime] == nullptr){
						data->problemNetwork[j][arriveTime] = new Vertex(data->numEquipments);
						data->problemNetwork[j][arriveTime]->setJob(j);
						data->problemNetwork[j][arriveTime]->setTime(arriveTime);
						myQueue.push(data->problemNetwork[j][arriveTime]);
						//Add vertex to topologically ordered set
						data->vertexSet.insert(data->problemNetwork[j][arriveTime]);
					}

					d = data->problemNetwork[j][arriveTime];				

					s->addAdjacentVertex(eqType,d);
					d->addInicidentVertex(eqType,s);	
				}

				//Add return to depot to the adjacence list of job s
				if(s->getJob() != 0){
					if(data->problemNetwork[0][data->horizonLength] == nullptr){
						data->problemNetwork[0][data->horizonLength] = new Vertex(data->numEquipments);
						data->problemNetwork[0][data->horizonLength]->setJob(0);
						data->problemNetwork[0][data->horizonLength]->setTime(data->horizonLength);
						//Add vertex to topologically ordered set
						data->vertexSet.insert(data->problemNetwork[0][data->horizonLength]);
					}
					d = data->problemNetwork[0][data->horizonLength];
					s->addAdjacentVertex(eqType,d);
					d->addInicidentVertex(eqType,s);
				}
			}

			//Add same job at t+1 to adjacence list of s. (Waiting)
			if(s->getJob() != 0 && sJob->getLastStartTimePeriod() > s->getTime())
			{
				if(data->problemNetwork[s->getJob()][s->getTime()+1] == nullptr)
				{
					data->problemNetwork[s->getJob()][s->getTime()+1] = new Vertex(data->numEquipments);
					data->problemNetwork[s->getJob()][s->getTime()+1]->setJob(s->getJob());
					data->problemNetwork[s->getJob()][s->getTime()+1]->setTime(s->getTime()+1);
					myQueue.push(data->problemNetwork[s->getJob()][s->getTime()+1]);
					//Add vertex to topologically ordered set
					data->vertexSet.insert(data->problemNetwork[s->getJob()][s->getTime()+1]);
				}
				d = data->problemNetwork[s->getJob()][s->getTime()+1];

				s->addAdjacentVertex(eqType,d);
				d->addInicidentVertex(eqType,s);
			}
		}

		myQueue.pop();
	}
}
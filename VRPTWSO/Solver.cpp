#include "Solver.h"
#include "SubproblemSolver.h"
#include "Bucket.h"
#include "Route.h"
#include "Node.h"

#include <sstream>
#include <iomanip>
#include <vector>
#include <queue>

string itos(int i) {stringstream s; s << i; return s.str(); }

Solver::Solver(ProblemData *d) : data(d)
{
	//Get global parameters
	parameters = GlobalParameters::getInstance();

	//Create Gurobi enviroment and model
	env = new GRBEnv();
	model = new GRBModel(*env);	

	//Objective function value of auxiliary variables
	bigM = parameters->getBigM();
	
	//Initialize subproblem solver vector
	spSolver = new SubproblemSolver(data, QROUTE_NOLOOP);	

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

	//Global parameters
	ZInc = 1e13;
	totalNodes = 0;
	exploredNodes = 0;
	routeCounter = 0;

	//Build problem graph representation
	buildProblemNetwork();

	//Build the initial model
	buildInitialModel();

	//Get a first integer feasible solution (ZInc)
	GRBEnv myEnv = model->getEnv();
	myEnv.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);
	myEnv.set(GRB_IntParam_MIPFocus, 1);
	myEnv.set(GRB_IntParam_SolutionLimit, 1);
	myEnv.set(GRB_DoubleParam_Heuristics, 1);
	myEnv.set(GRB_IntParam_RINS, 1);
	myEnv.set(GRB_IntParam_ZeroObjNodes, 1);
	myEnv.set(GRB_IntParam_PumpPasses, 1);

	//Create tempNode to get a first ZInc from ovf formulation
	Node *tempNode = new Node();
	tempNode->setModel(model);
	tempNode->setVHash(vHash);
	tempNode->setCHash(cHash);

	int s = tempNode->solve();
	tempNode->printSolution();
	ZInc = tempNode->getZLP();
	cout << "Solution status: " << s << " - ZInc = " << ZInc << endl;
	delete tempNode;

	//Disable gurobi output
	myEnv.set(GRB_IntParam_OutputFlag,0);

	//Build Explicit DWM model
	buildDWM();

	Node *rootNode = new Node();
	rootNode->setModel(model);
	rootNode->setVHash(vHash);
	rootNode->setCHash(cHash);

	//At this point, root node has its own copy of the model
	delete model;
	vHash.clear();
	cHash.clear();

	//Solve DWM model by CG
	tStart = clock();
	status = BaP(rootNode);

	return status;
}

void Solver::buildInitialModel()
{
	Job *job;
	Equipment *e;
	int tInit, tEnd;

	cout << "*******************" << endl;
	cout << "Creating ovf model." << endl;
	cout << "*******************" << endl;

	//----------------
	//CREATE VARIABLES
	//----------------
	Variable v, y, x, w;

	cout << "Creating y vars." << endl;
	//yVars
	for(int j=1; j < data->numJobs; j++){ //job 0 does not have an Y var
		job = data->jobs[j];
		tInit = job->getFirstStartTimePeriod();
		tEnd = job->getLastStartTimePeriod();

		for(int t=tInit; t <= tEnd; t++){
			y.reset();
			y.setType(V_Y);
			y.setStartJob(j);
			y.setTime(t);

			vHash[y] = true;
			model->addVar(0.0,1.0,0.0,GRB_INTEGER,y.toString());
		}
	}

	//xVars and wVars
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
						}
					}
					myQueue.push(d);
				}
			}
		}
		visited.clear();
	}
	
	model->update();
	//----------------------

	//----------------------
	//CREATE CONSTRAINTS
	//----------------------
	GRBVar var1, var2;
	Constraint c1, c2;

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
		}
	}

	cout << "Creating synchronization constraints." << endl;
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
				}
			}
		}
	}

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
	}
	
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
				}
			}
		}
	}
	model->update();
	//----------------------
	
	model->write("modelo_OVF.lp");
}

void Solver::buildDWM()
{		
	GRBVar var1, var2;
	GRBConstr cons;

	Constraint c;
	Variable v, b, f;

	cout << "****************************" << endl;
	cout << "Creating explicit dwm model." << endl;
	cout << "****************************" << endl;

	//Relax integer variables
	
	cout << "Deleting w (ovf) variables, relaxing y and x variaveis." << endl;
	//Delete w variables
	VariableHash::iterator vit = vHash.begin();
	while(vit != vHash.end()){
		v = vit->first;
		var1 = model->getVarByName(v.toString());

		if(v.getType() == V_W){ 
			model->remove(var1);
			vHash.erase(vit++);
		}else{
			if(v.getType() == V_X){
				//Relax integer x variable
				var1.set(GRB_CharAttr_VType, GRB_CONTINUOUS);				
			}else if(v.getType() == V_Y){
				//Relax integer variable
				var1.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
			}
			vit++;
		}
	}

	
	cout << "Creating b auxilaty variables." << endl;
	//Create bAux vars
	for(int i=0; i< data->numJobs; i++){
		for(int j=0; j<data->numJobs; j++){
			for(int t=0; t < data->horizonLength; t++){
				for(int eqType = 0; eqType < data->numEquipments; eqType++){
					v.reset();
					v.setType(V_X);
					v.setStartJob(i);
					v.setEndJob(j);
					v.setTime(t);
					v.setEquipmentTipe(eqType);

					if(vHash.find(v) == vHash.end()) continue;

					b.reset();
					b.setType(V_BAUX);
					b.setStartJob(i);
					b.setEndJob(j);
					b.setTime(t);
					b.setEquipmentTipe(eqType);

					if(vHash.find(b) == vHash.end()){
						vHash[b] = true;
						model->addVar(0.0,1.0,bigM,GRB_CONTINUOUS,b.toString());
					}
				}
			}
		}
	}
	
	cout << "Creating f auxiliary variables." << endl;
	//fAuxVar
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		vHash[f] = true;
		model->addVar(0.0,e->getNumMachines(),bigM,GRB_CONTINUOUS, f.toString());
	}

	model->update();
	//----------------------

	//-----------------------------
	//ERASE SUBPROBLEM CONSTRAINTS
	//-----------------------------	
	cout << "Erasing subproblem constraints (flow)." << endl;
	ConstraintHash::iterator cit = cHash.begin();

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
	cout << "Creating route number constraints." << endl;
	//Cardinality constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];
		
		GRBLinExpr expr = 0;

		c.reset();
		c.setType(C_CARD);
		c.setEquipmentType(eqType);

		f.reset();
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		if(cHash.find(c) == cHash.end() && vHash.find(f) != vHash.end()){
			var1 = model->getVarByName(f.toString());
			expr += var1;
			cHash[c] = true;
			model->addConstr(expr == e->getNumMachines(), c.toString());
		}
	}
	
	cout << "Creating explicit master constraints." << endl;
	//Explicit master constraints
	for(int i=0; i < data->numJobs; i++){
		Job *iJob = data->jobs[i];
		for(int j=0; j < data->numJobs; j++){
			for(int t=iJob->getFirstStartTimePeriod(); t <= iJob->getLastStartTimePeriod(); t++){
				for(int eqType=0; eqType < data->numEquipments; eqType++){
					GRBLinExpr expr = 0;

					c.reset();
					c.setType(C_EXPLICIT);
					c.setStartJob(i);
					c.setEndJob(j);
					c.setTime(t);
					c.setEquipmentType(eqType);

					v.reset();
					v.setType(V_X);
					v.setStartJob(i);
					v.setEndJob(j);
					v.setTime(t);
					v.setEquipmentTipe(eqType);

					b.reset();
					b.setType(V_BAUX);
					b.setStartJob(i);
					b.setEndJob(j);
					b.setTime(t);
					b.setEquipmentTipe(eqType);

					if(cHash.find(c) == cHash.end() && vHash.find(v) != vHash.end() && vHash.find(b) != vHash.end()){
						var1 = model->getVarByName(v.toString());
						var2 = model->getVarByName(b.toString());
						expr += var1;
						expr -= var2;
						cHash[c] = true;
						model->addConstr(expr == 0, c.toString());
					}

				}
			}
		}
	}

	model->update();
	//----------------------

	model->write("modelo_EDWM.lp");
}

int Solver::solveLPByColumnGeneration(Node *node, int treeSize)
{
	int status = GRB_INPROGRESS;
	double Zlp = 1e13;
	double lagrangeanBound = 0.0;
	int fixatedVars;
	int totalFixatedVars = 0;
	int rCount;
	int totalRoutes = 0;
	double minRouteCost = 0.0;
	int iteration = 0;

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

		//Solve current model
		iteration ++;
		status = node->solve();	

		if(status == GRB_OPTIMAL){
			int sw = 0;		
			
			rCount = 0;
			fixatedVars = 0;
			double Zlp = node->getZLP();
			lagrangeanBound = ZInc - Zlp;
			minRouteCost = 0.0;

			//Generate routes for each equipment type
			for(int eqType = 0; eqType < data->numEquipments; eqType++){
				Equipment *e = data->equipments[eqType];				
				
				spSolver->solve(node, eqType, 10);
				if(spSolver->isInfeasible()){
					return GRB_INFEASIBLE;
				}

				//Verify that at least 1 route was generated
				if(spSolver->routes.size() == 0) continue;

				//Append routes to generated routes vector
				minRouteCost = spSolver->routes[0]->getCost();
				generatedRoutes.insert(generatedRoutes.end(),spSolver->routes.begin(),spSolver->routes.end());
				lagrangeanBound -= (e->getNumMachines() * minRouteCost);
			}	

			//If no routes where generated, the current lp solution is optimal
			if(generatedRoutes.size() == 0){
				end = true;
			}else{
				//fix variables by reduced cost.
				if(iteration % 10 == 0){
					fixatedVars = node->fixVarsByReducedCost(lagrangeanBound);
					totalFixatedVars += fixatedVars;
				}

				//Add routes to the model
				rit = generatedRoutes.begin();
				eit = generatedRoutes.end();
				for(; rit != eit; rit++){
					myRoute = (*rit);
					myRoute->setRouteNumber(routeCounter++);
					//cout << myRoute->toString() << endl;
					if(!node->addColumn(myRoute)){
						cout << "Error: column " << myRoute->getRouteNumber() << "," 
							<< myRoute->getEquipmentType() << " already existed in node " << node->getNodeId() << endl;
					}
					delete myRoute;
					rCount ++;
					totalRoutes ++;
				}					
				generatedRoutes.clear();
			}

			if(iteration % 5 == 0 || end){
				output << left;
				output << "| " << "Id: " << setw(4) << node->getNodeId() << " Unexp: " << setw(4) << treeSize << " Iter: " << setw(5) << iteration;
				output << "| " << "Zlp: " << setw(7) << Zlp << " ZInc: " << setw(7) << ZInc;
				output << "| " << "Routes: " << setw(5) << rCount << "Total: " << setw(5) << totalRoutes << " MinRC: " << setw(10) << minRouteCost;
				output << "| " << "LagBound: " << setw(10) << lagrangeanBound << " Fix: " << setw(4) << fixatedVars << " TFix: " << setw(5) << totalFixatedVars;
				output << "| " << "Time: " << setw(5)  << (double)(clock() - tStart)/CLOCKS_PER_SEC << "s | ";

				cout << output.str() << endl;
			}
		}else{ //INFEASIBLE
			cout << "Infeasible" << endl;
			return GRB_INFEASIBLE;
		}
	}

	//node->printSolution();
	return status;
}

int Solver::BaP(Node *node)
{	
	int status = GRB_INPROGRESS;
	exploredNodes = 0;
	
	vector<Node*> myStack = vector<Node*>();
	myStack.push_back(node);
	Node *currentNode;

	string sep = "-------------------------------------------------------------------";
	cout << sep << endl;
	cout << "Starting branch and price algorithm. Initial Incumbent: " << ZInc << endl;
	cout << sep << endl;

	while(myStack.size() > 0){
		currentNode = myStack.back();
		myStack.pop_back();
		exploredNodes++;

		cout << sep << endl;
		cout << "Starting column generation on node " << exploredNodes << endl;

		currentNode->setNodeId(exploredNodes);
		status = solveLPByColumnGeneration(currentNode, myStack.size());

		cout << "Column generation on node " << exploredNodes << " completed." << endl;
		cout << sep << endl;

		if(status != GRB_OPTIMAL){
			cout << "Node " << exploredNodes << " INFEASIBLE. " << endl;
			delete currentNode;
			continue;
		}else{
			double Zlp = currentNode->getZLP();
			if(currentNode->isIntegerSolution()){
				if(Zlp < ZInc || solutions.size() == 0){
					Solution *s = currentNode->getSolution();
					
					cout << sep << endl;
					cout << "NEW INCUMBENT FOUND: " << endl;
					cout << s->toString() << endl;
					cout << sep << endl;

					solutions.insert(s);
					ZInc = Zlp;
				}
				continue;
			}else if(ceil(Zlp) >= ZInc && exploredNodes > 1){
				cout << "Node " << exploredNodes << " PRUNED BY BOUND. " << ceil(Zlp) << " > " << ZInc << endl;
				delete currentNode;
				continue;
			}
		}
		
		//Node cleaning
		cout << sep << endl;
		cout << "Cleaning node. " << endl;
		int cleaned = currentNode->cleanNode(500);
		cout << cleaned << " routes eliminated." << endl;
		cout << sep << endl;

		//Fix by reduced costs.
		cout << sep << endl;
		cout << "Fixating variables by reduced cost before branching. " << endl;
		int fix = currentNode->fixVarsByReducedCost(ZInc - currentNode->getZLP());
		cout << fix << " variables eliminated." << endl;
		cout << sep << endl;

		//Get branching candidate
		Variable branchV = currentNode->getMostFractional();
		cout << sep << endl;
		cout << "Branching on variable: " << branchV.toString() << endl;
		cout << sep << endl;

		//Add two nodes to the stack
		Node *nodeIzq = new Node(*currentNode);
		Node *nodeDer = new Node(*currentNode);

		nodeIzq->addBranchConstraint(branchV, 0.0);
		nodeDer->addBranchConstraint(branchV, 1.0);

		myStack.push_back(nodeIzq);
		myStack.push_back(nodeDer);
		
		//parent node not needed anymore
		delete currentNode;
	}

	if(solutions.size() > 0){
		cout << sep << endl;
		cout << "BEST SOLUTION FOUND: " << endl;
		cout << (*solutions.begin())->toString() << endl;
		cout << sep << endl;
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
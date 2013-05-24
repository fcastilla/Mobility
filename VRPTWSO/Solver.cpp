#include "Solver.h"
#include "SubproblemSolver.h"
#include "Bucket.h"
#include "Route.h"
#include "Node.h"

#include <sstream>
#include <vector>
#include <queue>

string itos(int i) {stringstream s; s << i; return s.str(); }

Solver::Solver(ProblemData *d) : data(d)
{
	//Create Gurobi enviroment and model
	env = new GRBEnv();
	//env->set(GRB_IntParam_Presolve,GRB_PRESOLVE_OFF);
	//env->set(GRB_IntParam_OutputFlag,0);
	model = new GRBModel(*env);	

	//Objective function value of auxiliary variables
	bigM = 10000;
	
	//Initialize subproblem solver vector
	spSolver = new SubproblemSolver(data, QROUTE_NOLOOP);	
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
	zInc = 10000;
	totalNodes = 0;
	exploredNodes = 0;
	routeCounter = 0;

	//Build problem graph representation
	buildProblemNetwork();

	//Build the initial model
	buildInitialModel();

	//Get a first integer feasible solution (Zinc)
	GRBEnv myEnv = model->getEnv();
	myEnv.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);
	myEnv.set(GRB_IntParam_MIPFocus, 0.5);
	myEnv.set(GRB_IntParam_SolutionLimit, 1);
	myEnv.set(GRB_DoubleParam_Heuristics, 1);
	myEnv.set(GRB_DoubleParam_ImproveStartTime,0.0);

	//Create tempNode
	Node *tempNode = new Node();
	tempNode->setModel(model);
	tempNode->setVHash(vHash);
	tempNode->setCHash(cHash);
	int s = tempNode->solve();
	tempNode->printSolution();

	zInc = model->get(GRB_DoubleAttr_ObjVal);
	cout << "Solution status: " << s << " - ZINC = " << zInc << endl;

	delete tempNode;

	//Build Explicit DWM model
	buildDWM();
	Node *rootNode = new Node();


	//Create root node
	/*Node *rootNode = new Node();
	rootNode->setModel(model);
	rootNode->setVHash(vHash);
	rootNode->setCHash(cHash);*/

	//Run Branch and Price (DFS)
	//int status = BaP(rootNode);

	return status;
}

void Solver::buildInitialModel()
{
	Job *job;
	Equipment *e;
	int tInit, tEnd;

	//----------------
	//CREATE VARIABLES
	//----------------
	Variable v, y, x, w;

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
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				vector<Vertex*>::const_iterator eit = o->getAdjacenceList(eqType).end();
				for(; it != eit; it++){
					d = (*it);
					int dJob = d->getJob();
					int dTime = d->getTime();

					int destinationReq = data->jobs[dJob]->getEquipmentTypeRequired(eqType);
					double transitionTime = e->getTransitionTime(oJob,dJob);

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
					cHash[c1] = true;
					model->addConstr(expr == 0, c1.toString());
				}
			}
		}
	}
	model->update();

	//Add variables to flow constraints	
	GRBConstr flowConstr;
	VariableHash::iterator vit = vHash.begin();
	VariableHash::iterator evit = vHash.end();

	for(; vit != evit; vit++){
		v = vit->first;		
		int j,i,time1,time2,eqType;

		if(v.getType() == V_X){
			var1 = model->getVarByName(v.toString());

			j = v.getStartJob();
			i = v.getEndJob();
			time1 = v.getTime();
			time2 = v.getArrivalTime();
			eqType = v.getEquipmentType();

			//Leaving j, at time1
			c1.reset();
			c1.setType(C_OVF_FLOW);
			c1.setStartJob(j);
			c1.setTime(time1);
			c1.setEquipmentType(eqType);

			if(cHash.find(c1) != cHash.end()){
				flowConstr = model->getConstrByName(c1.toString());
				model->chgCoeff(flowConstr, var1, 1.0);
			}

			//Entering i at time2
			c2.reset();
			c2.setType(C_OVF_FLOW);
			c2.setStartJob(i);
			c2.setTime(time2);
			c2.setEquipmentType(eqType);

			if(cHash.find(c2) != cHash.end()){
				flowConstr = model->getConstrByName(c2.toString());
				model->chgCoeff(flowConstr, var1, -1.0);
			}
		}else if(v.getType() == V_W){
			var1 = model->getVarByName(v.toString());

			j = v.getStartJob();
			time1 = v.getTime();
			time2 = time1 + 1;
			eqType = v.getEquipmentType();

			//leaving j at time1
			c1.reset();
			c1.setType(C_OVF_FLOW);
			c1.setStartJob(j);
			c1.setTime(time1);
			c1.setEquipmentType(eqType);

			if(cHash.find(c1) != cHash.end()){
				flowConstr = model->getConstrByName(c1.toString());
				model->chgCoeff(flowConstr,var1,1.0);
			}

			//entering j at time2
			c2.reset();
			c2.setType(C_OVF_FLOW);
			c2.setStartJob(j);
			c2.setTime(time2);
			c2.setEquipmentType(eqType);

			if(cHash.find(c2) != cHash.end()){
				flowConstr = model->getConstrByName(c2.toString());
				model->chgCoeff(flowConstr,var1,-1.0);
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

	//Relax integer variables
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

	model->write("modeloEDWM.lp");
}

int Solver::solveLPByColumnGeneration(Node *node)
{
	int status = GRB_INPROGRESS;

	//Reset subproblem solver and collapse vertices
	spSolver->reset();
	//spSolver->collapseVertices();

	//Dynamically solve the model by column generation.
	bool end = false;
	while(!end){	
		//Solve current model		
		status = node->solve();

		if(status != GRB_OPTIMAL)
			return status; 

		int sw = 0;		
		//node->printSolution();

		//Generate routes for each equipment type
		for(int eqType = 0; eqType < data->numEquipments; eqType++){
			spSolver->solve(node, eqType);
			double routeUseRC = node->getRouteUseReduzedCost(eqType);	

			if(spSolver->isInfeasible()){
				return GRB_INFEASIBLE;
			}

			//Add route(s) to the model
			vector<Route*>::iterator rit = spSolver->routes.begin();
			vector<Route*>::iterator eit = spSolver->routes.end();
			for(; rit != eit; rit++){				
				Route *myRoute = (*rit);
				double routeCost = myRoute->getCost();
				routeCost -= routeUseRC;
			
				if(routeCost >= 0) continue; //not a good route.

				myRoute->setCost(routeCost);
				myRoute->setRouteNumber(routeCounter++);				
				cout << myRoute->toString() << endl;

				if(node->addColumn(myRoute)){
					sw = 1;
				}else{
					cout << "Error: column " << myRoute->getRouteNumber() << "," 
						<< myRoute->getEquipmentType() << " already existed in node " << node->getNodeId() << endl;
				}

				delete myRoute;
			}
		}

		//If no routes where generated, the current lp solution is optimal
		if(sw == 0){
			end = true;
		}
	}

	node->printSolution();
	return status;
}

int Solver::BaP(Node *node)
{
	//TODO: implement Branch and Price method.
	exploredNodes = 0;
	node->setNodeId(exploredNodes++);
	int result = solveLPByColumnGeneration(node);
	cout << "Columns generated for node " << node->getNodeId() << ": " << node->getRouteCount() << endl;

	return result;
}

void Solver::buildProblemNetwork()
{
	Vertex *s, *d;
	queue<Vertex*> myQueue;

	s = new Vertex(data->numEquipments);	
	s->setJob(0);
	s->setTime(0);

	data->problemNetwork[0][0] = s;
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
					int arriveTime = s->getTime() + sJob->getServiceTime() + e->getTransitionTime(s->getJob(),j);
					if(arriveTime > jJob->getLastStartTimePeriod()) continue;

					if(data->problemNetwork[j][arriveTime] == nullptr){
						data->problemNetwork[j][arriveTime] = new Vertex(data->numEquipments);
						data->problemNetwork[j][arriveTime]->setJob(j);
						data->problemNetwork[j][arriveTime]->setTime(arriveTime);
						myQueue.push(data->problemNetwork[j][arriveTime]);
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
				}
				d = data->problemNetwork[s->getJob()][s->getTime()+1];

				s->addAdjacentVertex(eqType,d);
				d->addInicidentVertex(eqType,s);
			}
		}

		myQueue.pop();
	}
}
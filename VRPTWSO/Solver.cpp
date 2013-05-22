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
	model = new GRBModel(*env);	

	//Objective function value of auxiliary variables
	bigM = 1000000;
	
	//Initialize subproblem solver vector
	spSolver = new SubproblemSolver(data, QROUTE);	
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
	//Build problem graph representation
	buildProblemNetwork();

	//Build the initial model
	buildInitialModel();

	//Global parameters
	zInc = 1e13;
	totalNodes = 0;
	exploredNodes = 0;
	routeCounter = 0;

	//Create root node
	Node *rootNode = new Node();
	rootNode->setModel(model);
	rootNode->setVHash(vHash);
	rootNode->setCHash(cHash);

	//Run Branch and Price (DFS)
	int status = BaP(rootNode);

	return status;
}

void Solver::buildInitialModel()
{
	//----------------
	//CREATE VARIABLES
	//----------------

	//yVars
	for(int j=1; j < data->numJobs; j++){ //job 0 does not have an Y var
		Job *job = data->jobs[j];
		for(int t=job->getFirstStartTimePeriod(); t <= job->getLastStartTimePeriod(); t++){
			Variable y;
			y.setType(V_Y);
			y.setStartJob(j);
			y.setTime(t);

			vHash[y] = true;
			model->addVar(0.0,1.0,0.0,GRB_CONTINUOUS,y.toString());
		}
	}

	//xVars and bAuxVars
	queue<Vertex*> myQueue;
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		//Make a BFS over the problem network
		Vertex *o, *d;
		vector<vector<bool>> visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

		o = data->problemNetwork[0][0]; //depot
		myQueue.push(o);

		while(myQueue.size() > 0){
			o = myQueue.front();
			Job *oJob = data->jobs[o->getJob()];
			int sourceReq = oJob->getEquipmentTypeRequired(eqType);

			if(!visited[o->getJob()][o->getTime()]){
				visited[o->getJob()][o->getTime()] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				for(; it != o->getAdjacenceList(eqType).end(); it++)
				{
					d = (*it);
					Job *dJob = data->jobs[d->getJob()];
					int destinationReq = dJob->getEquipmentTypeRequired(eqType);
					double transitionTime = e->getTransitionTime(o->getJob(),d->getJob());

					//CREATE VARS
					if(o->getJob() != d->getJob()) //not waiting
					{
						//create xVar
						Variable x;
						x.setType(V_X);
						x.setStartJob(o->getJob());
						x.setEndJob(d->getJob());
						x.setTime(o->getTime());
						x.setEquipmentTipe(eqType);

						if(vHash.find(x) == vHash.end()){
							vHash[x] = true;
							model->addVar(0.0,1.0,ceil(transitionTime),GRB_CONTINUOUS, x.toString());
						}

						//create bAuxVar
						Variable b;
						b.setType(V_BAUX);						
						b.setStartJob(o->getJob());
						b.setEndJob(d->getJob());
						b.setTime(o->getTime());
						b.setEquipmentTipe(eqType);

						if(vHash.find(b) == vHash.end()){
							vHash[b] = true;
							model->addVar(0.0,1.0,bigM,GRB_CONTINUOUS, b.toString());
						}
						
					}				
					myQueue.push(d);
				}
			}
			myQueue.pop();
		}
		visited.clear();
	}

	//fAuxVar
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		Variable f;
		f.setType(V_FAUX);
		f.setEquipmentTipe(eqType);

		vHash[f] = true;
		model->addVar(0.0,e->getNumMachines(),bigM,GRB_CONTINUOUS, f.toString());
	}

	model->update();
	//----------------------

	//----------------------
	//CREATE CONSTRAINTS
	//----------------------
	GRBVar var1, var2;

	//Cover Constraints
	for(int j=1; j < data->numJobs; j++){
		GRBLinExpr expr = 0;

		Constraint c;
		c.setType(C_COVER);
		c.setStartJob(j);

		if(cHash.find(c) == cHash.end()){
			for(int t=0; t <= data->horizonLength; t++){
				Variable y;
				y.setType(V_Y);
				y.setStartJob(j);
				y.setTime(t);

				if(vHash.find(y) != vHash.end()){
					var1 = model->getVarByName(y.toString());
					expr += var1;
				}
			}
			cHash[c] = true;
			model->addConstr(expr == 1, c.toString());
		}
	}

	//Synchronization Constraints
	for(int j=1; j < data->numJobs; j++){
		Job *job = data->jobs[j];
		for(int t=job->getFirstStartTimePeriod(); t <= job->getLastStartTimePeriod(); t++){
			GRBLinExpr expr = 0;

			//Get y variable
			Variable y;
			y.setType(V_Y);
			y.setStartJob(j);
			y.setTime(t);

			if(vHash.find(y) == vHash.end()) continue; //job cannot be attended at time period t, so go to t+1

			var1 = model->getVarByName(y.toString());
			expr -= var1;

			for(int eqType=0; eqType < data->numEquipments; eqType++){
				//Verify that job j requires eqType
				if(job->getEquipmentTypeRequired(eqType) <= 0) continue;				

				Constraint c;
				c.setType(C_SYNCH);
				c.setStartJob(j);
				c.setTime(t);
				c.setEquipmentType(eqType);

				if(cHash.find(c) == cHash.end()){
					for(int i=0; i < data->numJobs; i++){
						//Get x Variable
						Variable v;
						v.setType(V_X);
						v.setStartJob(j);
						v.setEndJob(i);
						v.setTime(t);
						v.setEquipmentTipe(eqType);

						if(vHash.find(v) != vHash.end()){
							var2 = model->getVarByName(v.toString());
							expr += var2;
						}
					}
					cHash[c] = true;
					model->addConstr(expr == 0, c.toString());
				}
			}
		}
	}

	//Cardinality constraints
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];
		
		GRBLinExpr expr = 0;
		Constraint c;
		c.setType(C_CARD);
		c.setEquipmentType(eqType);

		Variable f;
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

					Constraint c;
					c.setType(C_EXPLICIT);
					c.setStartJob(i);
					c.setEndJob(j);
					c.setTime(t);
					c.setEquipmentType(eqType);

					Variable v;
					v.setType(V_X);
					v.setStartJob(i);
					v.setEndJob(j);
					v.setTime(t);
					v.setEquipmentTipe(eqType);

					Variable b;
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
	
	model->write("modelo.lp");
}

int Solver::solveLPByColumnGeneration(Node *node)
{
	int status = GRB_INPROGRESS;
	Route *myRoute;

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

		node->printSolution();
		int sw = 0;		

		//Generate routes for each equipment type
		for(int eqType = 0; eqType < data->numEquipments; eqType++){
			spSolver->solve(node, eqType);

			if(spSolver->isInfeasible()){
				return GRB_INFEASIBLE;
			}

			//Add route(s) to the model
			vector<Route*>::iterator rit = spSolver->routes.begin();
			for(; rit != spSolver->routes.end(); rit++){
				myRoute = (*rit);
				cout << myRoute->toString() << endl;
				double routeCost = myRoute->getCost();
				double verifiedCost = node->verifyRouteCost(myRoute);
				cout << endl;
				cout << "Verification: " << routeCost << " vs " << verifiedCost << endl;
				cout << endl;
				if(routeCost != verifiedCost){
					cout << "ERROOOOOO: " << routeCost << " vs " << verifiedCost << endl;
					getchar();
				}
				routeCost -= node->getRouteReducedCost(eqType);
				cout << routeCost << endl;
			
				if(routeCost >= 0) continue; //not a good route.

				myRoute->setRouteNumber(routeCounter++);
				if(node->addColumn(myRoute)){
					sw = 1;
				}else{
					cout << "Error: column " << myRoute->getRouteNumber() << "," 
						<< myRoute->getEquipmentType() << " already existed in node " << node->getNodeId() << endl;
				}
			}
		}

		//If no routes where generated, the current lp solution is optimal
		if(sw == 0){
			end = true;
		}
	}

	return status;
}

int Solver::BaP(Node *node)
{
	//TODO: implement Branch and Price method.
	int result = solveLPByColumnGeneration(node);

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
		
		for(int eqType=0; eqType < data->numEquipments; eqType++)
		{
			Equipment *e = data->equipments[eqType];

			//Verify equipment requirement of job s
			if(s->getJob() != 0 && !sJob->getEquipmentTypeRequired(eqType)) continue;	

			//Verify that vertex s belongs to the time window of sJob (sJob can be attended at this time)
			if(s->getTime() >= sJob->getFirstStartTimePeriod() && s->getTime() <= sJob->getLastStartTimePeriod())
			{
				//create adjacence list for vertex s (other Jobs)
				for(int j=1; j<data->numJobs; j++)
				{
					Job *jJob = data->jobs[j];

					//Verify is not the same job
					if(s->getJob() == j) continue;

					//Verify equipment requirement of job j
					if(!jJob->getEquipmentTypeRequired(eqType)) continue;			
				
					//Verify time window compatibility
					int arriveTime = s->getTime() + sJob->getServiceTime() + ceil(e->getTransitionTime(s->getJob(),j));
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
				if(s->getJob() != 0)
				{
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
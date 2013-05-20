#include "LPSolver.h"
#include "SubproblemSolver.h"
#include "Bucket.h"
#include "Route.h"

#include <sstream>
#include <vector>
#include <queue>

string itos(int i) {stringstream s; s << i; return s.str(); }

LPSolver::LPSolver(ProblemData *d) : data(d)
{
	//Create Gurobi enviroment and model
	env = new GRBEnv();
	model = new GRBModel(*env);

	//Objective function value of auxiliary variables
	bigM = 10000;

	//Initialize subproblem solver vector
	spSolvers = vector<SubproblemSolver*>(data->numEquipments);
	routeCounter = 0;
	
}

LPSolver::~LPSolver()
{
	//Destroy vector of subproblem solvers
	spSolvers.clear();

	//Destroy variable hash map
	vHash.clear();

	//Destroy constraint hash map
	cHash.clear();
	
	//Destroy Gurobi Enviroment
	delete env;
}

int LPSolver::solve()
{
	//Build problem graph representation
	buildProblemNetwork();

	//Build the initial model
	buildInitialModel();

	//Create a subproblem solver for each equipment (vehicle) type
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		spSolvers[eqType]  = new SubproblemSolver(this,data,eqType,SubproblemType::QROUTE);
	}

	//Run Branch and Price (DFS)
	int status = BaP();

	return status;
}

void LPSolver::buildInitialModel()
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

			vHash[y] = model->addVar(0.0,1.0,0.0,GRB_CONTINUOUS,y.toString());
		}
	}

	//xVars and bAuxVars
	queue<Vertex*> myQueue;
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		//Make a DFS over the problem network
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
							vHash[x] = model->addVar(0.0,1.0,transitionTime,GRB_CONTINUOUS, x.toString());
						}

						//create bAuxVar
						Variable b;
						b.setType(V_BAUX);						
						b.setStartJob(o->getJob());
						b.setEndJob(d->getJob());
						b.setTime(o->getTime());
						b.setEquipmentTipe(eqType);

						if(vHash.find(b) == vHash.end()){
							vHash[b] = model->addVar(0.0,1.0,bigM,GRB_CONTINUOUS, b.toString());
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

		vHash[f] = model->addVar(0.0,e->getNumMachines(),bigM,GRB_CONTINUOUS, f.toString());
	}

	model->update();
	//----------------------

	//----------------------
	//CREATE CONSTRAINTS
	//----------------------

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

				if(vHash.find(y) != vHash.end())
					expr += vHash[y];
			}
			cHash[c] = model->addConstr(expr == 1, c.toString());
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

			expr -= vHash[y];

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

						if(vHash.find(v) != vHash.end())
							expr += vHash[v];
					}
					cHash[c] = model->addConstr(expr == 0, c.toString());
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
			expr += vHash[f];
			cHash[c] = model->addConstr(expr == e->getNumMachines(), c.toString());
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
						expr += vHash[v];
						expr -= vHash[b];
						cHash[c] = model->addConstr(expr == 0, c.toString());
					}
				}
			}
		}
	}

	model->update();
	//----------------------
	
	model->write("modelo.lp");
}

int LPSolver::solveCurrentLPByColumnGeneration()
{
	int status = GRB_INPROGRESS;

	//Reset subproblem solvers and collapse vertices
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		spSolvers[eqType]->reset();
		spSolvers[eqType]->collapseVertices();
	}				

	//Dynamically solve the model by column generation.
	Route *myRoute;
	Edge *myEdge;
	Variable v;
	VariableHash::iterator vit;
	Constraint c1, c2;
	ConstraintHash::iterator cit1,cit2;
	bool end = false;
	while(!end){	
		//Solve current model
		model->optimize();
		status = model->get(GRB_IntAttr_Status);

		int cont = 0;

		if(status != GRB_OPTIMAL)
			return status; 

		//Print solution
		cout << "New Solution: " << endl;
		vit = vHash.begin();
		for(; vit != vHash.end(); vit++){
			v = vit->first;
			double sol = vit->second.get(GRB_DoubleAttr_X);
			if(sol > 0.000001)
				cout << v.toString() << " = " << sol << endl;
		}


		//Generate routes for each equipment type
		for(int eqType = 0; eqType < data->numEquipments; eqType++){
			SubproblemSolver *mySpSolver = spSolvers[eqType];
			mySpSolver->solve();
			if(mySpSolver->isInfeasible()){
				return GRB_INFEASIBLE;
			}
			//Add route(s) to the model
			if(mySpSolver->routes.size() > 0){
				//ADD ROUTE(S) TO THE MODEL
				vector<Route*>::iterator rit = mySpSolver->routes.begin();
				for(; rit != mySpSolver->routes.end(); rit++){
					myRoute = (*rit);
					cout << myRoute->toString() << endl;

					//Create lambda variable
					v.reset();
					v.setType(V_LAMBDA);
					v.setRouteNum(routeCounter);
					v.setEquipmentTipe(eqType);

					if(vHash.find(v) == vHash.end()){
						vHash[v] = model->addVar(0.0,1.0,0.0,GRB_CONTINUOUS,v.toString());
						model->update(); //TODO: verify if update is needed at this point.

						//Add column (Card constraints)
						c1.reset();
						c1.setType(C_CARD);
						c1.setEquipmentType(eqType);

						cit1 = cHash.find(c1);
						if(cit1 != cHash.end())
							model->chgCoeff(cit1->second,vHash[v],1.0);

						//Add column (Explicit Master constraints)
						vector<Edge*>::iterator eit = myRoute->edges.begin();
						for(; eit != myRoute->edges.end(); eit++){
							myEdge = (*eit);

							//Explicit constraints
							c2.reset();
							c2.setType(C_EXPLICIT);
							c2.setStartJob(myEdge->getStartJob());
							c2.setEndJob(myEdge->getEndJob());
							c2.setTime(myEdge->getTime());
							c2.setEquipmentType(myRoute->getEquipmentType());

							cit2 = cHash.find(c2);
							if(cit2 != cHash.end())
								model->chgCoeff(cit2->second, vHash[v], -1.0);
						}

						//Update the model to include new column
						model->update();
						model->write("modelo.lp");
						routeCounter++;
						cont++;
					}
				}
			}
		}

		//If no routes where generated, the current lp solution is optimal
		if(cont <= 0){
			end = true;
		}
	}

	return status;
}

int LPSolver::BaP()
{
	//TODO: implement Branch and Price method.
	int result = solveCurrentLPByColumnGeneration();

	return result;
}

void LPSolver::buildProblemNetwork()
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
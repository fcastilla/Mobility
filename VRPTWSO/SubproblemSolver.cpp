#include "gurobi_c++.h"
#include "SubproblemSolver.h"
#include "LPSolver.h"
#include "Variable.h"
#include "Constraint.h"
#include <queue>

SubproblemSolver::SubproblemSolver(LPSolver *mPtr, ProblemData *d, int e, SubproblemType m) : master(mPtr), data(d), eqType(e), method(m)
{
	infinityValue = 1e13;
	totalJobs = data->numJobs;

	//Initialize fixatedVars vector
	fixatedVars = vector<int>(totalJobs,0);	

	//Initialize reduced costs matrix
	reducedCosts = vector<vector<vector<double> > >(totalJobs, 
		vector<vector<double> >(totalJobs, vector<double>(data->horizonLength + 1)));

	//Initialize FMatrix
	fMatrix = vector<vector<Bucket*> >(totalJobs, vector<Bucket*>(data->horizonLength + 1));
	for(int j=0; j < totalJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			switch(method){
				case QROUTE:
					fMatrix[j][t] = new QRouteBucket();
					break;
				case QROUTE_NOLOOP:
					fMatrix[j][t] = new QRouteNoLoopBucket();
					break;
			}

			fMatrix[j][t]->setJob(j);
			fMatrix[j][t]->setTime(t);
		}
	}

	//Initialize routes vector
	routes = vector<Route*>();

}

SubproblemSolver::~SubproblemSolver()
{
	//fixatedVars
	fixatedVars.clear();

	//Reduced Costs Matrix
	reducedCosts.clear();

	//FMatrix
	fMatrix.clear();

	//Routes
	routes.clear();
}

void SubproblemSolver::reset()
{
	//reset reduced costs matrix and fMatrix
	for(int j=0; j < totalJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			fMatrix[j][t]->reset();
			for(int i=0; i < totalJobs; i++){
				reducedCosts[j][i][t] = 0;
			}
		}
	}

	//erase all previously generated routes
	routes.clear();
}

void SubproblemSolver::collapseVertices()
{
	infeasible = false;
	fixatedVars.clear();

	Variable v;
	VariableHash::iterator vit;
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

			if(!visited[o->getJob()][o->getTime()]){
				visited[o->getJob()][o->getTime()] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				for(; it != o->getAdjacenceList(eqType).end(); it++)				{
					d = (*it);

					if(o->getJob() != d->getJob()){ //not waiting
						//Verify associated variable bound
						v.reset();
						v.setType(V_X);
						v.setStartJob(o->getJob());
						v.setEndJob(d->getJob());
						v.setTime(o->getTime());
						v.setEquipmentTipe(eqType);

						vit = master->vHash.find(v);
						if(vit != master->vHash.end()){
							//Get variable
							GRBVar myVar = vit->second;
							//Get lowerbound
							double lb = myVar.get(GRB_DoubleAttr_LB);
							//If variable is fixated to 1, then collapse associated vertices (buckets)
							if(lb == 1){
								//"Colapse" associated vertices 
								fMatrix[o->getJob()][o->getTime()]->setSuccessor(fMatrix[d->getJob()][d->getTime()]);

								//Verify Conflicts (for the same job)
								fixatedVars[o->getJob()] ++;
								if(fixatedVars[o->getJob()] > 1){
									infeasible = true;
									return; //current master node is infeasible
								}
							}
						}
					}
					myQueue.push(d);
				}
			}
			myQueue.pop();
		}
		visited.clear();
	}
}

void SubproblemSolver::updateReducedCostsMatrix()
{
	Variable v;
	VariableHash::iterator vit;

	Constraint c;
	ConstraintHash::iterator cit;
	
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

			if(!visited[o->getJob()][o->getTime()]){
				visited[o->getJob()][o->getTime()] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				for(; it != o->getAdjacenceList(eqType).end(); it++){
					d = (*it);
					int sw = 0;

					if(o->getJob() != d->getJob()){ //not waiting
						//Verify associated variable bound
						v.reset();
						v.setType(V_X);
						v.setStartJob(o->getJob());
						v.setEndJob(d->getJob());
						v.setTime(o->getTime());
						v.setEquipmentTipe(eqType);

						vit = master->vHash.find(v);
						if(vit != master->vHash.end()){ //Variable exist in current model
							GRBVar myVar = vit->second;
							double ub = myVar.get(GRB_DoubleAttr_UB);
							if(ub == 0){
								reducedCosts[o->getJob()][d->getJob()][o->getTime()] = infinityValue;
								sw = 1;
							}
						}

						//if arc was not fixated to 0, retrieve its associated reduced cost
						if(sw == 0){
							c.reset();
							c.setType(C_EXPLICIT);
							c.setStartJob(o->getJob());
							c.setEndJob(d->getJob());
							c.setTime(o->getTime());
							c.setEquipmentType(eqType);

							cit = master->cHash.find(c);
							if(cit != master->cHash.end()){ //constraint exist
								GRBConstr myConstr = cit->second;
								//get shadow price
								double pi = myConstr.get(GRB_DoubleAttr_Pi);
								reducedCosts[o->getJob()][d->getJob()][o->getTime()] = pi;
							}
						}
					}
					myQueue.push(d);
				}
			}
			myQueue.pop();
		}
		visited.clear();
	}	
}

void SubproblemSolver::solve()
{
	//Reset buckets and reduced costs matrix
	reset();

	//Update reduced costs matrix
	updateReducedCostsMatrix();

	//If there are conflicts there is no solution.
 	if(infeasible)
		return;
	
	//DYNAMIC PROGRAMING
	Vertex *p;
	int pJob, pTime;
	
	vector<Vertex*> incidenceList, adjacenceList;	
	vector<Vertex*>::iterator itVertex;

	fMatrix[0][0]->addLabel(new Label(0,0,0));
	for(int t=1; t <= data->horizonLength; t++){
		for(int j=0; j < totalJobs; j++){
			//Verify that current bucket is reachable
			if(data->problemNetwork[j][t] != nullptr && data->problemNetwork[j][t]->getIncidenceList(eqType).size() > 0){
				//Node is reachable, iterate through incidence list
				incidenceList = data->problemNetwork[j][t]->getIncidenceList(eqType);
				itVertex = incidenceList.begin();
				for(; itVertex != incidenceList.end(); itVertex++){
					p = (*itVertex);
					pJob = p->getJob();
					pTime = p->getTime();

					if(pJob == j) continue;

					//Evaluate [pJob,pTime] Bucket
					if(fMatrix[pJob][pTime]->getSuccessor() != nullptr && (fMatrix[pJob][pTime]->getSuccessor()->getJob() != j || fMatrix[pJob][pTime]->getSuccessor()->getTime() != t)) continue;
					fMatrix[j][t]->evaluate(fMatrix[pJob][pTime]->getLabels(), reducedCosts[pJob][j][pTime], (fMatrix[pJob][pTime]->getSuccessor() == fMatrix[j][t]));
				}
			}

			//Evaluate t-1 Bucket (Waiting)
			if(j != 0 && fMatrix[j][t-1]->getLabels().size() > 0)
				fMatrix[j][t]->evaluate(fMatrix[j][t-1]->getLabels(), 0, false);
		}
	}
	
	//TODO: build more than 1 route
	//BUILD ROUTE
	//verify that there is a solution (with negative reduced cost)
	Label *bestLabel = fMatrix[0][data->horizonLength]->getBestLabel();
	if(bestLabel == nullptr || bestLabel->getCost() >= 0){
		return;
	}

	Route *myRoute = new Route(eqType);
	myRoute->setCost(bestLabel->getCost());
	//TODO: add other dual variable cost to the route

	Label *currentLabel, *previousLabel;
	currentLabel = bestLabel;
	previousLabel = currentLabel->getPredecessor();
	while(previousLabel != nullptr){
		if(currentLabel->getJob() != previousLabel->getJob()){ //not waiting
			myRoute->edges.push_back(new Edge(previousLabel->getJob(),currentLabel->getJob(), previousLabel->getTime()));
		}
		currentLabel = previousLabel;
		previousLabel = currentLabel->getPredecessor();		
	}

	routes.push_back(myRoute);

}

#include "gurobi_c++.h"
#include "SubproblemSolver.h"
#include "Solver.h"
#include "Node.h"
#include "Variable.h"
#include "Constraint.h"
#include <queue>

SubproblemSolver::SubproblemSolver(ProblemData *d, SubproblemType m) : data(d), method(m)
{
	infinityValue = 1e13;

	//Initialize fixatedVars vector
	fixatedVars = vector<int>(data->numJobs,0);	

	//Initialize reduced costs matrix
	reducedCosts = vector<vector<vector<double> > >(data->numJobs, 
		vector<vector<double> >(data->numJobs, vector<double>(data->horizonLength + 1)));

	//Initialize FMatrix
	fMatrix = vector<vector<Bucket*> >(data->numJobs, vector<Bucket*>(data->horizonLength + 1));
	for(int j=0; j < data->numJobs; j++){
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
	infeasible = false;

	//reset reduced costs matrix and fMatrix
	for(int j=0; j < data->numJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			fMatrix[j][t]->reset();
			for(int i=0; i < data->numJobs; i++){
				reducedCosts[j][i][t] = 0;
			}
		}
	}

	//erase all previously generated routes
	routes.clear();
}

void SubproblemSolver::collapseVertices(Node *node, int eqType)
{
	infeasible = false;
	fixatedVars.clear();

	Variable v;
	VariableHash::iterator vit;
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
			myQueue.pop();

			if(!visited[o->getJob()][o->getTime()]){
				visited[o->getJob()][o->getTime()] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				for(; it != o->getAdjacenceList(eqType).end(); it++){
					d = (*it);

					if(o->getJob() != d->getJob()){ //not waiting
						//Verify associated variable bound
						v.reset();
						v.setType(V_X);
						v.setStartJob(o->getJob());
						v.setEndJob(d->getJob());
						v.setTime(o->getTime());
						v.setEquipmentTipe(eqType);

						double lb = node->getVarLB(v);
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
					myQueue.push(d);
				}
			}
		}
		visited.clear();
	}
}

void SubproblemSolver::updateReducedCostsMatrix(Node *node, int eqType)
{
	Variable v;
	VariableHash::iterator vit;

	Constraint c;
	ConstraintHash::iterator cit;
	
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
			myQueue.pop();

			if(!visited[o->getJob()][o->getTime()]){
				visited[o->getJob()][o->getTime()] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::const_iterator it = o->getAdjacenceList(eqType).begin();
				for(; it != o->getAdjacenceList(eqType).end(); it++){
					d = (*it);
					int sw = 0;

					v.reset();
					v.setType(V_X);
					v.setStartJob(o->getJob());
					v.setEndJob(d->getJob());
					v.setTime(o->getTime());
					v.setEquipmentTipe(eqType);

					if(o->getJob() != d->getJob()){ //not waiting
						double rc = node->getArcReducedCost(v);
						reducedCosts[o->getJob()][d->getJob()][o->getTime()] = rc;
					}
					myQueue.push(d);
				}
			}
		}
		visited.clear();
	}	
}

void SubproblemSolver::solve(Node *node, int eqType)
{
	//Reset buckets and reduced costs matrix
	reset();

	//If there are conflicts there is no solution.
 	if(infeasible)
		return;

	//Update reduced costs matrix
	updateReducedCostsMatrix(node, eqType);	
	
	//DYNAMIC PROGRAMING
	Vertex *p;
	int pJob, pTime;
	
	vector<Vertex*> incidenceList, adjacenceList;	
	vector<Vertex*>::iterator itVertex;

	fMatrix[0][0]->addLabel(new Label(0,0,0));
	for(int t=1; t <= data->horizonLength; t++){
		for(int j=0; j < data->numJobs; j++){
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
	if(bestLabel == nullptr){
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

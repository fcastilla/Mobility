#include "gurobi_c++.h"
#include "SubproblemSolver.h"
#include "Solver.h"
#include "Node.h"
#include "Variable.h"
#include "Constraint.h"

#include <iostream>
#include <iomanip>
#include <queue>

SubproblemSolver::SubproblemSolver(ProblemData *d, SubproblemType m) : data(d), method(m)
{
	infinityValue = 1e13;

	//Initialize fixatedVars vector
	fixatedVars = vector<int>(data->numJobs,0);	

	//Visited vertex vector
	vector<vector<bool>> visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

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

	//Global parameters
	parameters = GlobalParameters::getInstance();

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
	Variable v;

	infeasible = false;
	fixatedVars.clear();

	queue<Vertex*> myQueue;	
	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		//Make a BFS over the problem network
		Vertex *o, *d;
		o = data->problemNetwork[0][0]; //depot

		myQueue.push(o);
		visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

		while(myQueue.size() > 0){
			o = myQueue.front();			
			myQueue.pop();
			int oJob = o->getJob();
			int oTime = o->getTime();

			if(!visited[oJob][oTime]){
				visited[oJob][oTime] = true;

				//Iterate through adjacence list for job o and eqType
				vector<Vertex*>::iterator it = o->getAdjacenceList(eqType).begin();
				vector<Vertex*>::iterator eit = o->getAdjacenceList(eqType).end();
				for(; it != eit; it++){
					d = (*it);
					int dJob = d->getJob();
					int dTime = d->getTime();

					if(oJob != dJob){ //not waiting
						//Verify associated variable bound
						v.reset();
						v.setType(V_X);
						v.setStartJob(oJob);
						v.setEndJob(dJob);
						v.setTime(oTime);
						v.setEquipmentTipe(eqType);

						double lb = node->getVarLB(v);
						//If variable is fixated to 1, then collapse associated vertices (buckets)
						if(lb == 1){
							//"Colapse" associated vertices 
							fMatrix[oJob][oTime]->setSuccessor(fMatrix[dJob][dTime]);

							//Verify Conflicts (for the same job)
							fixatedVars[oJob] ++;
							if(fixatedVars[oJob] > 1){
								infeasible = true;
								return; //current master node is infeasible
							}
						}
					}
					myQueue.push(d);
				}
			}
		}
	}
}

void SubproblemSolver::updateReducedCostsMatrix(Node *node, int eqType)
{
	Variable v;

	int oJob, oTime;
	int dJob;

	vector<Vertex*>::iterator it, eit;
		
	queue<Vertex*> myQueue;	
	visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

	for(int eqType=0; eqType < data->numEquipments; eqType++){
		Equipment *e = data->equipments[eqType];

		//Make a BFS over the problem network
		Vertex *o, *d;		

		o = data->problemNetwork[0][0]; //depot
		myQueue.push(o);

		while(myQueue.size() > 0){
			o = myQueue.front();			
			myQueue.pop();

			oJob = o->getJob();
			oTime = o->getTime();

			if(!visited[oJob][oTime]){
				visited[oJob][oTime] = true;

				//Iterate through adjacence list for job o and eqType
				it = o->getAdjacenceList(eqType).begin();
				eit = o->getAdjacenceList(eqType).end();

				for(; it != eit; it++){
					d = (*it);
					dJob = d->getJob();

					v.reset();
					v.setType(V_X);
					v.setStartJob(oJob);
					v.setEndJob(dJob);
					v.setTime(oTime);
					v.setEquipmentTipe(eqType);

					if(oJob != d->getJob()){ //not waiting
						reducedCosts[oJob][dJob][oTime] = node->getArcReducedCost(v);
					}

					myQueue.push(d);
				}
			}
		}
	}	

	/*cout << "Matriz Reducidos: " << endl;
	for(int i=0; i < data->numJobs; i++){
		for(int j=0; j < data->numJobs; j++){
			for(int t=0; t < data->horizonLength; t++){
				cout << setw(8) << reducedCosts[i][j][t];
			}
			cout << endl;
		}
		cout << "---------------------------------------" << endl;
	}*/
}

void SubproblemSolver::solve(Node *node, int eqType, int maxRoutes)
{
	//Reset buckets and reduced costs matrix
	reset();

	//If there are conflicts there is no solution.
 	if(infeasible)
		return;

	//Update reduced costs matrix
	//updateReducedCostsMatrix(node, eqType);	
	double routeUseCost = node->getRouteUseReducedCost(eqType);
	
	//Dynamic Programming
	Vertex *p, *next;
	int pJob, pTime;
	int cJob, cTime, nJob, nTime;
	double rc;

	vector<Vertex*> incidenceList, adjacenceList;	
	vector<Vertex*>::iterator itVertex, eitVertex;

	fMatrix[0][0]->addLabel(new Label(0,0,0));

	//Reaching algorithm
	Vertex *currentVertex, *nextVertex;
	set<Vertex*,VertexComparator>::iterator vit = data->vertexSet.begin();
	set<Vertex*,VertexComparator>::iterator veit = data->vertexSet.end();

	while(vit != veit){
		currentVertex = (*vit);
		cJob = currentVertex->getJob();
		cTime = currentVertex->getTime();

		adjacenceList = currentVertex->getAdjacenceList(eqType);
		itVertex = adjacenceList.begin();
		eitVertex = adjacenceList.end();
		for(; itVertex != eitVertex; itVertex++){
			nextVertex = (*itVertex);
			nJob = nextVertex->getJob();
			nTime = nextVertex->getTime();

			rc = (cJob == nJob)? 0 : node->getArcReducedCost(cJob,nJob,cTime,eqType);

			fMatrix[nJob][nTime]->evaluate(fMatrix[cJob][cTime]->getLabels(), rc, false);
		}
		vit++;
	}
	

	//for(int t=1; t <= data->horizonLength; t++){
	//	for(int j=0; j < data->numJobs; j++){
	//		//Verify that current bucket is reachable
	//		if(data->problemNetwork[j][t] != nullptr && data->problemNetwork[j][t]->getIncidenceList(eqType).size() > 0){
	//			//Node is reachable, iterate through incidence list
	//			incidenceList = data->problemNetwork[j][t]->getIncidenceList(eqType);
	//			itVertex = incidenceList.begin();
	//			eitVertex = incidenceList.end();
	//			for(; itVertex != eitVertex; itVertex++){
	//				p = (*itVertex);
	//				pJob = p->getJob();
	//				pTime = p->getTime();

	//				if(pJob == j) continue;

	//				//Evaluate [pJob,pTime] Bucket
	//				//if(fMatrix[pJob][pTime]->getSuccessor() != nullptr && (fMatrix[pJob][pTime]->getSuccessor()->getJob() != j || fMatrix[pJob][pTime]->getSuccessor()->getTime() != t)) continue;
	//				//fMatrix[j][t]->evaluate(fMatrix[pJob][pTime]->getLabels(), reducedCosts[pJob][j][pTime], (fMatrix[pJob][pTime]->getSuccessor() == fMatrix[j][t]));
	//				//fMatrix[j][t]->evaluate(fMatrix[pJob][pTime]->getLabels(), reducedCosts[pJob][j][pTime], false);
	//				fMatrix[j][t]->evaluate(fMatrix[pJob][pTime]->getLabels(), node->getArcReducedCost(pJob,j,pTime,eqType), false);
	//			}
	//		}

	//		//Evaluate t-1 Bucket (Waiting)
	//		if(j != 0 && fMatrix[j][t-1]->getLabels().size() > 0)
	//			fMatrix[j][t]->evaluate(fMatrix[j][t-1]->getLabels(), 0, false);
	//	}
	//}

	//BUILD ROUTES
	Route *myRoute;
	int contRoutes = 0;
	set<Label*,LabelComparator>::iterator it = fMatrix[0][data->horizonLength]->getLabels().begin();
	set<Label*,LabelComparator>::iterator eit = fMatrix[0][data->horizonLength]->getLabels().end();
	for(; it != eit; it++){
		Label *currentLabel = (*it);
		if(currentLabel == nullptr){
			return;
		}

		myRoute = new Route(eqType);
		myRoute->setCost(currentLabel->getCost() - routeUseCost);

		if(myRoute->getCost() >= -parameters->getEpsilon()) break; //labels are ordered by reduced cost.

		Label *previousLabel = currentLabel->getPredecessor();
		while(previousLabel != nullptr){
			if(currentLabel->getJob() != previousLabel->getJob()){ //not waiting
				myRoute->edges.push_back(new Edge(previousLabel->getJob(),currentLabel->getJob(), previousLabel->getTime()));
			}
			currentLabel = previousLabel;
			previousLabel = currentLabel->getPredecessor();		
		}

		routes.push_back(myRoute);
		contRoutes++;
		if(contRoutes >= parameters->getMaxRoutes())
			break;
	}

}

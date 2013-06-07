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

	//Global parameters
	parameters = GlobalParameters::getInstance();

	//Initialize fixatedVars vector
	fixatedVars = vector<int>(data->numJobs,0);	

	//Visited vertex vector
	vector<vector<bool>> visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

	//Initialize FMatrix
	fMatrix = vector<vector<Bucket*> >(data->numJobs, vector<Bucket*>(data->horizonLength + 1));
	for(int j=0; j < data->numJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			switch(method){
				case QROUTE:
					if(j==0 && t == data->horizonLength)
						fMatrix[j][t] = new QRouteBucket(parameters->getMaxRoutes());
					else
						fMatrix[j][t] = new QRouteBucket(1);

					break;
				case QROUTE_NOLOOP:
					if(j==0 && t == data->horizonLength)
						fMatrix[j][t] = new QRouteNoLoopBucket(parameters->getMaxRoutes());
					else
						fMatrix[j][t] = new QRouteNoLoopBucket(parameters->getMaxRoutes());
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

void SubproblemSolver::solve(Node *node, int eqType, int maxRoutes)
{
	//Reset buckets and reduced costs matrix
	reset();

	//If there are conflicts there is no solution.
 	if(infeasible)
		return;
	
	//Dynamic Programming
	Vertex *p, *next;
	int pJob, pTime;
	int cJob, cTime, nJob, nTime;
	double rc;

	vector<Vertex*> incidenceList, adjacenceList;	
	vector<Vertex*>::iterator itVertex, eitVertex;

	fMatrix[0][0]->addLabel(new Label(0,0,0));

	//Reaching algorithm
	Equipment *e = data->equipments[eqType];
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

			//Get objective function value for arc Xjite (distance)
			double dist = e->getNotRoundedTransitionTime(cJob,nJob);

			rc = node->getArcReducedCost(cJob,nJob,cTime,nTime,eqType,dist);

			fMatrix[nJob][nTime]->evaluate(fMatrix[cJob][cTime]->getLabels(), rc, false);
		}
		vit++;
	}

	//BUILD ROUTES
	Route *myRoute;
	int contRoutes = 0;
	set<Label*,LabelComparator>::iterator it = fMatrix[0][data->horizonLength]->getLabels().begin();
	set<Label*,LabelComparator>::iterator eit = fMatrix[0][data->horizonLength]->getLabels().end();
	while(it != eit){
		Label *currentLabel = (*it);
		if(currentLabel == nullptr){
			return;
		}

		myRoute = new Route(eqType);
		myRoute->setReducedCost(currentLabel->getCost());

		if(myRoute->getReducedCost() >= -parameters->getEpsilon()) break; //labels are ordered by reduced cost.

		double cost = 0.0;
		Label *previousLabel = currentLabel->getTimePredecessor();
		while(currentLabel != nullptr){
			//if(currentLabel->getJob() == 0 || currentLabel->getJob() != previousLabel->getJob()){ //not waiting
				/*cout << endl <<  "(" << currentLabel->getJob() << "," << currentLabel->getTime() << ")";
				cout << " Valor do Label: " << setw(8) << currentLabel->getCost(); 
				if(currentLabel->getTime() != 230){
					cout << " Valor Dual_" << currentLabel->getJob() 
						<< "," << currentLabel->getTime() << "," << eqType << ": " 
						<< node->getDualVal(currentLabel->getJob(),currentLabel->getTime(),eqType) << endl;
				}*/

				if(previousLabel != nullptr){
					cost += e->getNotRoundedTransitionTime(previousLabel->getJob(),currentLabel->getJob());
					myRoute->edges.push_back(data->getEdge(previousLabel->getJob(), currentLabel->getJob(), previousLabel->getTime()));
				}
			//}
			currentLabel = previousLabel;
			if(previousLabel != nullptr)
				previousLabel = currentLabel->getTimePredecessor();		
		}
		myRoute->setCost(cost);

		routes.push_back(myRoute);
		contRoutes++;
		if(contRoutes >= parameters->getMaxRoutes())
			break;

		it++;
	}

}

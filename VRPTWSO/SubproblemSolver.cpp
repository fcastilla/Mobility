#include "gurobi_c++.h"
#include "SubproblemSolver.h"
#include "Solver.h"
#include "Node.h"
#include "Variable.h"
#include "Constraint.h"

#include <iostream>
#include <iomanip>
#include <queue>

SubproblemSolver::SubproblemSolver(ProblemData *d) : data(d)
{
	infinityValue = 1e13;

	//Global parameters
	parameters = GlobalParameters::getInstance();

	//Visited vertex vector
	vector<vector<bool>> visited = vector<vector<bool>>(data->numJobs, vector<bool>(data->horizonLength+1,false));

	//Initialize FMatrix
	fMatrix = vector<vector<Bucket*> >(data->numJobs, vector<Bucket*>(data->horizonLength + 1));
	for(int j=0; j < data->numJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			if(j==0 && t == data->horizonLength)
				fMatrix[j][t] = new QRouteNoLoopBucket(parameters->getMaxRoutes());
			else
				fMatrix[j][t] = new QRouteNoLoopBucket(2);

			fMatrix[j][t]->setJob(j);
			fMatrix[j][t]->setTime(t);
		}
	}

	//Initialize routes vector
	routes = vector<Route*>();

}

SubproblemSolver::~SubproblemSolver()
{
	//Routes
	routes.clear();
}

void SubproblemSolver::reset()
{
	//reset reduced costs matrix and fMatrix
	for(int j=0; j < data->numJobs; j++){
		for(int t=0; t <= data->horizonLength; t++){
			fMatrix[j][t]->reset();
		}
	}

	//erase all previously generated routes
	routes.clear();
}

void SubproblemSolver::solve(Node *node, int eqType)
{
	//Reset buckets and reduced costs matrix
	reset();
	
	//Dynamic Programming
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
		while(itVertex != eitVertex){
			nextVertex = (*itVertex);
			nJob = nextVertex->getJob();
			nTime = nextVertex->getTime();

			double dist = e->getNotRoundedTransitionTime(cJob,nJob);

			rc = (cJob == nJob)? 0 : node->getArcReducedCost(cJob,nJob,cTime,nTime,eqType,dist);
			fMatrix[nJob][nTime]->evaluate(fMatrix[cJob][cTime]->getLabels(), rc, false);

			itVertex++;
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

		if(myRoute->getReducedCost() >= (parameters->getEpsilon()*-1)) break; //labels are ordered by reduced cost.

		double cost = 0.0;
		Label *previousLabel = currentLabel->getPredecessor();
		while(currentLabel != nullptr){
			if(currentLabel->getJob() == 0 || currentLabel->getJob() != previousLabel->getJob()){ //not waiting
				if(previousLabel != nullptr){
					cost += e->getNotRoundedTransitionTime(previousLabel->getJob(),currentLabel->getJob());
					myRoute->edges.push_back(data->getEdge(previousLabel->getJob(), currentLabel->getJob(), previousLabel->getTime()));
				}
			}
			currentLabel = previousLabel;
			if(previousLabel != nullptr)
				previousLabel = currentLabel->getPredecessor();		
		}
		myRoute->setCost(cost);

		routes.push_back(myRoute);
		contRoutes++;
		if(contRoutes >= parameters->getMaxRoutes())
			break;

		it++;
	}

}

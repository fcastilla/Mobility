#include "Bucket.h"

//Vertex
Vertex::Vertex(int numEq) : job(-1), time(-1)
{
	adjacenceList = vector<vector<Vertex*>>(numEq, vector<Vertex*>());
	incidenceList = vector<vector<Vertex*>>(numEq, vector<Vertex*>());
}

Vertex::~Vertex()
{
	adjacenceList.clear();
	incidenceList.clear();
}

void Vertex::addInicidentVertex(int eqType, Vertex *v)
{
	incidenceList[eqType].push_back(v);
}

void Vertex::addAdjacentVertex(int eqType, Vertex *v)
{
	adjacenceList[eqType].push_back(v);
}

//BUCKET
void Bucket::reset()
{
	//Free memory
	labels.clear();

	successor = nullptr; 
}

//QROUTE BUCKET
void QRouteBucket::evaluate(const vector<Label*> &oLabels, double rCost, bool fix)
{
	//Create label
	Label *myLabel = new Label(job,time);

	//Evaluate best label of oLabels (if such exist)
	if(oLabels.size() > 0)
	{
		Label *pLabel = oLabels[0];

		//set label cost
		myLabel->setCost(pLabel->getCost() + rCost);
		myLabel->setPredecessor(pLabel);
		myLabel->setFixed(fix);
	}

	//Dominance Check (same bucket)
	if(labels.size() <= 0)
		labels.push_back(myLabel);
	else
	{
		Label *bestLabel = labels[0];
		if(myLabel->getCost() < bestLabel->getCost() || myLabel->isFixed())
		{
			labels[0] = myLabel;
			delete bestLabel;
		}			
	}
}

Label* QRouteBucket::getBestLabel()
{
	if(labels.size() > 0)
		return labels[0];
	return nullptr;
}

//QROUTENOLOOP BUCKET
void QRouteNoLoopBucket::evaluate(const vector<Label*> &oLabels, double rCost, bool fix)
{
	//Create Label
	Label *myLabel = new Label(job,time);


}

Label* QRouteNoLoopBucket::getBestLabel()
{
	if(labels.size() > 0)
		return labels[0];
	return nullptr;
}

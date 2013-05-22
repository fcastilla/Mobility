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

//Label
bool Label::operator<(const Label& other) const{
	if(this->getCost() < other.getCost())
		return true;

	return false;
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
	if(labels.size() == 0)
		labels.push_back(myLabel);
	else
	{
		Label *bestLabel = labels[0];
		if(myLabel < bestLabel || myLabel->isFixed())
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

	//Get best label of predecessor
	int pSize = oLabels.size() -1;
	for(int i=0; i < 2; i++){
		if(i > pSize) break;

		Label *pLabel = oLabels[i];
		//Avoid Loop
		if(job != 0){
			if(pLabel->getPredecessor() != nullptr && pLabel->getPredecessor()->getJob() == job) continue;
		}
		//No Loop at this point 		
		myLabel->setCost(pLabel->getCost() + rCost);
		myLabel->setPredecessor(pLabel);
		myLabel->setFixed(fix);

		break;
	}

	//Dominance Check (same bucket)
	if(labels.size() == 0)
		labels.push_back(myLabel);
	else
	{
		//Evaluate best label
		Label *bestLabel = labels[0];
		if(*myLabel < *bestLabel){
			Label *temp = labels[0];
			labels[0] = myLabel;
			if(labels.size() >= 2){
				delete labels[1];
				labels[1] = temp;
			}else{
				labels.push_back(temp);
			}
		}else{ //Evaluate second best label
			if(labels.size() >= 2){
				if(*myLabel < *labels[1])
					labels[1] = myLabel;
			}else{
				labels.push_back(myLabel);
			}
		}
	}
}

Label* QRouteNoLoopBucket::getBestLabel()
{
	if(labels.size() > 0)
		return labels[0];
	return nullptr;
}

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
	//By cost
	if(this->getCost() < other.getCost())
		return true;
	else if(this->getCost() > other.getCost())
		return false;

	//By job
	if(this->getJob() < other.getJob())
		return true;
	else if(this->getJob() > other.getJob())
		return false;

	//By time
	if(this->getTime() < other.getTime())
		return true;
	else if(this->getTime() > other.getTime())
		return false;

	return false;
}

//BUCKET
void Bucket::reset()
{
	//Free memory
	set<Label*,LabelComparator>::iterator it = labels.begin();
	for(; it != labels.end(); it++){
		delete (*it);
	}
	labels.clear();

	successor = nullptr; 
}

//QROUTE BUCKET
void QRouteBucket::evaluate(set<Label*,LabelComparator> oLabels, double rCost, bool fix)
{
	//Create label
	Label *myLabel = new Label(job,time);

	//Evaluate best label of oLabels (if such exist)
	if(oLabels.size() > 0){
		Label *pLabel = (*oLabels.begin());

		//set label cost
		myLabel->setCost(pLabel->getCost() + rCost);
		myLabel->setPredecessor(pLabel);
		myLabel->setFixed(fix);
	}

	//Add the route
	pair<set<Label*,LabelComparator>::iterator,bool> success;
	success = labels.insert(myLabel);
	if(success.second == false){
		delete myLabel;
	}

	//Mantain the label structure of propper size
	if(labels.size() > maxSize){
		set<Label*,LabelComparator>::iterator it = labels.end();
		--it;
		Label *tempLabel = (*it);
		labels.erase(it);
		delete tempLabel;
	}

	//Evaluate best label of oLabels (if such exist)
	//if(oLabels.size() > 0){
	//	Label *pLabel = oLabels[0];

	//	//set label cost
	//	myLabel->setCost(pLabel->getCost() + rCost);
	//	myLabel->setPredecessor(pLabel);
	//	myLabel->setFixed(fix);
	//}

	////Dominance Check (same bucket)
	//if(labels.size() == 0)
	//	labels.push_back(myLabel);
	//else
	//{
	//	Label *bestLabel = labels[0];
	//	if(*myLabel < *bestLabel)
	//	{
	//		labels[0] = myLabel;
	//		delete bestLabel;
	//	}			
	//}
}

Label* QRouteBucket::getBestLabel()
{
	if(labels.size() > 0)
		return *labels.begin();
	return nullptr;
}

//QROUTENOLOOP BUCKET
void QRouteNoLoopBucket::evaluate(set<Label*,LabelComparator> oLabels, double rCost, bool fix)
{
	//Create Label
	Label *myLabel = new Label(job,time);

	//Get best label of predecessor
	set<Label*,LabelComparator>::iterator it = oLabels.begin();
	for(; it != oLabels.end(); it++){
		Label *pLabel = (*it);
		Label *predecessor = pLabel->getPredecessor();
		//Avoid Loop
		if(job != 0){
			if(predecessor != nullptr && predecessor->getJob() == job) continue;
		}
		//No Loop at this point 		
		myLabel->setCost(pLabel->getCost() + rCost);
		if(pLabel->getJob() != job)
			myLabel->setPredecessor(pLabel);
		else
			myLabel->setPredecessor(predecessor);
		myLabel->setFixed(fix);

		break;
	}

	//Add the route
	pair<set<Label*,LabelComparator>::iterator,bool> success;
	success = labels.insert(myLabel);
	if(success.second == false){
		delete myLabel;
	}

	//Mantain the label structure of propper size
	if(labels.size() > maxSize){
		set<Label*,LabelComparator>::iterator it = labels.end();
		--it;
		Label *tempLabel = (*it);
		labels.erase(it);
		delete tempLabel;
	}

	////Get best label of predecessor
	//int pSize = oLabels.size() -1;
	//for(int i=0; i < 2; i++){
	//	if(i > pSize) break;

	//	Label *pLabel = oLabels[i];
	//	Label *predecessor = pLabel->getPredecessor();
	//	//Avoid Loop
	//	if(job != 0){
	//		if(predecessor != nullptr && predecessor->getJob() == job) continue;
	//	}
	//	//No Loop at this point 		
	//	myLabel->setCost(pLabel->getCost() + rCost);
	//	if(pLabel->getJob() != job)
	//		myLabel->setPredecessor(pLabel);
	//	else
	//		myLabel->setPredecessor(predecessor);
	//	myLabel->setFixed(fix);

	//	break;
	//}

	//Dominance Check (same bucket)
	/*int numLabels = labels.size();
	if(numLabels == 0){
		labels.push_back(myLabel);
	}else if(numLabels == 1){
		labels.resize(2);
		if(myLabel->getCost() < labels[0]->getCost()){
			labels[1] = labels[0];
			labels[0] = myLabel;
		}else{
			labels[1] = myLabel;
		}
	}else if(numLabels == 2){
		if(myLabel->getCost() < labels[0]->getCost()){
			delete labels[1];
			labels[1] = labels[0];
			labels[0] = myLabel;
		}else if(myLabel->getCost() < labels[1]->getCost()){
			delete labels[1];
			labels[1] = myLabel;
		}else{
			delete myLabel;
		}
	}else{
		delete myLabel;
		labels.resize(2);
	}*/
}

Label* QRouteNoLoopBucket::getBestLabel()
{
	if(labels.size() > 0)
		return *labels.begin();
	return nullptr;
}

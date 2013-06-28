#include "Bucket.h"

bool isContained(Label *label1, Label *label2){
	if ((label1->unreachable & label2->unreachable) != label1->unreachable) return false;
	return true;
}

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
		//myLabel->unreachable = pLabel->unreachable;
		//myLabel->unreachable.set(job);

		if(pLabel->getJob() != job)
			myLabel->setPredecessor(pLabel);
		else
			myLabel->setPredecessor(predecessor);
		myLabel->setFixed(fix);

		break;
	}

	////Proper Dominance Test
	////verificar se o label é dominado por algum que ja esteja no bucket
	//set<Label*,LabelComparator>::iterator itLabel = labels.begin();
	//while(itLabel != labels.end() && (*itLabel)->getCost() <= myLabel->getCost()){
	//	if(isContained(*itLabel, myLabel)){
	//		delete myLabel;
	//		return;
	//	}
	//	itLabel++;
	//}

	//Add the label
	pair<set<Label*,LabelComparator>::iterator,bool> success;
	success = labels.insert(myLabel);
	if(success.second == false){
		delete myLabel;
	}

	////Verificar se o label domina a outro que já esteja no bucket
	//while(itLabel != labels.end()){
	//	if(isContained(myLabel, *itLabel)){
	//		delete *itLabel;
	//		itLabel = labels.erase(itLabel);
	//	}else{
	//		itLabel++;
	//	}
	//}

	//Mantain the label structure of propper size
	if(labels.size() > maxSize){
		set<Label*,LabelComparator>::iterator it = labels.end();
		--it;
		Label *tempLabel = (*it);
		labels.erase(it);
		delete tempLabel;
	}

}

Label* QRouteNoLoopBucket::getBestLabel()
{
	if(labels.size() > 0)
		return *labels.begin();
	return nullptr;
}



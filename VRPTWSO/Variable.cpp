#include "Variable.h"
#include "HashUtil.h"

#include <sstream>
#include <stdio.h>

using namespace std;

Variable::Variable()
{
	value = new double();
	score = new double();
	fractionality = new double();
	rank = new int();
	rc = new double();
	reset();
}

Variable::Variable(const Variable& var)
{
	value = new double();
	score = new double();
	fractionality = new double();
	rank = new int();
	rc = new double();
	*this = var;
}

void Variable::reset()
{
	type = V_ERROR;
	*value = -1.0;
	*score = 0.0;
	*fractionality = 1.0;
	*rc = 0.0;
	*rank = 0;
	sJob = -1;
	eJob = -1;
	time = -1; 
	arrivalTime = -1;
	eqType = -1;
	routeNumber = -1;
}

Variable::~Variable()
{
	reset();
	delete value;
	delete score;
	delete fractionality;
	delete rank;
	delete rc;
}

Variable& Variable::operator=(const Variable& var)
{
	this->type = var.getType();
	*(this->value) = var.getValue();
	*(this->fractionality) = var.getFractionality();
	*(this->rank) = var.getRank();
	this->sJob = var.getStartJob();
	this->eJob = var.getEndJob();
	this->time = var.getTime();
	this->arrivalTime = var.getArrivalTime();
	this->eqType = var.getEquipmentType();
	this->routeNumber = var.getRouteNumber();
	this->route = var.getRoute();

	return *this;
}

bool Variable::operator <(const Variable& var) const
{
	if( (int)this->getType() < (int) var.getType() )
		return true;
	else if( (int)this->getType() > (int) var.getType() )
		return false;

	//Compare by start job
	if(this->getStartJob() < var.getStartJob())
		return true;
	else if(this->getStartJob() > var.getStartJob())
		return false;

	//Compare by end job
	if(this->getEndJob() < var.getEndJob())
		return true;
	else if(this->getEndJob() > var.getEndJob())
		return false;

	//Compare by time
	if(this->getTime() < var.getTime())
		return true;
	else if(this->getTime() > var.getTime())
		return false;

	//Compare by equipment type
	if(this->getEquipmentType() < var.getEquipmentType())
		return true;
	else if(this->getEquipmentType() > var.getEquipmentType())
		return false;

	//Compare by route 
	if(this->getRouteNumber() < var.getRouteNumber())
		return true;
	else if(this->getRouteNumber() > var.getRouteNumber()) 
		return false;
	
	return false;
}

bool Variable::operator ==(const Variable& var) const
{
	return (!(*this < var) && !(var < *this));
}

string Variable::toString() const
{
	stringstream str;

	//type
	switch(type){
		case(V_Y):
			str << "Y_" << sJob << "," << time;
			break;
		case(V_X):
			str << "X_" << sJob << "," << eJob << "," << time << "," << eqType;
			break;
		case(V_W):
			str << "W_" << sJob << "," << time << "," << eqType;
			break;
		case(V_LAMBDA):
			str << "LAMBDA_" << eqType << "," << routeNumber;
			break;
		case(V_FAUX):
			str << "FAUX_" << eqType;
			break;
		case(V_BAUX):
			str << "BAUX_" << sJob << "," << time << "," << eqType;
			break;
		default:
			str << "UNDEFINED";
	}

	return str.str();
}

bool VariableHasher::operator()(const Variable& v1, const Variable& v2) const
{
	return (v1 < v2);
}

size_t VariableHasher::operator()(const Variable& v) const
{
	unsigned int sum = 0;

	//add the type contribution
	if (v.type != NULL){
		sum *= HASH_PRIME;
		sum += intHash(v.type);
	}

	//add start job contribution	
	if (v.sJob != -1){
		sum *= HASH_PRIME;
		sum += intHash(v.sJob);
	}

	//add end job contribution
	if (v.eJob != -1){
		sum *= HASH_PRIME;
		sum += intHash(v.eJob);
	}

	//add time contribution
	if (v.time != -1){
		sum *= HASH_PRIME;
		sum += intHash(v.time);
	}

	//add equipment type contribution
	if (v.eqType != -1){
		sum *= HASH_PRIME;
		sum += intHash(v.eqType);
	}
	
	//add route number contribution
	if (v.getRouteNumber() != -1){
		sum *= HASH_PRIME;
		sum += intHash(v.getRouteNumber());
	}

	return sum;
}


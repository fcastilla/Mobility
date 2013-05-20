#include "Constraint.h"
#include "HashUtil.h"

#include <stdio.h>
#include <sstream>

using namespace std;

Constraint::Constraint()
{
	reset();
}

Constraint::Constraint(const Constraint &cons)
{
	*this = cons;
}

Constraint::~Constraint()
{
	reset();
}

void Constraint::reset()
{
	type = C_ERROR;
	sJob = -1;
	eJob = -1;
	time = -1;
	eqType = -1;
}

Constraint& Constraint::operator= (const Constraint& cons)
{   
	this->type = cons.getType();
	this->sJob = cons.getStartJob();
	this->eJob = cons.getEndJob();
	this->time = cons.getTime();
	this->eqType = cons.getEquipmentType();

	return *this;
}

bool Constraint::operator< (const Constraint& cons) const
{
	if( (int)this->getType() < (int) cons.getType() )
		return true;
	else if( (int)this->getType() > (int) cons.getType() )
		return false;
		
	//Compare by start job
	if(this->getStartJob() < cons.getStartJob())
		return true;
	else if(this->getStartJob() > cons.getStartJob())
		return false;

	//Compare by end job
	if(this->getEndJob() < cons.getEndJob())
		return true;
	else if(this->getEndJob() > cons.getEndJob())
		return false;

	//Compare by time
	if(this->getTime() < cons.getTime())
		return true;
	else if(this->getTime() > cons.getTime())
		return false;

	//Compare by equipment type
	if(this->getEquipmentType() < cons.getEquipmentType())
		return true;
	else if(this->getEquipmentType() > cons.getEquipmentType())
		return false;

	return false;
}

bool Constraint::operator== (const Constraint& cons) const
{
	return (!(*this < cons) && !(cons < *this));
}


std::string Constraint::toString()
{
	stringstream str;

	//type
	switch(type){
		case C_COVER:
			str << "COVER_" << sJob ;
			break;
		case C_SYNCH:
			str << "SYNCH_" << sJob << "," << time << "," << eqType;
			break;
		case C_CARD:
			str << "CARD_" << eqType;
			break;
		case C_EXPLICIT:
			str << "EXPLICIT_" << sJob << "," << eJob << "," << time << "," << eqType;
			break;
		default:			
			str << "N/A";
	}

	return str.str();
}

size_t ConstraintHasher::operator() (const Constraint& cons) const
{
	unsigned int sum = 0;

	//add the type cntribution
	if (cons.type != NULL)
	{
		sum *= HASH_PRIME;
		sum += intHash(cons.type);
	}

	//add start job contribution
	if(cons.sJob != -1){
		sum *= HASH_PRIME;
		sum += intHash(cons.sJob);
	}

	//add end job contribution
	if(cons.eJob != -1){
		sum *= HASH_PRIME;
		sum += intHash(cons.eJob);
	}

	//add time contribution
	if(cons.time != -1){
		sum *= HASH_PRIME;
		sum += intHash(cons.time);
	}

	//add equipment type contribution
	if(cons.eqType != -1){
		sum *= HASH_PRIME;
		sum += intHash(cons.eqType);
	}	

	return sum;
}

bool ConstraintHasher::operator() (const Constraint& cons1, const Constraint& cons2) const
{
	return (cons1 < cons2);
}

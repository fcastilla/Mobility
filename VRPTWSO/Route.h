#pragma once

#include <string>
#include <vector>
#include "Bucket.h"

#include <sstream>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

using namespace std;

class Route
{
public:
	Route(int e) : eqType(e){}
	~Route(){ edges.clear(); }

	vector<Edge*> edges;

	//GET METHODS
	int getRouteNumber() const{ return routeNumber; }
	int getEquipmentType() const{ return eqType; }
	double getCost() const{ return cost; }

	//SET METHODS
	void setRouteNumber(int num){ routeNumber = num; }
	void setCost(double c){ cost = c; }

	//OPERATORS
	bool operator<(const Route& other){
		if(this->getCost() < other.getCost())
			return true;
		else if(other.getCost() < this->getCost())
			return false;

		if(this->getRouteNumber() < other.getRouteNumber())
			return true;

		return false;
	}

	//OTHER
	string toString(){ 
		stringstream s;
		s << "Route " << routeNumber << " | Cost: " << cost << " | Route: ";
		Edge *myEdge;
		vector<Edge*>::reverse_iterator it = edges.rbegin();
		for(; it != edges.rend(); it++){
			myEdge = (*it);			
			s << myEdge->getStartJob() << ",";
		}
		s << myEdge->getEndJob();
		return s.str();
	}

private:
	int routeNumber;
	int eqType;
	double cost;
};
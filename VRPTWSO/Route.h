#pragma once

#include <string>
#include <vector>
#include "Bucket.h"

#include <sstream>

using namespace std;

class Route
{
public:
	Route(int e) : eqType(e){}
	~Route(){ edges.clear(); }

	vector<Edge*> edges;

	//GET METHODS
	int getRouteNumber(){ return routeNumber; }
	int getEquipmentType(){ return eqType; }
	double getCost(){ return cost; }

	//SET METHODS
	void setRouteNumber(int num){ routeNumber = num; }
	void setCost(double c){ cost = c; }

	//OTHER
	string toString(){ 
		stringstream s;
		s << "Route " << routeNumber << " - Cost: " << cost << " - Route: ";
		vector<Edge*>::reverse_iterator it = edges.rbegin();
		for(; it != edges.rend(); it++){
			Edge *myEdge = (*it);			
			s << "(" << myEdge->getStartJob() << "," << myEdge->getEndJob() << "," << myEdge->getTime() << ") ->";
		}

		return s.str();
	}

private:
	int routeNumber;
	int eqType;
	double cost;
};
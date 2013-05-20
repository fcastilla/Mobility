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
	int getEquipmentType(){ return eqType; }
	double getCost(){ return cost; }

	//SET METHODS
	void setCost(double c){ cost = c; }

	//OTHER
	string toString(){ 
		stringstream s;
		s << "New Route - Cost: " << cost << " - Route: ";
		vector<Edge*>::reverse_iterator it = edges.rbegin();
		for(; it != edges.rend(); it++){
			Edge *myEdge = (*it);			
			s << "(" << myEdge->getStartJob() << "," << myEdge->getEndJob() << "," << myEdge->getTime() << ") ->";
		}

		return s.str();
	}

private:
	int eqType;
	double cost;
};
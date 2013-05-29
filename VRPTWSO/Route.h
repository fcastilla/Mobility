#pragma once

#include <string>
#include <vector>
#include "Bucket.h"

#include <sstream>
#include <iomanip>

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
	Route(Route *r){
		routeNumber = r->getRouteNumber();
		eqType = r->getEquipmentType();
		cost = r->getCost();
		reducedCost = r->getReducedCost();
		solVal = r->getSolVal();
		edges = r->edges;
	}
	Route() : routeNumber(-1), eqType(-1), cost(0), reducedCost(0), solVal(0) { edges = vector<Edge*>(); }
	Route(int e) : routeNumber(-1), eqType(e), cost(0), reducedCost(0), solVal(0){ edges = vector<Edge*>(); }

	~Route(){
		vector<Edge*>::iterator it = edges.begin();
		for(; it != edges.end(); it++)
			delete (*it);
		edges.clear(); 
	}

	vector<Edge*> edges;

	//GET METHODS
	int getRouteNumber() const{ return routeNumber; }
	int getEquipmentType() const{ return eqType; }
	double getCost() const{ return cost; }
	double getReducedCost() const { return reducedCost; }
	double getSolVal() const{ return solVal; }

	//SET METHODS
	void setEquipmentType(int e){ eqType = e; }
	void setRouteNumber(int num){ routeNumber = num; }
	void setCost(double c){ cost = c; }
	void setReducedCost(double c){ reducedCost = c; }
	void setSolVal(double val){ solVal = val; }

	//OPERATORS
	bool operator<(const Route *other){
		if(this->getCost() < other->getCost())
			return true;
		else if(other->getCost() < this->getCost())
			return false;

		if(this->getRouteNumber() < other->getRouteNumber())
			return true;

		return false;
	}
	
	//OTHER
	bool findArc(int j, int i, int t){
		Edge *myEdge;
		vector<Edge*>::reverse_iterator it = edges.rbegin();
		for(; it != edges.rend(); it++){
			myEdge = (*it);	
			if(myEdge->getStartJob() == j && myEdge->getEndJob() == i && myEdge->getTime() == t)
				return true;
		}
		return false;
	}

	string toString(){ 
		stringstream s;
		s << "Route:" << setw(8) << routeNumber << " | RCost:" << setw(8) << reducedCost << " | EqType:" << eqType << " = ";
		Edge *myEdge;
		vector<Edge*>::reverse_iterator it = edges.rbegin();
		for(; it != edges.rend(); it++){
			myEdge = (*it);			
			s << "(" << myEdge->getStartJob() << "," << myEdge->getEndJob() << "," << myEdge->getTime() << ")->";
		}
		return s.str();
	}

private:
	double solVal;
	int routeNumber;
	int eqType;
	double cost;
	double reducedCost;
};
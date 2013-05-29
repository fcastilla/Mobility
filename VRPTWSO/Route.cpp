#include "Route.h"

Route::Route(Route *r)
{
	routeNumber = r->getRouteNumber();
	eqType = r->getEquipmentType();
	cost = r->getCost();
	reducedCost = r->getReducedCost();
	solVal = r->getSolVal();
	edges = r->edges;
}

Route::~Route()
{
	//DONT DELETE EDGES, NOW THEY EXIST ONLY ONCE, AND ARE REFERENCED BY MULTIPLE ROUTES.
	/*vector<Edge*>::iterator it = edges.begin();
	for(; it != edges.end(); it++)
		delete (*it);*/
	edges.clear(); 
}

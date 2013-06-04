#include "Solution.h"
#include "Route.h"

Solution::Solution()
{
	parameters = GlobalParameters::getInstance();
	routes = vector<Route*>(); 
}

Solution::~Solution()
{
	routes.clear();
}

string Solution::toString()
{
	stringstream output;
	output << "Solution value: " << solutionVal << endl;
	output << "Selected Routes: ";
	vector<Route*>::iterator it = routes.begin();
	while(it != routes.end()){
		output << endl << (*it)->toString();
		it ++;
	}

	return output.str();
}

std::ostream& operator << (std::ostream& out, Solution &sol)
{
	//Solution value
	out << "#Solution value: " << endl; 
	out << sol.getSolutionValue() << endl;

	
	Route *route;
	vector<Route*>::iterator it = sol.getRoutes().begin();
	vector<Route*>::iterator eit = sol.getRoutes().end();

	//Visited Customers
	out << "#Pure Spatial Routes:";

	while(it != eit){	
		out << endl;
		route = (*it);

		vector<Edge*>::reverse_iterator itEdge = route->edges.rbegin();
		for(; itEdge != route->edges.rend(); itEdge++){
			out << (*itEdge)->getStartJob() << " ";
		}
		out << "0";
		it++;
	}

	//Complete Solution
	out << endl << "#Complete Routes:";
	it = sol.getRoutes().begin();
	eit = sol.getRoutes().end();
	while(it != eit){
		out << endl << (*it)->toString();
		it ++;
	}

	out << endl;

	return out;
}
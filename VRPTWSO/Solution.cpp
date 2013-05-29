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
	return out;
}
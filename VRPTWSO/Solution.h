#pragma once

#include "Bucket.h"
#include <string>
#include <sstream>
#include <vector>

using namespace std;

class Route;

class Solution
{
public:
	Solution();
	~Solution();

	//GET METHODS
	string getInstanceName() const { return instanceName; }
	double getSolutionValue() const { return solutionVal; }
	vector<Route*> &getRoutes() { return routes; }

	//SET METHODS
	void setInstanceName(string name) { instanceName = name; }
	void setSolutionValue(double val){ solutionVal = val; }
	void addRoute(Route *r){ routes.push_back(r); }
	

	//OPERATORS
	bool operator<(const Solution *sol) const{
		return this->getSolutionValue() < sol->getSolutionValue();
	}

	//OTHER METHODS
	string toString();

private:
	GlobalParameters *parameters;
	string instanceName;
	double solutionVal;
	vector<Route*> routes;
};

std::ostream& operator << (std::ostream& out, Solution &sol);
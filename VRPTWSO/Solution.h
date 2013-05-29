#pragma once

#include "Bucket.h"
#include <string>
#include <sstream>
#include <vector>

using namespace std;

class Solution
{
public:
	Solution(){ routes = vector<Route*>(); }
	~Solution(){ routes.clear(); }

	//GET METHODS
	double getSolutionValue() const { return solutionVal; }

	//SET METHODS
	void setSolutionValue(double val){ solutionVal = val; }
	void addRoute(Route *r){ routes.push_back(r); }
	

	//OPERATORS
	bool operator<(const Solution *sol) const{
		return this->getSolutionValue() < sol->getSolutionValue();
	}

	string toString(){
		stringstream output;
		output << "---------------------------------------------" << endl;
		output << "Solution value: " << solutionVal << endl;
		output << "Selected Routes: " << endl;
		vector<Route*>::iterator it = routes.begin();
		while(it != routes.end()){
			output << (*it)->toString() << endl;
			it ++;
		}
		output << "---------------------------------------------" << endl;

		return output.str();
	}

private:
	GlobalParameters *parameters;
	double solutionVal;
	vector<Route*> routes;
};
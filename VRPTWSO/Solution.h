#pragma once

#include "Bucket.h"
#include <string>
#include <sstream>
#include <vector>

using namespace std;

class Solution
{
public:
	Solution();
	~Solution();

	//GET METHODS
	double getSolutionValue() const { return solutionVal; }

	//SET METHODS
	void setSolutionValue(double val){ solutionVal = val; }
	void addEdge(int sJob, int dJob, int sTime, int dTime, int eqType);

	//OPERATORS
	bool operator<(const Solution *sol) const{
		return this->getSolutionValue() < sol->getSolutionValue();
	}

	string toString();

private:
	GlobalParameters *parameters;
	double solutionVal;
	vector<vector<Vertex*>> network;
};
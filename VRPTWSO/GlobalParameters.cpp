#include "GlobalParameters.h"
#include <stdio.h>

GlobalParameters* GlobalParameters::instance = NULL;

GlobalParameters::GlobalParameters()
{
	bigM = 10000;
	epsilon = 1e-5;
	maxRoutesPerIteration = 10;
	dualStabilization = false;
	printLevel = 0;
	solveMIP = false;
	numSolutions = 100;
	timeLimit = 15000;
}

GlobalParameters* GlobalParameters::getInstance()
{
	if(!instance){
		instance = new GlobalParameters();
	}
	return instance;
}
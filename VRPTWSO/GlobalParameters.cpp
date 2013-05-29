#include "GlobalParameters.h"
#include <stdio.h>

GlobalParameters* GlobalParameters::instance = NULL;

GlobalParameters::GlobalParameters()
{
	bigM = 10000;
	epsilon = 1e-5;
	maxRoutesPerIteration = 1;
	dualStabilization = false;
}

GlobalParameters* GlobalParameters::getInstance()
{
	if(!instance){
		instance = new GlobalParameters();
	}
	return instance;
}
#pragma once

class GlobalParameters
{
public:
	static GlobalParameters *getInstance();

	//GET METHODS
	double getBigM(){ return bigM; }
	double getEpsilon(){ return epsilon; }
	int getMaxRoutes(){ return maxRoutesPerIteration; }
	bool useDualStabilization(){ return dualStabilization; }

	int getNumJobs(){ return numJobs; }
	int getNumEquipments(){ return numEquipments; }
	int getHorizonLength(){ return horizon; }
	int getPrintLevel(){ return printLevel; }
	bool solveByMIP(){ return solveMIP; }
	int getNumSolutions(){ return numSolutions; }
	double getTimeLimit(){ return timeLimit; }

	//SET METHODS
	void setBigM(double m){ bigM = m; }
	void setEpsilon(double e){ epsilon = e; }
	void setMaxRoutes(int max){ maxRoutesPerIteration = max; }
	void setDualStabilization(bool opt){ dualStabilization = opt; }
	void setSolveMIP(bool opt){ solveMIP = opt; }
	void setNumSolutions(int num){ numSolutions = num; }
	void setTimeLimit(double t){ timeLimit = t; }

	void setNumJobs(int n){ numJobs = n; }
	void setNumEquipments(int e){ numEquipments = e; }
	void setHorizonLength(int h){ horizon = h; }

private:
	GlobalParameters();
	static GlobalParameters *instance;

	bool solveMIP;
	int printLevel;
	int numSolutions;
	double timeLimit;

	int numJobs;
	int numEquipments;
	int horizon;

	double bigM;
	double epsilon;
	int maxRoutesPerIteration;
	bool dualStabilization;
};
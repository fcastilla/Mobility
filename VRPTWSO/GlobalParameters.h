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

	//SET METHODS
	void setBigM(double m){ bigM = m; }
	void setEpsilon(double e){ epsilon = e; }
	void setMaxRoutes(int max){ maxRoutesPerIteration = max; }
	void setDualStabilization(bool opt){ dualStabilization = opt; }

	void setNumJobs(int n){ numJobs = n; }
	void setNumEquipments(int e){ numEquipments = e; }
	void setHorizonLength(int h){ horizon = h; }

private:
	GlobalParameters();
	static GlobalParameters *instance;

	int numJobs;
	int numEquipments;
	int horizon;

	double bigM;
	double epsilon;
	int maxRoutesPerIteration;
	bool dualStabilization;
};
#include "Data.h"
#include "Bucket.h"
#include "Route.h"

#include <vector>

using namespace std;

enum SubproblemType
{
	QROUTE,
	QROUTE_NOLOOP,
	NGROUTE
};

class LPSolver;

class SubproblemSolver
{
	
public:
	SubproblemSolver(LPSolver *mPtr, ProblemData * d, int e, SubproblemType m);
	~SubproblemSolver();

	vector<Route*> routes;

	void reset();
	void collapseVertices();
	void updateReducedCostsMatrix();
	bool isInfeasible(){ return infeasible; }
	void solve();

private:
	LPSolver *master;
	ProblemData * data;
	int eqType;
	SubproblemType method;

	int totalJobs;
	double infinityValue;
	bool infeasible;

	vector<vector<vector<double>>> reducedCosts;
	vector<vector<Bucket*>> fMatrix;
	vector<int> fixatedVars;

};


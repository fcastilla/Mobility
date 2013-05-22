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

class Node;
class Solver;

class SubproblemSolver
{
	
public:
	SubproblemSolver(ProblemData * d, SubproblemType m);
	~SubproblemSolver();

	vector<Route*> routes;

	void reset();
	void solve(Node *node, int eqType);
	void collapseVertices(Node *node, int eqType);
	void updateReducedCostsMatrix(Node *node, int eqType);
	bool isInfeasible(){ return infeasible; }	

private:
	ProblemData * data;
	SubproblemType method;

	double infinityValue;
	bool infeasible;

	vector<vector<vector<double>>> reducedCosts;
	vector<vector<Bucket*>> fMatrix;
	vector<int> fixatedVars;

};


#include "Data.h"
#include "Bucket.h"
#include "Route.h"

#include <vector>
#include <set>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

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
	void solve(Node *node, int eqType, int maxRoutes);
	void collapseVertices(Node *node, int eqType);
	bool isInfeasible(){ return infeasible; }	

private:
	ProblemData * data;
	SubproblemType method;
	GlobalParameters *parameters;

	double infinityValue;
	bool infeasible;

	vector<vector<bool>> visited;
	vector<vector<vector<double>>> reducedCosts;
	vector<vector<Bucket*>> fMatrix;
	vector<int> fixatedVars;

};


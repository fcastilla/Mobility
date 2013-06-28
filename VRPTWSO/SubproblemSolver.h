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
	SubproblemSolver(ProblemData * d);
	~SubproblemSolver();

	vector<Route*> routes;

	void reset();
	void solve(Node *node, int eqType);

private:
	ProblemData * data;
	SubproblemType method;
	GlobalParameters *parameters;

	double infinityValue;

	vector<vector<bool>> visited;
	vector<vector<Bucket*>> fMatrix;

};


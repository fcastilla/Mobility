#include "gurobi_c++.h"
#include "Data.h"
#include "Solver.h"
#include "SubproblemSolver.h"
#include "gurobi_c++.h"

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

using namespace std;

int main(int argc, char *argv[])
{
	string fileName, sp, mip;
	
	//_CrtSetBreakAlloc(173594);

	//Validate parameters
	if(argc <= 2){
		cout << "VRPTWSO <instance_name> <subproblem_type> <solve_mip>" << endl;
		goto error;
	}
	
	fileName = string(argv[1]);
	sp =  string(argv[2]);
	//mip = string(argv[3]);

	SubproblemType type = (atoi(sp.c_str()) == 0)? QROUTE : QROUTE_NOLOOP;

	//Create object for data reading
	ProblemData *data = new ProblemData();
	data->readData(fileName);

	//Create solver
	Solver *mySolver = new Solver(data,type);
	int result = mySolver->solve();
	cout << "Status final: " << result << endl;
	
	delete mySolver;

	//getchar();
	//_CrtDumpMemoryLeaks();

	return EXIT_SUCCESS;

error:
	return EXIT_FAILURE;
}
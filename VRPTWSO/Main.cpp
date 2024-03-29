#include "gurobi_c++.h"
#include "Data.h"
#include "Solver.h"
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
	string fileName;
	//_CrtSetBreakAlloc(173594);

	//Validate parameters
	if(argc < 1){
		cout << "Please provide a instance file name. " << endl;
		goto error;
	}
	
	fileName = string(argv[1]);

	//Create object for data reading
	ProblemData *data = new ProblemData();
	data->readData(fileName);

	//Create solver
	Solver *mySolver = new Solver(data);
	int result = mySolver->solve();
	cout << "Status final: " << result << endl;
	
	delete mySolver;

	//_CrtDumpMemoryLeaks();

	getchar();
	return EXIT_SUCCESS;

error:
	return EXIT_FAILURE;
}
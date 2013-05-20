#include "gurobi_c++.h"
#include "Data.h"
#include "LPSolver.h"

using namespace std;

int main(int argc, char *argv[])
{
	string fileName;

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
	LPSolver *mySolver = new LPSolver(data);
	int result = mySolver->solve();

	cout << "Status final: " << result << endl;

	return EXIT_SUCCESS;

error:
	return EXIT_FAILURE;
}
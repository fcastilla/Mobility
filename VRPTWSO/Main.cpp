#include "gurobi_c++.h"
#include "Data.h"
#include "Solver.h"
#include "gurobi_c++.h"

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
	Solver *mySolver = new Solver(data);
	int result = mySolver->solve();
	cout << "Status final: " << result << endl;

	getchar();
	return EXIT_SUCCESS;

error:
	return EXIT_FAILURE;
}
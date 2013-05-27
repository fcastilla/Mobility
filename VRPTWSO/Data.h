#pragma once

#include "Job.h"
#include "Equipment.h"
#include "Location.h"
#include "Bucket.h"
#include "GlobalParameters.h"

#include <string>
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

struct ProblemData
{
	ProblemData(): numJobs(0), numEquipments(0), numLocations(0), numPrecedences(0), horizonLength(0), numWorkShifts(0), workShiftLength(0) {}

	int numJobs;
	int numEquipments;
	int numLocations;
	int numPrecedences;
	int horizonLength;
	int numWorkShifts;
	int workShiftLength;

	vector<Job*> jobs;
	vector<Equipment*> equipments;

	GlobalParameters *parameters;
	int getNumJobs() const{ return numJobs; }
	int getHorizonLength() const { return horizonLength; }

	//successors of job i on machine e
	vector<vector<vector<int>>> successors;

	//problem network
	vector<vector<Vertex*>> problemNetwork;
	set<Vertex*,VertexComparator> vertexSet;

	void readData(const std::string & inputFileName);
};


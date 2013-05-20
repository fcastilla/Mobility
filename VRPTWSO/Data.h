#pragma once

#include "Job.h"
#include "Equipment.h"
#include "Location.h"
#include "Bucket.h"

#include <string>
#include <vector>

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

	//successors of job i on machine e
	vector<vector<vector<int>>> successors;

	//problem network
	vector<vector<Vertex*>> problemNetwork;

	void readData(const std::string & inputFileName);
};

#include "Data.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

void ProblemData::readData(const std::string & fileName)
{
	ifstream in;
	in.open(fileName.c_str());
	if (!in)
	{
		std::cout << "Instance does not exist." << endl;
		exit(EXIT_FAILURE);
	}

	string line, str;
	string space;
	int sw = 0;
	int con = 0;
	int precCounter = 0;

	std::vector<Location*> locations;

	while (getline(in, line))
	{
		if (line.find('#') != string::npos) //Header line
		{ 
			sw++;
			continue;
		}

		stringstream is(line);
		switch (sw)
		{
		case 1:
			is >> numEquipments;
			equipments.resize(numEquipments);
			sw++;
			break;
		case 2:
			//types of equipment
			int equipmentId, machineQty, maxTransitionTime, capacity;
			is >> equipmentId >> machineQty >> maxTransitionTime >> capacity;
			equipments[equipmentId] = new Equipment(equipmentId, machineQty, maxTransitionTime,	capacity);
			break;
		case 3:
			is >> numLocations;
			locations.resize(numLocations);
			sw++;
			break;
		case 4:
			//locations
			int locationId, x, y;
			is >> locationId >> x >> y;
			locations[locationId] = new Location(locationId, x, y);
			break;
		case 5:
			//total workshifts
			is >> horizonLength >> numWorkShifts;
			workShiftLength = horizonLength / numWorkShifts;
			if (workShiftLength * numWorkShifts != horizonLength)
				horizonLength = workShiftLength * numWorkShifts;
			break;
		case 6:
			//total Jobs
			is >> numJobs;
			jobs.resize(numJobs); 
			sw++;
			break;
		case 7:
			int jobId, weight, serviceTime, demand, numWS, locId;
			std::vector<bool> workShifts(numWorkShifts, false);
			std::vector<int> readyDates(numWorkShifts, 0);
			std::vector<int> dueDates(numWorkShifts, 0);
			std::vector<int> machineTypes(numEquipments);

			is >> jobId >> weight >> serviceTime >> locId >> demand;
			jobs[jobId] = new Job(jobId, weight, serviceTime, locId, demand);

			for (int j = 0; j < numEquipments; j++)
				is >> machineTypes[j];
			jobs[jobId]->setEquipmentTypes(machineTypes);

			is >> numWS;
			for (int sh = 0; sh < numWS; sh++)
			{
				int id, rel, due;
				is >> id >> rel >> due;
				workShifts[id] = true;
				readyDates[id] = rel;
				dueDates[id] = due;
			}
			jobs[jobId]->setWorkShifts(workShifts);
			jobs[jobId]->setReadyDates(readyDates);
			jobs[jobId]->setDueDates(dueDates);
			break;
		}
	}

	/// Calculate distances between locations
	std::vector<std::vector<double>> locDistances(numLocations,std::vector<double>(numLocations, 0.0));
	Location *origLocPtr, *destLocPtr;
	double maxLocDistance = 0;
	for (int origId = 0; origId < numLocations - 1; origId++)
	{
		for (int destId = origId + 1; destId < numLocations; destId++)
		{
			origLocPtr = locations[origId];
			destLocPtr = locations[destId];
			double distance = sqrt(
				pow(origLocPtr->getX() - destLocPtr->getX(), 2.0)
				+ pow(origLocPtr->getY() - destLocPtr->getY(), 2.0));
			locDistances[origId][destId] = locDistances[destId][origId] = distance;
			if (distance > maxLocDistance)
				maxLocDistance = distance;
		}
	}

	//Recalculate distances proportionally to max transition time 
	//and initialize transition times for each equipment type
	int origJobLocId, destJobLocId;
	double transitionTime;
	for (std::vector<Equipment *>::iterator vepIt = equipments.begin(); vepIt != equipments.end();
		vepIt++)
	{
		/// dummy job is also used
		std::vector<std::vector<double>> jobTransitionTimes(numJobs + 1, std::vector<double>(numJobs + 1, 0));
		for (int origJob = 0; origJob < numJobs; origJob++)
		{
			/// to real jobs
			for (int destJob = origJob + 1; destJob < numJobs; destJob++)
			{
				origJobLocId = jobs[origJob]->getLocationId();
				destJobLocId = jobs[destJob]->getLocationId();
				if ((*vepIt)->getMaxTransitionTime() <= 0){ //dont recalculate
					transitionTime = (double) locDistances[origJobLocId][destJobLocId];
				}else{
					transitionTime = (double) (locDistances[origJobLocId][destJobLocId] / maxLocDistance
						* (*vepIt)->getMaxTransitionTime());
				}
				jobTransitionTimes[destJob][origJob] = jobTransitionTimes[origJob][destJob] = transitionTime;
			}
		}
		(*vepIt)->setTransitionTimes(jobTransitionTimes);
	}

	//Set time windows for all jobs
	for(int j=0; j<numJobs; j++){		
		jobs[j]->setTimePeriods(horizonLength, equipments);
	}

	//Initialize problem network
	problemNetwork = vector<vector<Vertex*>>(numJobs, vector<Vertex*>(horizonLength + 1, nullptr));

	
}

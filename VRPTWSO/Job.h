#pragma once

#include "Equipment.h"

#include <vector>
#include <set>

using namespace std;

class Job
{
public:	
	Job(int _id, int _w, int _sTime, int _loc, int _demand) : 
		id(_id), weight(_w), serviceTime(_sTime), locationId(_loc), 
		demand(_demand), firstStartTimePeriod(100000), lastStartTimePeriod(0) { }

	//GET METHODS
	int getJobId(){ return id; }
	int getWeight(){ return weight; }
	int getServiceTime(){ return serviceTime; }
	int getLocationId(){ return locationId; }
	int getDemand(){ return demand; }
	int getFirstEquipmentType();
	int getFirstStartTimePeriod(){ return firstStartTimePeriod; }
	int getLastStartTimePeriod() { return lastStartTimePeriod; }

	bool getWorkShift(int s) { return workShifts[s]; }
	int getReleaseDate(int s) { return readyDates[s]; }
	int getDueDate(int s) { return dueDates[s]; }
	int getEquipmentTypeRequired(int e) { return equipmentTypes[e]; }	
	bool getTimePeriod(int t) { return (t <= 0 ? false : timePeriods[t]); }
	set<int>& getTimePeriodsSet() { return timePeriodsSet; }

	//SET METHODS
	void setWorkShifts(vector<bool>& _workshifts) { workShifts = _workshifts; }	
	void setReadyDates(vector<int>& _releaseDates) { readyDates = _releaseDates; }	
	void setDueDates(vector<int>& _dueDates) { dueDates = _dueDates; }
	void setEquipmentTypes(vector<int>& _equipmentTypes){ equipmentTypes = _equipmentTypes; }
	void setTimePeriods(int horizonLength, const vector<Equipment*>& equipments);

private:
	vector<bool> workShifts;
	vector<int> equipmentTypes;
	vector<bool> timePeriods;	
	set<int> timePeriodsSet;
	vector<int> readyDates;
	vector<int> dueDates;
	int id, weight, serviceTime, locationId, demand;
	int firstStartTimePeriod;
	int lastStartTimePeriod;
};

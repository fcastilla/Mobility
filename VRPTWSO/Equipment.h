#pragma once

#include <vector>
using namespace std;

class Equipment
{

public:
	Equipment(int _id, int _numM, int _maxTime = 0, int _cap = 0) : 
		id(_id), numMachines(_numM), maxTransitionTime(_maxTime), capacity(_cap) {}

	//GET METHODS
	int getEquipmentId() { return id; }
	int getNumMachines() { return numMachines; }	
	double getTransitionTime(int i, int j) { return transitionTime[i][j]; }
	double getMaxTransitionTime() { return maxTransitionTime;}
	
	//SET METHODS
	void setTransitionTimes(vector<vector<double>> &_transitionTime){	transitionTime = _transitionTime; }
	void setMaxTransitionTime(int _maxTransitionTime){ maxTransitionTime = _maxTransitionTime; }

private:
	vector<vector<double>> transitionTime;
	int id, numMachines;
	int capacity;
	double maxTransitionTime;
};
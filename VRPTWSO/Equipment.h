#pragma once

#include <vector>
#include <math.h>
using namespace std;

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

class Equipment
{

public:
	Equipment(int _id, int _numM, int _maxTime = 0, int _cap = 0) : 
		id(_id), numMachines(_numM), maxTransitionTime(_maxTime), capacity(_cap) {}

	//GET METHODS
	int getEquipmentId() { return id; }
	int getNumMachines() { return numMachines; }	
	double getTransitionTime(int i, int j) { return ceil(transitionTime[i][j]); }
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
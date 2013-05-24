#include "Job.h"
#include <math.h>

void Job::setTimePeriods(int horizonLength, const vector<Equipment*>& equipments)
{
	timePeriods = std::vector<bool>(horizonLength + 1, false);
	timePeriodsSet.clear();
	int timeIndex, shiftIndex;	

	//Let the time window of sJob be denoted by [a;b], transition time of sJob to depot by dTime and the horizon length by T
	//The time window of sJob can safely be shrinked to [max(a,dTime);min(b,T-dTime-serviceTime)]
	int timeToDepot = 10000000;
	for(int eqType = 0; eqType < equipmentTypes.size(); eqType++){
		if(equipmentTypes[eqType])
			timeToDepot = min(timeToDepot, (int)equipments[eqType]->getTransitionTime(id,0));
	}
	
	for (shiftIndex = 0; shiftIndex < workShifts.size(); shiftIndex++)
	{
		if (!workShifts[shiftIndex]) continue;

		for (timeIndex = readyDates[shiftIndex]; timeIndex <= dueDates[shiftIndex] /*- serviceTime*/; timeIndex++){
			//shrink time window
			if(timeIndex > (horizonLength - serviceTime - timeToDepot)) break;
			if(timeIndex < timeToDepot) continue;
			//------------------

			timePeriods[timeIndex] = true;
			timePeriodsSet.insert(timeIndex);
			firstStartTimePeriod = min(firstStartTimePeriod,timeIndex);
			lastStartTimePeriod = timeIndex;
		}
	}
}

int Job::getFirstEquipmentType()
{
	int eq;

	for(eq=0; eq<equipmentTypes.size(); eq++)
	{
		if(equipmentTypes[eq] != 0) 
			break;
	}

	return eq;
}
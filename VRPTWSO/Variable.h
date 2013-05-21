#pragma once

#include "gurobi_c++.h"
#include <map>
#include <hash_map>

// All variable types
enum VariableType
{
	V_ERROR = 0,
	V_Y,
	V_X,
	V_LAMBDA,
	V_FAUX,
	V_BAUX
};

//Variables
class Variable 
{
	//Struct used by hash table
	friend class VariableHasher;

public:
	//Constructors
	Variable();
	Variable(const Variable& orig);

	//Destructor
	virtual ~Variable();

	// GET METHODS 
	VariableType getType() const { return type; }
	double getValue() const { return *value; }
	int getStartJob() const { return sJob; }
	int getEndJob() const { return eJob; }
	int getTime() const { return time; }
	int getEquipmentType() const { return eqType; }
	int getRouteNum() const { return routeNum; }

	// SET METHODS 
	void reset();
	void setType(VariableType t){ type = t; }
	void setValue(double v) { *value = v; }
	void setStartJob(int s) { sJob = s; }
	void setEndJob(int e) { eJob = e; }
	void setTime(int t) { time = t; }
	void setEquipmentTipe(int e){ eqType = e; }
	void setRouteNum(int num){ routeNum = num; }

	// OPERATORS 
	//Assignment 
	Variable& operator=(const Variable& var);
	//Less 
	bool operator<(const Variable& var) const;
	//Equals 
	bool operator==(const Variable& var) const;

	//Variable name
	std::string toString();

	//Other members
	int rank;
	double fractionality;
	double score;

private:
	VariableType type;
	double *value;
	int sJob, eJob;
	int time;
	int eqType;
	int routeNum;
};


class VariableHasher : public stdext::hash_compare<Variable>
{
public:
	//Less operator
	bool operator()(const Variable& v1, const Variable& v2) const;

	//Hash value
	size_t operator()(const Variable& v) const;
};

class VariableRankComparator
{
public:
    bool operator()(const Variable& v1, const Variable& v2){
        return v1.rank < v2.rank;
    }
}varRankComparator;

class VariableFractionalityComparator
{
public:
    bool operator()(const Variable& v1, const Variable& v2){
        return v1.fractionality < v2.fractionality;
    }
}varFractionalityComparator;

class VariableScoreComparator
{
public:
    bool operator()(const Variable& v1, const Variable& v2){
        return v1.score < v2.score;
    }
}varScoreComparator;

/**
* Type definition for the hash object.
*/
typedef stdext::hash_map<Variable, bool, VariableHasher> VariableHash;


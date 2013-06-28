#pragma once

#include "GlobalParameters.h"
#include <vector>
#include <bitset>
#include <set>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

using namespace std;

class Label;
class Bucket;
class qRouteBucket;
class qRouteNoLoopBucket;
class Vertex;

class Edge
{
public:
	Edge(int i, int j, int t, int a, double c) : sJob(i), eJob(j), time(t), aTime(a), cost(c){ }

	int getStartJob() const { return sJob; }
	int getEndJob() const { return eJob; }
	int getTime() const { return time; }
	double getCost() const{ return cost; }
	int getArriveTime() const{ return aTime; }

private:
	int sJob;
	int eJob;
	int time;
	int aTime;
	double cost;
};

class Vertex
{
public:
	Vertex(int numEq);
	~Vertex();

	//GET METHODS
	int getJob() const{ return job; }
	int getTime() const{ return time; }
	vector<Vertex*> &getAdjacenceList(int eq){ return adjacenceList[eq]; }
	vector<Vertex*> &getIncidenceList(int eq){ return incidenceList[eq]; }

	//SET METHODS
	void setJob(int j){ job = j; }
	void setTime(int t){ time = t; }
	void addInicidentVertex(int eqType, Vertex *v);
	void addAdjacentVertex(int eqType, Vertex *v);


private:
	int job, time;
	vector<vector<Vertex*>> adjacenceList;
	vector<vector<Vertex*>> incidenceList; //one incidence list per equipment type
};

class VertexComparator
{
public:
	bool operator()(const Vertex *v1, const Vertex *v2){
		//Compare by time
		if(v1->getTime() < v2->getTime())
			return true;
		else if(v2->getTime() < v1->getTime())
			return false;

		//Compare by job
		if(v1->getJob() < v2->getJob())
			return true;
		else if(v2->getTime() < v1->getTime())
			return false;

		return false;
	}
};

class Label
{
public:
	Label() : job(0), time(0), cost(0.0), predecessor(nullptr),fixed(false) { }
	Label(int j, int t) : job(j), time(t), cost(1e13), predecessor(nullptr), fixed(false) { }
	Label(int j, int t, double c) : job(j), time(t), cost(c), predecessor(nullptr), fixed(false) {  }
	Label(int j, int t, double c, Label *p) : job(j), time(t), cost(c), predecessor(p),fixed(false) {  }
	~Label(){ predecessor = nullptr;}
	

	//GET METHODS
	int getJob() const { return job; }
	int getTime() const { return time; }
	double getCost() const { return cost; }
	Label *getPredecessor(){ return predecessor; }
	bool isFixed() const{ return fixed; }

	//SET METHODS
	void setJob(int j){ job = j; }
	void setTime(int t){ time = t; }
	void setCost(double c){ cost = c; }
	void setPredecessor(Label * p){ predecessor = p; }
	void setFixed(bool f){ fixed = f; }

	std::bitset<51> unreachable;

	bool operator<(const Label& other) const;

private:
	int job;
	int time;
	double cost;
	bool fixed;
	Label *predecessor;
};

class LabelComparator
{
public:
    bool operator()(const Label *l1, const Label *l2){
        if(l1->getCost() < l2->getCost())
			return true;
		else if(l2->getCost() < l1->getCost())
			return false;

		if(l1->getJob() < l2->getJob())
			return true;
		else if(l2->getJob() < l1->getJob())
			return false;

		if(l1->getTime() < l2->getTime())
			return true;
		else if(l2->getTime() < l1->getTime())
			return false;

		return false;
    }
};

class Bucket
{
public:
	Bucket(){};
	~Bucket(){ reset(); }

	void addLabel(Label *l){ labels.insert(l); }
	virtual void evaluate(set<Label*,LabelComparator> oLabels, double rCost, bool fix) = 0;
	virtual Label *getBestLabel() = 0;


	//GET METHODS
	Bucket *getSuccessor(){ return successor; }
	int getJob(){ return job; }
	int getTime(){ return time; }
	set<Label *,LabelComparator> &getLabels(){ return labels; }

	//SET METHODS
	void setJob(int j){ job = j; }
	void setTime(int t){ time = t; }
	void setSuccessor(Bucket *b){ successor = b; }

	void reset();
	bool isEmpty(){ return (labels.size() == 0)? true : false; }

protected:
	int job;
	int time;
	int maxSize;
	set<Label*,LabelComparator> labels;
	Bucket *successor;
	GlobalParameters *parameters;
};

class QRouteNoLoopBucket : public Bucket
{
public:
	QRouteNoLoopBucket(int size) {
		maxSize = size;
		successor = nullptr;
		parameters = GlobalParameters::getInstance(); 
		labels = set<Label*,LabelComparator>();
	}

	//Interface methods
	void evaluate(set<Label*,LabelComparator> oLabels, double rCost, bool fix);
	Label *getBestLabel();
};

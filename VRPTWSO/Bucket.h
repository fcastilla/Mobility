#pragma once

#include <vector>

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
	Edge(int i, int j, int t) : sJob(i), eJob(j), time(t){ }

	int getStartJob(){ return sJob; }
	int getEndJob(){ return eJob; }
	int getTime(){ return time; }

private:
	int sJob;
	int eJob;
	int time;
};

class Vertex
{
public:
	Vertex(int numEq);
	~Vertex();

	//GET METHODS
	int getJob(){ return job; }
	int getTime(){ return time; }
	const vector<Vertex*> &getAdjacenceList(int eq){ return adjacenceList[eq]; }
	const vector<Vertex*> &getIncidenceList(int eq){ return incidenceList[eq]; }

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

class Label
{
public:
	Label() : job(0), time(0), cost(0.0), predecessor(nullptr), fixed(false) {}
	Label(int j, int t) : job(j), time(t), cost(1e13), predecessor(nullptr), fixed(false) {}
	Label(int j, int t, double c) : job(j), time(t), cost(c), predecessor(nullptr), fixed(false) {}
	Label(int j, int t, double c, Label *p) : job(j), time(t), cost(c), predecessor(p), fixed(false) {}
	~Label(){ predecessor = nullptr; }

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

	bool operator<(const Label& other) const;

private:
	int job;
	int time;
	double cost;
	bool fixed;
	Label *predecessor;
};

class Bucket
{
public:
	Bucket() : successor(nullptr) { labels = vector<Label*>(); };
	~Bucket(){ reset(); }

	void addLabel(Label *l){ labels.push_back(l); }
	virtual void evaluate(vector<Label*> oLabels, double rCost, bool fix) = 0;
	virtual Label *getBestLabel() = 0;

	//GET METHODS
	Bucket *getSuccessor(){ return successor; }
	int getJob(){ return job; }
	int getTime(){ return time; }
	vector<Label *> getLabels(){ return labels; }

	//SET METHODS
	void setJob(int j){ job = j; }
	void setTime(int t){ time = t; }
	void setSuccessor(Bucket *b){ successor = b; }

	void reset();
	bool isEmpty(){ return (labels.size() == 0)? true : false; }

protected:
	int job;
	int time;
	vector<Label*> labels;
	Bucket *successor;
};

class QRouteBucket : public Bucket
{
public:
	//Interface methods
	void evaluate(vector<Label*> oLabels, double rCost, bool fix);
	Label *getBestLabel();
};

class QRouteNoLoopBucket : public Bucket
{
public:
	//Interface methods
	void evaluate(vector<Label*> oLabels, double rCost, bool fix);
	Label *getBestLabel();
};

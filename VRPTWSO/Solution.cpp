#include "Solution.h"

Solution::Solution()
{
	parameters = GlobalParameters::getInstance();
	network = vector<vector<Vertex*>>(parameters->getNumJobs(), vector<Vertex*>(parameters->getHorizonLength()+1,nullptr));
}

Solution::~Solution()
{
	network.clear();
}

void Solution::addEdge(int sJob, int dJob, int sTime, int dTime, int eqType){
	Vertex *v, *o;
	if(network[sJob][sTime] == nullptr){
		v = new Vertex(parameters->getNumEquipments());
		v->setJob(sJob);
		v->setTime(sTime);
		network[sJob][sTime] = v;
	}

	v = network[sJob][sTime];

	if(network[dJob][dTime] == nullptr){
		o = new Vertex(parameters->getNumEquipments());
		o->setJob(dJob);
		o->setTime(dTime);
		network[dJob][dTime] = o;
	}

	o = network[dJob][dTime];

	v->addAdjacentVertex(eqType,o);
	o->addInicidentVertex(eqType,v);
}

string Solution::toString(){
	stringstream output;

	if(network[0][0] != nullptr){
		output << "---------------------------------------------" << endl;
		output << "Solution value: " << solutionVal << endl;
		output << "Selected Routes: " << endl;

		Vertex *v, *o, *depot;
		vector<Vertex*> myStack;
		depot = network[0][0];
		int contRoutes = 0;

		for(int eqType=0; eqType<parameters->getNumEquipments(); eqType++){
			vector<vector<bool>> visited = vector<vector<bool>>(parameters->getNumJobs(), vector<bool>(parameters->getHorizonLength() +1, false));
			myStack.push_back(depot);
			while(myStack.size() > 0){
				v = myStack.back();
				myStack.pop_back();
				if(v->getJob() == 0 && v->getTime() != 0){ //end of a route
					output << "(" << v->getJob() << ")" << endl;
					myStack.push_back(depot);
				}else if(v->getJob() == 0 && v->getTime() == 0){ //begin of a route (depot)
					//Try to add an adjacent vertex
					vector<Vertex*>::iterator it = v->getAdjacenceList(eqType).begin();
					while(it != v->getAdjacenceList(eqType).end()){
						o = (*it);
						if(!visited[o->getJob()][o->getTime()]){
							myStack.push_back(o);
							visited[o->getJob()][o->getTime()] = true;
							output << "Route " << contRoutes++ << ": (" << v->getJob() << "," << o->getJob() << "," << v->getTime() << ")"; 
							break;
						}
						it++;
					}
				}else{
					//This vertex should have an adjacent vertex, find it and add it.
					if(v->getAdjacenceList(eqType).size() > 0){
						vector<Vertex*>::iterator it = v->getAdjacenceList(eqType).begin();
						while(it != v->getAdjacenceList(eqType).end()){
							o = (*it);
							if(!visited[o->getJob()][o->getTime()]){
								if(o->getJob() != 0) //back to depot
									visited[o->getJob()][o->getTime()] = true;
								myStack.push_back(o);								
								output << "(" << v->getJob() << "," << o->getJob() << "," << v->getTime() << ")"; 
								break;
							}
							it++;
						}
					}else{
						//look for exit from the same job, later in time
						for(int t=v->getTime()+1; t<= parameters->getHorizonLength(); t++){
							if(network[v->getJob()][t] != nullptr){
								visited[v->getJob()][t] = true;
								myStack.push_back(network[v->getJob()][t]);
							}
						}
					}
				}
			}
			myStack.clear();
		}

		output << "---------------------------------------------" << endl;
	}else{
		output << "ATENTION: Empty solution?" << endl;
	}

	return output.str();
}
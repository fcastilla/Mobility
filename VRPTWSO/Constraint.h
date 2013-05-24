#pragma once

#include "gurobi_c++.h"
#include <map>
#include <hash_map>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif


/** All constraint types */
enum ConstraintType
{
   C_ERROR = 0,
   C_COVER,
   C_SYNCH,
   C_OVF_FLOW_INIT,
   C_OVF_FLOW,
   C_CARD,
   C_EXPLICIT,
   C_BRANCH
};

/**
* Class which defines a contraint in the LP.
*/
class Constraint
{
   //Struct used by hash table
   friend class ConstraintHasher;

public:
   Constraint();
   Constraint(const Constraint& cons);
   ~Constraint();

   // GET METHODS 
   ConstraintType getType() const { return type; }
   int getStartJob() const { return sJob; }
   int getEndJob() const { return eJob; }
   int getTime() const { return time; }
   int getEquipmentType() const { return eqType; }
   
   int getRank() const { return *rank; }

   // SET METHODS 
   void reset();
   void setType(ConstraintType t){ type = t; } 
   void setStartJob(int s){ sJob = s; }
   void setEndJob(int e){ eJob = e; }
   void setTime(int t){ time = t; }
   void setEquipmentType(int e){ eqType = e; }

   void increaseRank(){ rank++; }

   /** Assign operator. */
   Constraint& operator= (const Constraint& cons);
   /** Less operator. */
   bool operator< (const Constraint& cons) const;
   /** Equals operator. */
   bool operator== (const Constraint& cons) const;

   std::string toString();

   

private:
	ConstraintType type;
	int sJob, eJob;
	int time;
	int eqType;

	int *rank;
};

/**
* Defines the operations needed by the hash object.
*/
class ConstraintHasher : public stdext::hash_compare<Constraint>
{
public:
   size_t operator() (const Constraint& cons) const;
   bool operator() (const Constraint& cons1, const Constraint& cons2) const;
};

class ConstraintRankComparator
{
public:
    bool operator()(const Constraint& c1, const Constraint& c2){
        return c1.getRank() < c2.getRank();
    }
};

/**
* Type definition for the hash object.
*/
typedef stdext::hash_map<Constraint, bool, ConstraintHasher> ConstraintHash;

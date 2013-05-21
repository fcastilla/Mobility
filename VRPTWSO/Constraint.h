#pragma once

#include "gurobi_c++.h"
#include <map>
#include <hash_map>


/** All constraint types */
enum ConstraintType
{
   C_ERROR = 0,
   C_COVER,
   C_SYNCH,
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

   // SET METHODS 
   void reset();
   void setType(ConstraintType t){ type = t; } 
   void setStartJob(int s){ sJob = s; }
   void setEndJob(int e){ eJob = e; }
   void setTime(int t){ time = t; }
   void setEquipmentType(int e){ eqType = e; }

   /** Assign operator. */
   Constraint& operator= (const Constraint& cons);
   /** Less operator. */
   bool operator< (const Constraint& cons) const;
   /** Equals operator. */
   bool operator== (const Constraint& cons) const;

   std::string toString();

   int rank;

private:
	ConstraintType type;
	int sJob, eJob;
	int time;
	int eqType;

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
        return c1.rank < c2.rank;
    }
}consRankComparator;

/**
* Type definition for the hash object.
*/
typedef stdext::hash_map<Constraint, bool, ConstraintHasher> ConstraintHash;

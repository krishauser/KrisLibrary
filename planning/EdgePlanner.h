#ifndef ROBOTICS_EDGE_PLANNER_H
#define ROBOTICS_EDGE_PLANNER_H

#include "CSpace.h"
#include <KrisLibrary/utils/SmartPointer.h>
#include <list>
#include <queue>

/** @ingroup MotionPlanning
 * @brief Abstract base class for an edge planner.
 *
 * NOTE: Sharing pointers between edge planners is dangerous; the only 
 * pointer that can be copied is the configuration space!  Use SmartPointers
 * if data must be shared among instances.
 *
 * IsVisible performs the planning process
 * Eval returns the path p(u) with u from 0->1.  p(0)=a, p(1)=b
 * Start returns p(0)
 * Goal returns p(1)
 * Space() returns the workspace in which this lives
 * Copy returns a copy
 * ReverseCopy returns the reverse edge planner
 * 
 * For incremental planners:
 * Priority returns some priority measure (such as distance btwn configs)
 * Plan performs one step of the planning process, returns true
 *   to continue, false on failure
 * Done returns true if the planning's done
 * Failed returns true if an infeasible configuration has been found
 */
class EdgePlanner
{
public:
  EdgePlanner() {}
  virtual ~EdgePlanner() {}
  virtual bool IsVisible() =0;
  virtual void Eval(Real u,Config& x) const=0;
  virtual const Config& Start() const=0;
  virtual const Config& Goal() const=0;
  virtual CSpace* Space() const=0;
  virtual EdgePlanner* Copy() const=0;
  virtual EdgePlanner* ReverseCopy() const=0;

  //for incremental planners
  virtual Real Priority() const { abort(); return 0; }
  virtual bool Plan() { abort(); return false; }
  virtual bool Done() const { abort(); return false; }
  virtual bool Failed() const { abort(); return false; }
};

/// Edge planner that always is visible 
class TrueEdgePlanner : public EdgePlanner
{
public:
  TrueEdgePlanner(CSpace* space,const Config& x,const Config& y);
  virtual bool IsVisible() { return true; }
  virtual void Eval(Real u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new TrueEdgePlanner(space,a,b); }
  virtual EdgePlanner* ReverseCopy() const { return new TrueEdgePlanner(space,b,a); }

  virtual Real Priority() const { return 0; }
  virtual bool Plan() { return false; }
  virtual bool Done() const { return true; }
  virtual bool Failed() const {  return false; }

  Config a,b;
  CSpace* space;
};

/// Edge planner that is never visible 
class FalseEdgePlanner : public EdgePlanner
{
public:
  FalseEdgePlanner(CSpace* space,const Config& x,const Config& y);
  virtual bool IsVisible() { return false; }
  virtual void Eval(Real u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new FalseEdgePlanner(space,a,b); }
  virtual EdgePlanner* ReverseCopy() const { return new FalseEdgePlanner(space,b,a); }

  virtual Real Priority() const { return 0; }
  virtual bool Plan() { return false; }
  virtual bool Done() const { return true; }
  virtual bool Failed() const {  return true; }

  Config a,b;
  CSpace* space;
};

/// Edge planner that copies its truth value from another planner, but
/// not the space nor the endpoints
class PiggybackEdgePlanner : public EdgePlanner
{
public:
  PiggybackEdgePlanner(CSpace* space,const Config& a,const Config& b,const SmartPointer<EdgePlanner>& e);
  virtual bool IsVisible() { return e->IsVisible(); }
  virtual void Eval(Real u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new PiggybackEdgePlanner(space,a,b,e); }
  virtual EdgePlanner* ReverseCopy() const { return new PiggybackEdgePlanner(space,b,a,e->ReverseCopy()); }

  virtual Real Priority() const { return e->Priority(); }
  virtual bool Plan() { return e->Plan(); }
  virtual bool Done() const { return e->Done(); }
  virtual bool Failed() const { return e->Failed(); }

  Config a,b;
  CSpace* space;
  SmartPointer<EdgePlanner> e;
};

/** @ingroup MotionPlanning
 * @brief Straight-line edge planner that divides the segment until 
 * epsilon is reached.
 */
class StraightLineEpsilonPlanner : public EdgePlanner
{
public:
  StraightLineEpsilonPlanner(CSpace* space,const Config& a,const Config& b,Real epsilon);
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;
  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;

  Config a,b;
  CSpace* space;
  Real epsilon;

protected:
  bool foundInfeasible;
  Real dist;
  int depth;
  int segs;
  Config m;
};

/** @ingroup MotionPlanning
 * @brief Straight-line edge planner that divides the segment until 
 * the segment distance is below CSpace.ObstacleDistance()
 */
class StraightLineObstacleDistancePlanner : public EdgePlanner
{
public:
  StraightLineObstacleDistancePlanner(CSpace* space,const Config& a,const Config& b);
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  bool CheckVisibility(const Config& a,const Config& b,Real da,Real db);

  Config a,b;
  CSpace* space;
};

/** @ingroup MotionPlanning
 * The same as StraightLineEpsilonPlanner, but keeps the bisected configs,
 * and recalculates distances for every subdivision.
 *
 * Used in constrained configuration spaces where CSpace.Midpoint() doesn't
 * necessarily return a configuration whose distance is half of the endpoints.
 */
class BisectionEpsilonEdgePlanner : public EdgePlanner
{
public:
  BisectionEpsilonEdgePlanner(CSpace* space,const Config& a,const Config& b,Real epsilon);
  virtual ~BisectionEpsilonEdgePlanner() {}
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual const Config& Start() const { return path.front(); }
  virtual const Config& Goal() const { return path.back(); }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;
  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;
  //on failure, returns the segment last checked
  bool Plan(Config*& pre,Config*& post);

  const std::list<Config>& GetPath() const { return path; }
  const Config& InfeasibleConfig() const { return x; }

protected:
  BisectionEpsilonEdgePlanner(CSpace* space,Real epsilon);

  std::list<Config> path;
  CSpace* space;
  Real epsilon;

  struct Segment
  {
    inline bool operator < (const Segment& s) const { return length<s.length; }

    std::list<Config>::iterator prev;
    Real length;
  };

  std::priority_queue<Segment,std::vector<Segment> > q;
  Config x;
};

///helper, returns non-NULL if the edge is visible
inline EdgePlanner* IsVisible(CSpace* w,const Config& a,const Config& b)
{
  EdgePlanner* e=w->LocalPlanner(a,b);
  if(e->IsVisible()) return e;
  delete e;
  return NULL;
}

#endif

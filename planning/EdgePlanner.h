#ifndef ROBOTICS_EDGE_PLANNER_H
#define ROBOTICS_EDGE_PLANNER_H

#include "CSpace.h"
#include "Interpolator.h"
#include <memory>
#include <list>
#include <queue>

/** @ingroup MotionPlanning
 * @brief Abstract base class for an edge planner / edge checker (i.e., local planner).
 *
 * There is a subtle difference between edge planners and edge checkers, and for
 * the most part we use checkers as planners.  The specific difference is this:
 * - An edge *planner* creates the path after it is done with planning.
 * - An edge *checker* knows the path upon initialization.
 * 
 * As an example, both EpsilonEdgeChecker and BisectionEpsilonEdgePlanner cut the path
 * into a number of segments and checks each for feasibility.  They are equivalent if
 * the space's Interpolate method is a geodesic, so that the interpolant between any two
 * points on a curve is a subset of the curve itself.  If not, however, the
 * BisectionEpsilonEdgePlanner creates a different path after it is done. This is used,
 * for example, with CSpaces that lie on submanifolds and use projection during
 * interpolation.
 *
 * NOTE: Sharing pointers between edge planners is dangerous; the only 
 * pointer that can be copied is the configuration space!  Use shared_ptr's
 * if data must be shared among instances.
 *
 * There is a difference between one-shot and incremental EdgePlanners.  One-shot EdgePlanners
 * implement the following methods:
 *
 * - IsVisible performs the planning process
 * - Space returns the workspace in which this lives
 * - Copy returns a copy
 * - ReverseCopy returns the reverse edge planner
 * And the following Interpolator methods need implementing too:
 * - Eval returns the path p(u) with u from 0->1.  p(0)=a, p(1)=b
 * - Start returns p(0)
 * - End returns p(1)
 * 
 * In addition, incremental planners implement the following:
 * - Priority returns some priority measure (such as distance btwn configs)
 * - Plan performs one step of the planning process, returns true
 *   to continue, false on failure
 * - Done returns true if the planning's done
 * - Failed returns true if an infeasible configuration has been found
 */
class EdgePlanner : public Interpolator
{
public:
  EdgePlanner() {}
  virtual ~EdgePlanner() {}
  virtual bool IsVisible() =0;
  virtual CSpace* Space() const =0;
  virtual std::shared_ptr<EdgePlanner> Copy() const=0;
  virtual std::shared_ptr<EdgePlanner> ReverseCopy() const=0;

  //for incremental planners
  virtual bool IsIncremental() const { return false; } 
  virtual Real Priority() const { abort(); return 0; }
  virtual bool Plan() { abort(); return false; }
  virtual bool Done() const { abort(); return false; }
  virtual bool Failed() const { abort(); return false; }
};

typedef std::shared_ptr<EdgePlanner> EdgePlannerPtr;

/** @ingroup MotionPlanning
 * @brief An EdgePlanner that just checks a given interpolator or straight line
 * path between two configurations.
 */
class EdgeChecker : public EdgePlanner
{
public:
  EdgeChecker(CSpace* space,const InterpolatorPtr& path);
  EdgeChecker(CSpace* space,const Config& a,const Config& b);
  virtual ~EdgeChecker() {}
  virtual void Eval(Real u,Config& x) const { path->Eval(u,x); }
  virtual Real Length() const { return path->Length(); }
  virtual const Config& Start() const { return path->Start(); }
  virtual const Config& End() const { return path->End(); }
  virtual CSpace* Space() const { return space; }

  CSpace* space;
  InterpolatorPtr path;
};

typedef std::shared_ptr<EdgeChecker> EdgeCheckerPtr;


/** @ingroup MotionPlanning
 * @brief Edge checker that divides the path until epsilon resolution is reached.
 */
class EpsilonEdgeChecker : public EdgeChecker
{
public:
  EpsilonEdgeChecker(CSpace* space,const InterpolatorPtr& path,Real epsilon);
  EpsilonEdgeChecker(CSpace* space,const Config& a,const Config& b,Real epsilon);
  virtual bool IsVisible();
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;
  virtual bool IsIncremental() const { return true; } 
  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;

  Real epsilon;

protected:
  bool foundInfeasible;
  Real dist;
  int depth;
  int segs;
  Config m;
};

/** @ingroup MotionPlanning
 * @brief Edge checker that divides the path  until 
 * the segment distance is below CSpace.ObstacleDistance()
 */
class ObstacleDistanceEdgeChecker : public EdgeChecker
{
public:
  ObstacleDistanceEdgeChecker(CSpace* space,const InterpolatorPtr& path);
  ObstacleDistanceEdgeChecker(CSpace* space,const Config& a, const Config& b);
  virtual bool IsVisible();
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;

  bool CheckVisibility(Real ua,Real ub,const Config& a,const Config& b,Real da,Real db);
};

/** @ingroup MotionPlanning
 * Similar to EpsilonEdgeChecker, but keeps the bisected configs,
 * and recalculates distances for every subdivision.
 *
 * Used in constrained configuration spaces where CSpace.Midpoint() doesn't
 * necessarily return a configuration whose distance is half of the endpoints.
 * An example is recursive projection on a manifold.
 */
class BisectionEpsilonEdgePlanner : public EdgePlanner
{
public:
  BisectionEpsilonEdgePlanner(CSpace* space,const Config& a,const Config& b,Real epsilon);
  virtual ~BisectionEpsilonEdgePlanner() {}
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const;
  virtual const Config& End() const;
  virtual CSpace* Space() const { return space; }
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;
  virtual bool IsIncremental() const { return true; } 
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

  CSpace* space;
  std::list<Config> path;
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
inline EdgePlannerPtr IsVisible(CSpace* w,const Config& a,const Config& b)
{
  EdgePlannerPtr e=w->LocalPlanner(a,b);
  if(e->IsVisible()) return e;
  return EdgePlannerPtr();
}

#endif

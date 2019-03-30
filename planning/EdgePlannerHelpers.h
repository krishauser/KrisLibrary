#ifndef PLANNING_EDGE_PLANNER_UTILS_H
#define PLANNING_EDGE_PLANNER_UTILS_H

#include "EdgePlanner.h"

/// Edge planner that always is visible 
class TrueEdgeChecker : public EdgeChecker
{
public:
  TrueEdgeChecker(CSpace* space,const InterpolatorPtr& path);
  TrueEdgeChecker(CSpace* space,const Config& a,const Config& b);
  virtual bool IsVisible() { return true; }
  virtual EdgePlannerPtr Copy() const { return std::make_shared<TrueEdgeChecker>(space,path); }
  virtual EdgePlannerPtr ReverseCopy() const { return std::make_shared<TrueEdgeChecker>(space,std::make_shared<ReverseInterpolator>(path)); }

  virtual bool IsIncremental() const { return true; } 
  virtual Real Priority() const { return 0; }
  virtual bool Plan() { return false; }
  virtual bool Done() const { return true; }
  virtual bool Failed() const {  return false; }
};

/// Edge planner that is never visible 
class FalseEdgeChecker : public EdgeChecker
{
public:
  FalseEdgeChecker(CSpace* space,const InterpolatorPtr& path);
  FalseEdgeChecker(CSpace* space,const Config& a,const Config& b);
  virtual bool IsVisible() { return false; }
  virtual EdgePlannerPtr Copy() const { return std::make_shared<FalseEdgeChecker>(space,path); }
  virtual EdgePlannerPtr ReverseCopy() const { return std::make_shared<FalseEdgeChecker>(space,std::make_shared<ReverseInterpolator>(path)); }

  virtual bool IsIncremental() const { return true; } 
  virtual Real Priority() const { return 0; }
  virtual bool Plan() { return false; }
  virtual bool Done() const { return true; }
  virtual bool Failed() const {  return true; }
};

/// Edge planner that only checks the endpoint
class EndpointEdgeChecker : public EdgeChecker
{
public:
  EndpointEdgeChecker(CSpace* space,const InterpolatorPtr& path);
  EndpointEdgeChecker(CSpace* space,const Config& a,const Config& b);
  virtual bool IsVisible();
  virtual EdgePlannerPtr Copy() const { return std::make_shared<EndpointEdgeChecker>(space,path); }
  virtual EdgePlannerPtr ReverseCopy() const { return std::make_shared<EndpointEdgeChecker>(space,std::make_shared<ReverseInterpolator>(path)); }
};


/* @ingroup MotionPlanning
 * Edge planner/checker that copies its truth value from another planner. Note:
 * this adds a reference to the input edge planner, or takes ownership if it is
 * a raw pointer.
 * 
 * Optionally, can override the space and the endpoints such that the
 * interpolation is done with the input path, but the work in verification
 * is done by the edge planner e.
 */
class PiggybackEdgePlanner : public EdgeChecker
{
public:
  ///Initialize normally
  PiggybackEdgePlanner(EdgePlannerPtr e);
  ///Initialize in override mode
  PiggybackEdgePlanner(CSpace* space,const InterpolatorPtr& path,EdgePlannerPtr e);
  ///Initialize in override mode
  PiggybackEdgePlanner(CSpace* space,const Config& a,const Config& b,EdgePlannerPtr e);
  virtual ~PiggybackEdgePlanner() {}
  virtual bool IsVisible() { return e->IsVisible(); }
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;

  virtual bool IsIncremental() const { return e->IsIncremental(); } 
  virtual Real Priority() const { return e->Priority(); }
  virtual bool Plan() { return e->Plan(); }
  virtual bool Done() const { return e->Done(); }
  virtual bool Failed() const { return e->Failed(); }

  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const;
  virtual const Config& End() const;
  virtual CSpace* Space() const;

  EdgePlannerPtr e;
};

/** @ingroup MotionPlanning
 * @brief An edge checker that checks a sequence of edges.
 */
class PathEdgeChecker : public EdgePlanner
{
public:
  PathEdgeChecker(CSpace* space,const std::vector<EdgePlannerPtr> & path);
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const;
  virtual const Config& End() const;  
  virtual bool IsVisible();
  virtual CSpace* Space() const { return space; }
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;
  virtual bool IsIncremental() const { return true; } 
  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;

  //TODO: queue them
  CSpace* space;
  std::vector<EdgePlannerPtr > path;
  size_t progress;
  bool foundInfeasible;
};

/** @ingroup MotionPlanning
 * @brief An edge planners that plans a stacked list of edges
 */
class MultiEdgePlanner : public PiggybackEdgePlanner
{
public:
  MultiEdgePlanner(CSpace* space,const InterpolatorPtr& path,const std::vector<EdgePlannerPtr >& components);
};

/** @ingroup MotionPlanning
 * @brief An incremental edge planner that calls a non-incremental edge planner.
 * This adapts one-shot edge planners to be used in lazy motion planners.
 * 
 * The result of IsVisible is cached so multiple Plan(), Done(), and Failed() calls
 * will not re-check.
 */
class IncrementalizedEdgePlanner : public PiggybackEdgePlanner
{
public:
  IncrementalizedEdgePlanner(const EdgePlannerPtr& e);
  virtual bool IsIncremental() const { return true; } 
  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;
  
  bool checked,visible;
};

/** @brief Convenience class for edge planner that holds a smart pointer
 * to a CSpace so that the temporary CSpace is destroyed when the edge is destroyed.
 * Typically used for single-constraint edge checkers as follows:
 *
 * SubsetConstraintCSpace* ospace = new SubsetConstraintCSpace(this,obstacle)
 * return new EdgePlannerWithCSpaceContainer(ospace,new XEdgePlanner(ospace,path))
 */
class EdgePlannerWithCSpaceContainer : public PiggybackEdgePlanner
{
public:
  EdgePlannerWithCSpaceContainer(const std::shared_ptr<CSpace>& space,const EdgePlannerPtr& e);
  virtual ~EdgePlannerWithCSpaceContainer() { }
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;

  std::shared_ptr<CSpace> spacePtr;
};


#endif 
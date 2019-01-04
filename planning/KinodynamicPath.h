#ifndef KINODYNAMIC_PATH_H
#define KINODYNAMIC_PATH_H

#include "EdgePlanner.h"
class KinodynamicSpace;
class ControlSpace;
class SteeringFunction;
typedef Vector State;
typedef Vector ControlInput;


/** @brief Stores a kinodynamic path with piecewise constant controls
 *
 * Must satisfy the property that milestones[i+1] = f(milestones[i],controls[i]), and the 
 * simulation trace is stored in paths[i].  Optionally, edges[i] can store
 * the trajectory checker.
 *
 * The Duration and *Time* methods assume there's some state variable that indexes
 * time.  This variable must be monotonically nondecreasing along the path.
 * by default it is element 0, which is compatible with all the SpaceTime* spaces in
 * TimeCSpace.h
 */
class KinodynamicMilestonePath : public Interpolator
{
 public:
  KinodynamicMilestonePath();
  KinodynamicMilestonePath(const ControlInput& u,const InterpolatorPtr& path);
  virtual const Config& Start() const { return milestones.front(); }
  virtual const Config& End() const { return milestones.back(); }
  virtual void Eval(Real u,Config& q) const { Eval2(u,q); }
  virtual Real Length() const;

  bool Empty() const;
  bool IsConstant() const;
  CSpace* Space() const;
  inline int NumMilestones() const { return milestones.size(); }
  inline const Config& GetMilestone(int i) const { return milestones[i]; }

  void Clear();
  ///Given milestones and controls, creates the paths and path checkers
  void SimulateFromControls(KinodynamicSpace* space);
  ///Given existing milestones and controls, creates the paths
  void MakePaths(ControlSpace* space);
  void MakePaths(KinodynamicSpace* space);
  ///Given existing milestones, controls, and (optionally) paths, creates the path checkers.
  ///If paths are not filled out, SimulateFromControls is called.
  void MakeEdges(KinodynamicSpace* space);
  void Append(const ControlInput& u,KinodynamicSpace* space);
  void Append(const ControlInput& u,const InterpolatorPtr& path,KinodynamicSpace* space);
  void Append(const ControlInput& u,const InterpolatorPtr& path,const EdgePlannerPtr& e);
  void Concat(const KinodynamicMilestonePath& suffix);
  bool IsValid() const;
  bool IsFeasible();

  //evaluates the state, given path parameter [0,1].  Returns the edge index
  int Eval2(Real u,Config& q) const;

  ///For timed path, returns the start time.
  Real StartTime(int timeindex=0) const { return Start()[timeindex]; }
  ///For timed path, returns the end time.
  Real EndTime(int timeindex=0) const { return End()[timeindex]; }
  ///For timed path, returns the duration of the known path
  Real Duration(int timeIndex=0) const { return milestones.back()[timeIndex] - milestones.front()[timeIndex]; }
  //evaluates the state at time t.  Returns the edge index
  int EvalTime(Real t,Config& q,int timeIndex=0) const;
  //splits the path at time t.  Note: Assumes u(timeIndex) is equal to dt!
  void SplitTime(Real t,KinodynamicMilestonePath& before,KinodynamicMilestonePath& after,int timeIndex=0) const;
  //ensures that the time is monotonically increasing
  bool IsValidTime(int timeIndex=0) const;

  /// Tries to shorten the path by connecting subsequent milestones.
  /// Returns # of shortcuts made.  The steering function must be exact!
  int Shortcut(KinodynamicSpace* space,SteeringFunction* fn);
  /// Tries to shorten the path by connecting random points
  /// with a shortcut, for numIters iterations.  Returns # of shortcuts
  /// The steering function must be exact!
  int Reduce(KinodynamicSpace* space,SteeringFunction* fn,int numIters);
  /// Replaces the section of the path between parameters u1 and u2
  /// with a new path.  If u1 < 0 or u2 > Length(), 
  /// erases the corresponding start/goal milestones too.
  /// If the edges of this path are made, the space needs to be passed in
  /// in the 4th argument.
  void Splice(Real u1,Real u2,const KinodynamicMilestonePath& path,KinodynamicSpace* space=NULL); 
  /// Replaces the section of the path between milestones
  /// start and goal with a new path.  If the index is negative,
  /// erases the corresponding start/goal milestones too.
  void Splice2(int start,int goal,const KinodynamicMilestonePath& path); 

  std::vector<State> milestones;
  std::vector<ControlInput> controls;
  std::vector<InterpolatorPtr> paths;
  std::vector<EdgePlannerPtr> edges;
};

#endif

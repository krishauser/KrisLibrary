#ifndef KINODYNAMIC_PATH_H
#define KINODYNAMIC_PATH_H

#include "EdgePlanner.h"
class KinodynamicCSpace;
typedef Vector State;
typedef Vector ControlInput;


/** @brief Stores a kinodynamic path with piecewise constant controls
 *
 * Must satisfy the property that milestones[i+1] = f(milestones[i],controls[i]), and the 
 * simulation trace is stored in paths[i].  Optionally, edges[i] can store
 * the trajectory checker.
 */
class KinodynamicMilestonePath
{
 public:
  bool Empty() const { return edges.empty(); }
  bool IsConstant() const { return milestones.size()==1; }
  CSpace* Space() const { return edges[0]->Space(); }
  const Config& Begin() const { return milestones.front(); }
  const Config& End() const { return milestones.back(); }
  inline int NumMilestones() const { return milestones.size(); }
  inline const Config& GetMilestone(int i) const { return milestones[i]; }

  void Clear();
  void SimulateFromControls(KinodynamicCSpace* space);
  void Append(const ControlInput& u,KinodynamicCSpace* space);
  void Append(const ControlInput& u,const std::vector<State>& path,KinodynamicCSpace* space);
  void Append(const ControlInput& u,const std::vector<State>& path,const SmartPointer<EdgePlanner>& e);
  void Concat(const KinodynamicMilestonePath& suffix);
  bool IsValid() const;

  //returns the cspace distance of the path
  Real PathLength() const;
  //evaluates the state, given path parameter [0,1].  Returns the edge index
  int Eval(Real u,Config& q) const;

  //the following routines assume there's some state variable that indexes
  //time.  This variable must be monotonically nondecreasing along the path.
  Real Duration(int timeIndex=0) const { return milestones.back()[timeIndex] - milestones.front()[timeIndex]; }
  //evaluates the state at time t.  Returns the edge index
  int EvalTime(Real t,Config& q,int timeIndex=0) const;
  //splits the path at time t.  Note: Assumes u(timeIndex) is equal to dt!
  void SplitTime(Real t,KinodynamicMilestonePath& before,KinodynamicMilestonePath& after,int timeIndex=0) const;
  //ensures that the time is monotonically increasing
  bool IsValidTime(int timeIndex) const;

  std::vector<State> milestones;
  std::vector<ControlInput> controls;
  std::vector<std::vector<State> > paths;
  std::vector<SmartPointer<EdgePlanner> > edges;
};

#endif

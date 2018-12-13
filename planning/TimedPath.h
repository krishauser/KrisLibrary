#ifndef ROBOTICS_TIMED_PATH_H
#define ROBOTICS_TIMED_PATH_H

#include "Path.h"

class TimedMilestonePath : public Interpolator
{
public:
  bool Empty() const { return edges.empty(); }
  bool IsConstant() const { return edges.size()==1 && edges[0]->Start()==edges[0]->End(); }
  CSpace* Space() const { return edges[0]->Space(); }
  const Config& Begin() const { return edges[0]->Start(); }
  const Config& End() const { return edges.back()->End(); }
  inline int NumMilestones() const { return edges.size()+1; }
  inline const Config& GetMilestone(int i) const {
    if(i<(int)edges.size()) return edges[i]->Start();
    else return End();
  }
  virtual Real Length() const;
  virtual void Eval(Real t,Config& q) const { Eval2(t,q); }
  virtual Real ParamStart() const { return 0.0; }
  virtual Real ParamEnd() const;

  void SetConstant(const Config& q,CSpace* space);
  void Set(const MilestonePath& path,int timeIndex=-1);
  void Set(const EdgePlannerPtr& e,int timeIndex=-1);
  void Append(const EdgePlannerPtr& e,int timeIndex=-1);
  void AppendDelay(Real t);
  void Clear();
  void Concat(const TimedMilestonePath& path);
  //returns the edge index (-1 if before the path begins, or edges.size() if after the path ends)
  int Eval2(Real t,Config& q) const;
  void Split(Real time,TimedMilestonePath& before,TimedMilestonePath& after) const;

  std::vector<EdgePlannerPtr> edges;
  std::vector<Real> durations;
};

#endif


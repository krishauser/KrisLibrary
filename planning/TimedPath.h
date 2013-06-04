#ifndef ROBOTICS_TIMED_PATH_H
#define ROBOTICS_TIMED_PATH_H

#include "Path.h"

struct TimedMilestonePath
{
  bool Empty() const { return edges.empty(); }
  bool IsConstant() const { return edges.size()==1 && edges[0]->Start()==edges[0]->Goal(); }
  CSpace* Space() const { return edges[0]->Space(); }
  const Config& Begin() const { return edges[0]->Start(); }
  const Config& End() const { return edges.back()->Goal(); }
  inline int NumMilestones() const { return edges.size()+1; }
  inline const Config& GetMilestone(int i) const {
    if(i<(int)edges.size()) return edges[i]->Start();
    else return End();
  }
  Real Length() const;

  void SetConstant(const Config& q,CSpace* space);
  void Set(const MilestonePath& path);
  void Set(const SmartPointer<EdgePlanner>& e);
  void Append(const SmartPointer<EdgePlanner>& e);
  void AppendDelay(Real t);
  void Clear();
  void Concat(const TimedMilestonePath& path);
  //returns the edge index (-1 if before the path begins, or edges.size() if after the path ends)
  int Eval(Real t,Config& q) const;
  void Split(Real time,TimedMilestonePath& before,TimedMilestonePath& after) const;

  vector<SmartPointer<EdgePlanner> > edges;
  vector<Real> durations;
};

#endif


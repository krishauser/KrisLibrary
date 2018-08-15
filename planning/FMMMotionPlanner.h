#ifndef PLANNING_FMM_MOTION_PLANNER_H
#define PLANNING_FMM_MOTION_PLANNER_H

#include "CSpace.h"
#include "Path.h"
#include "FMM.h"

class FMMMotionPlanner
{
 public:
  ///Initialize planner.  Use a dynamic domain, dynamic resolution.
  FMMMotionPlanner(CSpace* space);
  ///Initialize planner.  Use a fixed domain with given resolution divs.
  FMMMotionPlanner(CSpace* space,const Vector& bmin,const Vector& bmax,int divs = 10);
  void Init(const Config& a,const Config& b);
  ///Runs a single instance of the FMM at the current resolution, 
  ///returns true if successful.  (Be sure the set the resolution!
  ///The default is very coarse.)
  bool SolveFMM();
  ///Runs instances of the FMM at increasingly fine resolutions.
  ///Returns true if a new feasible path was successfully found within
  ///the given time cutoff.  Safe to call this multiple times.
  bool SolveAnytime(Real time);

  Vector ToGrid(const Vector& q) const;
  Vector FromGrid(const Vector& q) const;
  Vector FromGrid(const std::vector<int>& pt) const;

  CSpace* space;
  Vector bmin,bmax;
  bool dynamicDomain;
  Vector resolution;
  Config start,goal;
  ArrayND<Real> distances;
  MilestonePath solution;
  //debug: a path that failed the secondary feasibility check
  MilestonePath failedCheck;
};

#endif

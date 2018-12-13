#include "TimedPath.h"

Real TimedMilestonePath::ParamEnd() const
{
  Real t=0;
  for(size_t i=0;i<durations.size();i++) t+=durations[i];
  return t;
}

Real TimedMilestonePath::Length() const
{
  Real len=0;
  for(size_t i=0;i<edges.size();i++)
    len += edges[i]->Length();
  return len;
}

void TimedMilestonePath::SetConstant(const Config& q,CSpace* space)
{
  edges.resize(1);
  durations.resize(1);
  edges[0] = space->LocalPlanner(q,q);
  durations[0] = 0;
}

void TimedMilestonePath::Set(const MilestonePath& path,int timeIndex)
{
  edges = path.edges;
  durations.resize(edges.size());
  for(size_t i=0;i<edges.size();i++) {
    if(timeIndex >= 0) 
      durations[i] = edges[i]->End()[timeIndex] - edges[i]->Start()[timeIndex];
    else
      durations[i] = edges[i]->Length();
  }
}

void TimedMilestonePath::Set(const EdgePlannerPtr& e,int timeIndex)
{
  edges.resize(1);
  durations.resize(1);
  edges[0] = e;
  durations[0] = (timeIndex >= 0 ? e->End()[timeIndex]-e->Start()[timeIndex] : e->Length());
}

void TimedMilestonePath::Append(const EdgePlannerPtr& e,int timeIndex)
{
  if(!edges.empty()) Assert(e->Start()==End());
  edges.push_back(e);
  Real duration = (timeIndex >= 0 ? e->End()[timeIndex]-e->Start()[timeIndex] : e->Length());
  durations.push_back(duration);
}

void TimedMilestonePath::AppendDelay(Real t)
{
  edges.push_back(EdgePlannerPtr(Space()->LocalPlanner(End(),End())));
  durations.push_back(t);
}

void TimedMilestonePath::Clear()
{
  edges.clear();
  durations.clear();
}

void TimedMilestonePath::Concat(const TimedMilestonePath& path)
{
  edges.insert(edges.end(),path.edges.begin(),path.edges.end());
  durations.insert(durations.end(),path.durations.begin(),path.durations.end());
}

int TimedMilestonePath::Eval2(Real t,Config& x) const
{
  Assert(!edges.empty());
  if(t < 0) {
    x = Begin();
    return -1;
  }
  for(size_t i=0;i<edges.size();i++) {
    if(t <= durations[i]) {
      if(durations[i] == 0) x=edges[i]->Start();
      else edges[i]->Eval(t/durations[i],x);
      return (int)i; 
    }
    t -= durations[i];
  }
  //fall through: t is longer than path
  x = End();
  return edges.size();
}


void TimedMilestonePath::Split(Real dt,TimedMilestonePath& before,TimedMilestonePath& after) const
{
  before.edges.resize(0);
  before.durations.resize(0);
  after.edges.resize(0);
  after.durations.resize(0);

  CSpace* cspace = Space();
  if(dt < 0) {  //dt is before path
    before.edges.push_back(cspace->LocalPlanner(Begin(),Begin()));
    before.durations.push_back(0);
    after.edges.push_back(cspace->LocalPlanner(Begin(),Begin()));
    after.durations.push_back(-dt);
  }
 
  for(size_t i=0;i<edges.size();i++) {
    if(dt < 0) {
      after.edges.push_back(edges[i]);
      after.durations.push_back(durations[i]);
    }
    else {
      if(dt <= durations[i]) {
        //cut current path
        Config x;
        if(durations[i] == 0) x=edges[i]->Start();
        else edges[i]->Eval(dt/durations[i],x);
        before.edges.push_back(cspace->LocalPlanner(edges[i]->Start(),x));
        before.durations.push_back(dt);
        after.edges.push_back(cspace->LocalPlanner(x,edges[i]->End()));
        after.durations.push_back(durations[i]-dt);
      }
      else {
        before.edges.push_back(edges[i]);
        before.durations.push_back(durations[i]);
      }
      dt -= durations[i];
    }
  }

  if(dt > 0) {  //dt is longer than path
    before.edges.push_back(cspace->LocalPlanner(End(),End()));
    before.durations.push_back(dt);
    after.edges.push_back(cspace->LocalPlanner(End(),End()));
    after.durations.push_back(0);
  }
}

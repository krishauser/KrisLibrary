#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "KinodynamicPath.h"
#include "KinodynamicCSpace.h"
using namespace std;

void KinodynamicMilestonePath::Clear()
{
  milestones.clear();
  controls.clear();
  edges.clear();
  paths.clear();
}

void KinodynamicMilestonePath::SimulateFromControls(KinodynamicCSpace* space)
{
  Assert(milestones.size()==controls.size()+1);
  edges.resize(controls.size());
  paths.resize(controls.size());
  for(size_t i=0;i<milestones.size();i++) {
    space->Simulate(milestones[i],controls[i],paths[i]);
    edges[i] = space->TrajectoryChecker(paths[i]);
  }
}

void KinodynamicMilestonePath::Append(const ControlInput& u,KinodynamicCSpace* space)
{
  Assert(!milestones.empty());
  vector<State> path;
  space->Simulate(milestones.back(),u,path);
  Append(u,path,space);
}

void KinodynamicMilestonePath::Append(const ControlInput& u,const std::vector<State>& path,KinodynamicCSpace* space)
{
  EdgePlanner* e=space->TrajectoryChecker(path);
  Append(u,path,e);
}

void KinodynamicMilestonePath::Append(const ControlInput& u,const std::vector<State>& path,const SmartPointer<EdgePlanner>& e)
{
  Assert(!path.empty());
  if(!milestones.empty()) {
    if(path.front() != milestones.back()) {
      LOG4CXX_INFO(logger,"KinodynamicMilestonePath::Append: incorrect starting point"<<"\n");
      LOG4CXX_INFO(logger,path.front()<<"\n");
      LOG4CXX_INFO(logger,milestones.back()<<"\n");
    }
  }
  Assert(path.front() == milestones.back());
  milestones.push_back(path.back());
  controls.push_back(u);
  paths.push_back(path);
  edges.push_back(e);
}

void KinodynamicMilestonePath::Concat(const KinodynamicMilestonePath& suffix)
{
  if(milestones.empty()) {
    *this = suffix;
    return;
  }
  Assert(suffix.milestones.front()==milestones.back());
  milestones.insert(milestones.end(),++suffix.milestones.begin(),suffix.milestones.end());
  controls.insert(controls.end(),suffix.controls.begin(),suffix.controls.end());
  paths.insert(paths.end(),suffix.paths.begin(),suffix.paths.end());
  edges.insert(edges.end(),suffix.edges.begin(),suffix.edges.end());
}

bool KinodynamicMilestonePath::IsValid() const
{
  if(milestones.empty()) {
        LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: no milestones\n");
    return false;
  }
  if(milestones.size() != edges.size()+1) {
        LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: wrong sized edges ("<<edges.size()<<"!="<<milestones.size()-1);
    return false;
  }
  if(edges.size() != controls.size()) {
        LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: wrong sized controls ("<<controls.size()<<"!="<<edges.size());
    return false;
  }
  if(edges.size() != paths.size()) {
        LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: wrong sized paths ("<<paths.size()<<"!="<<edges.size());
    return false;
  }
  for(size_t i=0;i<paths.size();i++) {
    if(paths[i].front() != milestones[i]) {
            LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: path "<<i);
      LOG4CXX_ERROR(logger,paths[i].front()<<"\n");
      LOG4CXX_ERROR(logger,milestones[i]<<"\n");
      return false;
    }
    if(paths[i].back() != milestones[i+1]) {
            LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: path "<<i);
      LOG4CXX_ERROR(logger,paths[i].back()<<"\n");
      LOG4CXX_ERROR(logger,milestones[i+1]<<"\n");
      return false;
    }
  }
  for(size_t i=0;i<edges.size();i++) {
    if(edges[i]->Start() != milestones[i]) {
            LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: edge planner "<<i);
      LOG4CXX_ERROR(logger,edges[i]->Start()<<"\n");
      LOG4CXX_ERROR(logger,milestones[i]<<"\n");
      return false;
    }
    if(edges[i]->Goal() != milestones[i+1]) {
            LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValid: edge planner "<<i);
      LOG4CXX_ERROR(logger,edges[i]->Goal()<<"\n");
      LOG4CXX_ERROR(logger,milestones[i+1]<<"\n");
      return false;
    }
  }
  return true;
}

//returns the cspace distance of the path
Real KinodynamicMilestonePath::PathLength() const
{
  Real l=0;
  for(size_t i=0;i<paths.size();i++) {
    CSpace* c = Space();
    for(size_t j=0;j+1<paths[i].size();j++)
      l += c->Distance(paths[i][j],paths[i][j+1]);
  }
  return l;
}

//evaluates the state, given path parameter [0,1]
int KinodynamicMilestonePath::Eval(Real t,Config& c) const
{
  if(t <= Zero || edges.empty()) { c = milestones.front(); return 0; }
  else if(t >= One) { c = milestones.back(); return edges.size()-1; }
  else {
    Real u=t*(Real)edges.size();
    Real u0=Floor(u);
    int index = (int)u0;
    Assert(index >= 0 && index < (int)edges.size());
    edges[index]->Eval(u-u0,c);
    return index;
  }
}


int KinodynamicMilestonePath::EvalTime(Real t,Config& q,int timeIndex) const
{
  if(t <= milestones.front()(timeIndex) || edges.empty()) { q = milestones.front(); return 0; }
  for(size_t i=1;i<milestones.size();i++) {
    Assert(milestones[i](timeIndex) >= milestones[i-1](timeIndex));
    if(t < milestones[i](timeIndex)) {
      Real u = (t-milestones[i-1](timeIndex))/(milestones[i](timeIndex)-milestones[i-1](timeIndex));
      Assert(u >= 0 && u <= 1.0);
      edges[i-1]->Eval(u,q);
      return i-1;
    }
  }
  q = milestones.back();
  return edges.size()-1;
}

struct TimeIndexCmp
{
  int index;
  TimeIndexCmp(int _index):index(_index){}
  bool operator ()(const State& a,const State& b) const { return a(index) < b(index); }
  bool operator ()(Real a,const State& b) const { return a < b(index); }
  bool operator ()(const State& a,Real b) const { return a(index) < b; }
};

void KinodynamicMilestonePath::SplitTime(Real t,KinodynamicMilestonePath& before,KinodynamicMilestonePath& after,int timeIndex) const
{
  if(t <= milestones.front()(timeIndex) || edges.empty()) {
    if(t < milestones.front()(timeIndex)) {
            LOG4CXX_ERROR(logger,"Warning, can't properly split before the beginning of a path\n");
      if(logger->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    }
    before.Clear();
    before.milestones.resize(1);
    before.milestones[0] = milestones.front();
    after = *this;
    return;
  }
  KinodynamicCSpace* space = dynamic_cast<KinodynamicCSpace*>(Space());

  //binary search
  TimeIndexCmp cmp(timeIndex);
  vector<State>::const_iterator i = --std::upper_bound(milestones.begin(),milestones.end(),t,cmp);
  vector<State>::const_iterator n = i; n++;
  int index = i-milestones.begin();
  
  if(n == milestones.end()) {
    Assert(this->IsValid());
    before = *this;
    after.Clear();
    after.milestones.resize(1);
    after.milestones[0] = milestones.back();
    
    if(t!=End()(timeIndex)) {
      Assert(t>End()(timeIndex));
      //the split time is AFTER the end of the path -- need to append a zero control
      before.milestones.push_back(End());
      before.milestones.back()(timeIndex) = t;
      before.controls.resize(before.controls.size()+1);
      before.paths.resize(before.paths.size()+1);
      before.edges.resize(before.edges.size()+1);
      if(!space->ConnectionControl(End(),before.milestones.back(),before.controls.back())) {
	//hack it?
	ControlInput zeroControl;
	space->SampleControl(before.milestones.back(),zeroControl);
	zeroControl.setZero();
	zeroControl(timeIndex) = t - End()(timeIndex);
	before.controls.back() = zeroControl;
      }
      space->Simulate(End(),before.controls.back(),before.paths.back());
      before.paths.back().back() = before.milestones.back();
      before.edges.back() = space->TrajectoryChecker(before.paths.back());
      
      after.milestones[0](timeIndex) = t;
    }

    //sanity check
    Assert(before.IsValid());
    Assert(before.IsValidTime(timeIndex));
    Assert(after.IsValid());
    Assert(after.IsValidTime(timeIndex));
  }
  else {
    Assert((*n)(timeIndex) >= (*i)(timeIndex));
    vector<State>::const_iterator pi = --std::upper_bound(paths[index].begin(),paths[index].end(),t,cmp);
    vector<State>::const_iterator pn = pi; pn++;
    Assert(pi != paths[index].end());
    Assert(pn != paths[index].end());
    Assert((*pi)(timeIndex) <= t && (*pn)(timeIndex) >= t);
    Assert((*pi)(timeIndex) != (*pn)(timeIndex));
    Real u=(t-(*pi)(timeIndex))/((*pn)(timeIndex)-(*pi)(timeIndex));
    Config q;
    space->Interpolate(*pi,*pn,u,q);
    Assert(FuzzyEquals(q(timeIndex),t));
    q(timeIndex) = t;
    int index = i-milestones.begin();

    //do the messy job of splitting
    before.milestones.resize(index+2);
    before.edges.resize(index+1);
    before.paths.resize(index+1);
    before.controls.resize(index+1);
    after.milestones.resize(milestones.size()-index);
    after.edges.resize(milestones.size()-index-1);
    after.paths.resize(milestones.size()-index-1);
    after.controls.resize(milestones.size()-index-1);
    copy(milestones.begin(),milestones.begin()+index+1,before.milestones.begin());
    copy(controls.begin(),controls.begin()+index+1,before.controls.begin());
    copy(paths.begin(),paths.begin()+index+1,before.paths.begin());
    copy(edges.begin(),edges.begin()+index+1,before.edges.begin());
    copy(milestones.begin()+index,milestones.end(),after.milestones.begin());
    copy(controls.begin()+index,controls.end(),after.controls.begin());
    copy(paths.begin()+index,paths.end(),after.paths.begin());
    copy(edges.begin()+index,edges.end(),after.edges.begin());
    //fix up edge i-1
    before.milestones.back() = q;
    after.milestones.front() = q;
    if(!space->ConnectionControl(*i,q,before.controls.back())) {
      //assume controls(timeIndex) is dt?
      before.controls.back()(timeIndex) *= u;
    }
    vector<State>& bpath = before.paths.back();
    for(size_t j=0;j<bpath.size();j++) {
      if(bpath[j](timeIndex) > t) {
	bpath.resize(j+1);
	bpath.back()=q;
	break;
      }
    }
    before.edges.back() = space->TrajectoryChecker(bpath);
    if(!space->ConnectionControl(q,*n,after.controls.front())) {
      //assume controls(timeIndex) is dt?
      after.controls.front()(timeIndex) *= (1.0-u);
    }
    vector<State>& apath = after.paths.front();
    for(size_t j=0;j<apath.size();j++) {
      if(apath[j](timeIndex) > t) {
	apath.erase(apath.begin(),apath.begin()+j-1);
	apath.front()=q;
	break;
      }
    }
    after.edges.front() = space->TrajectoryChecker(apath); 

    //sanity check
    Assert(before.IsValid());
    Assert(before.IsValidTime(timeIndex));
    Assert(after.IsValid());
    Assert(after.IsValidTime(timeIndex));
  }
}


bool KinodynamicMilestonePath::IsValidTime(int timeIndex) const
{
  for(size_t i=0;i+1<milestones.size();i++) {
    if(!(milestones[i](timeIndex) <= milestones[i+1](timeIndex))) {
            LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValidTime: Edge "<<i<<" has time reversed: "<<milestones[i](timeIndex)<<" > "<<milestones[i+1](timeIndex));
      return false;
    }
  }
  for(size_t i=0;i<paths.size();i++) {
    for(size_t j=0;j+1<paths[i].size();j++) {
      if(!(paths[i][j](timeIndex) <= paths[i][j+1](timeIndex))) {
		LOG4CXX_ERROR(logger,"KinodynamicMilestonePath::IsValidTime: Path "<<i<<"["<<j<<"] has time reversed: "<<paths[i][j](timeIndex)<<" > "<<paths[i][j+1](timeIndex));
	return false;
      }
    }
  }
  return true;
}

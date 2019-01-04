#include <KrisLibrary/Logger.h>
#include "KinodynamicPath.h"
#include "KinodynamicSpace.h"
#include "InterpolatorHelpers.h"
#include <KrisLibrary/math/random.h>
using namespace std;

KinodynamicMilestonePath::KinodynamicMilestonePath()
{}

KinodynamicMilestonePath::KinodynamicMilestonePath(const ControlInput& u,const std::shared_ptr<Interpolator>& path)
{
  milestones.resize(2);
  controls.resize(1);
  paths.resize(1);
  controls[0] = u;
  milestones[0] = path->Start();
  milestones[1] = path->End();
  paths[0] = path;
}

bool KinodynamicMilestonePath::Empty() const { return paths.empty(); }
bool KinodynamicMilestonePath::IsConstant() const { return milestones.size()==1; }
CSpace* KinodynamicMilestonePath::Space() const { return edges.empty() ? NULL : edges[0]->Space(); }

void KinodynamicMilestonePath::Clear()
{
  milestones.clear();
  controls.clear();
  edges.clear();
  paths.clear();
}

void KinodynamicMilestonePath::SimulateFromControls(KinodynamicSpace* space)
{
  MakePaths(space);
  edges.resize(controls.size());
  for(size_t i=0;i<controls.size();i++) {
    edges[i] = space->TrajectoryChecker(controls[i],paths[i]);
  }
}

void KinodynamicMilestonePath::MakePaths(ControlSpace* space)
{
  milestones.resize(controls.size()+1);
  paths.resize(controls.size());
  for(size_t i=0;i<controls.size();i++) {
    paths[i] = space->Simulate(milestones[i],controls[i]);
    milestones[i+1] = paths[i]->End();
  }
}

void KinodynamicMilestonePath::MakePaths(KinodynamicSpace* space)
{
  MakePaths(space->controlSpace.get());
}

void KinodynamicMilestonePath::MakeEdges(KinodynamicSpace* space)
{
  if(paths.empty()) {
    SimulateFromControls(space);
    return;
  }
  edges.resize(paths.size());
  for(size_t i=0;i<paths.size();i++) 
    edges[i] = space->TrajectoryChecker(controls[i],paths[i]);
}

bool KinodynamicMilestonePath::IsFeasible()
{
  if(milestones.size()==1) return true;
  if(edges.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsFeasible(): edges are not created\n");
    return false;
  }
  for(size_t i=0;i<milestones.size();i++)
    if(!Space()->IsFeasible(milestones[i])) return false;
  for(size_t i=0;i<edges.size();i++)
    if(!edges[i]->IsVisible()) return false;
  return true;
}

void KinodynamicMilestonePath::Append(const ControlInput& u,KinodynamicSpace* space)
{
  Assert(!milestones.empty());
  InterpolatorPtr path(space->Simulate(milestones.back(),u));
  Append(u,path,space);
}

void KinodynamicMilestonePath::Append(const ControlInput& u,const std::shared_ptr<Interpolator>& path,KinodynamicSpace* space)
{
  EdgePlannerPtr e(space->TrajectoryChecker(u,path));
  Append(u,path,e);
}

void KinodynamicMilestonePath::Append(const ControlInput& u,const std::shared_ptr<Interpolator>& path,const std::shared_ptr<EdgePlanner>& e)
{
  if(!milestones.empty()) {
    if(path->Start() != milestones.back()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"KinodynamicMilestonePath::Append: incorrect starting point");
      LOG4CXX_INFO(KrisLibrary::logger(),path->Start());
      LOG4CXX_INFO(KrisLibrary::logger(),milestones.back());
    }
  }
  Assert(path->Start() == milestones.back());
  milestones.push_back(path->End());
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: no milestones\n");
    return false;
  }
  if(milestones.size() != edges.size()+1) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: wrong sized edges ("<<(int)edges.size()<<"!="<<(int)milestones.size()-1);
    return false;
  }
  if(edges.size() != controls.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: wrong sized controls ("<<(int)controls.size()<<"!="<<(int)edges.size());
    return false;
  }
  if(edges.size() != paths.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: wrong sized paths ("<<(int)paths.size()<<"!="<<(int)edges.size());
    return false;
  }
  for(size_t i=0;i<paths.size();i++) {
    if(paths[i]->Start() != milestones[i]) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: path "<<i);
      LOG4CXX_ERROR(KrisLibrary::logger(),paths[i]->Start());
      LOG4CXX_ERROR(KrisLibrary::logger(),milestones[i]);
      return false;
    }
    if(paths[i]->End() != milestones[i+1]) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: path "<<i);
      LOG4CXX_ERROR(KrisLibrary::logger(),paths[i]->End());
      LOG4CXX_ERROR(KrisLibrary::logger(),milestones[i+1]);
      return false;
    }
  }
  for(size_t i=0;i<edges.size();i++) {
    if(edges[i]->Start() != milestones[i]) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: edge planner "<<i);
      LOG4CXX_ERROR(KrisLibrary::logger(),edges[i]->Start());
      LOG4CXX_ERROR(KrisLibrary::logger(),milestones[i]);
      return false;
    }
    if(edges[i]->End() != milestones[i+1]) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValid: edge planner "<<i);
      LOG4CXX_ERROR(KrisLibrary::logger(),edges[i]->End());
      LOG4CXX_ERROR(KrisLibrary::logger(),milestones[i+1]);
      return false;
    }
  }
  return true;
}

//returns the cspace distance of the path
Real KinodynamicMilestonePath::Length() const
{
  Real l=0;
  for(size_t i=0;i<paths.size();i++) 
    l += paths[i]->Length();
  return l;
}

//evaluates the state, given path parameter [0,1]
int KinodynamicMilestonePath::Eval2(Real t,Config& c) const
{
  if(t <= Zero || edges.empty()) { c = milestones.front(); return 0; }
  else if(t >= One) { c = milestones.back(); return paths.size()-1; }
  else {
    Real u=t*(Real)paths.size();
    Real u0=Floor(u);
    int index = (int)u0;
    Assert(index >= 0 && index < (int)paths.size());
    paths[index]->Eval(u-u0,c);
    return index;
  }
}

int KinodynamicMilestonePath::Shortcut(KinodynamicSpace* space,SteeringFunction* fn)
{
  if(!fn || !fn->IsExact()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::Shortcut: steering function is not exact, cannot shortcut\n");
    return 0;
  }
  int numSplices = 0;
  for(size_t i=0;i+2<milestones.size();i++) {
    KinodynamicMilestonePath segment;
    if(fn->Connect(milestones[i],milestones[i+2],segment)) {
      segment.SimulateFromControls(space);
      if(segment.IsFeasible()) {
        Splice2(i,i+2,segment);
        i = i+segment.paths.size()-1;
        numSplices++;
      }
    }
  }
  return numSplices;
}

int KinodynamicMilestonePath::Reduce(KinodynamicSpace* space,SteeringFunction* fn,int numIters)
{
  if(!fn || !fn->IsExact()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::Reduce: steering function is not exact, cannot shortcut\n");
    return 0;
  }
  if(edges.empty()) MakeEdges(space);
  Assert(!edges.empty());
  //pick random points on the path, connect them if they're visible
  Config x1,x2;
  int i1,i2;
  int numsplices=0;
  for(int iters=0;iters<numIters;iters++) {
    i1 = rand()%edges.size();
    i2 = rand()%edges.size();
    if(i2 < i1) swap(i1,i2);
    else if(i1 == i2) continue;  //if they're on the same segment, forget it

    Real t1=Rand();
    Real t2=Rand();
    edges[i1]->Eval(t1,x1);
    edges[i2]->Eval(t2,x2);
    const Config& a=edges[i1]->Start();
    const Config& b=edges[i2]->End();
    KinodynamicMilestonePath e_x1x2, e_ax1, e_x2b;
    if(!fn->Connect(x1,x2,e_x1x2)) continue;
    e_x1x2.MakeEdges(space);
    if(!e_x1x2.IsFeasible()) continue;

    if(!fn->Connect(a,x1,e_ax1)) continue;
    if(!fn->Connect(x2,b,e_x2b)) continue;
    e_ax1.MakeEdges(space);
    e_x2b.MakeEdges(space);
    if(!e_ax1.IsFeasible()) continue;
    if(!e_x2b.IsFeasible()) continue;
    e_ax1.Concat(e_x1x2);
    e_ax1.Concat(e_x2b);
    Splice2(i1,i2+1,e_ax1);
    numsplices++;
  }
  return numsplices;
}

void KinodynamicMilestonePath::Splice(Real u1,Real u2,const KinodynamicMilestonePath& path,KinodynamicSpace* space)
{
  Assert(path.edges.empty() ==edges.empty());
  int e1 = (int)Floor(paths.size()*u1);
  int e2 = (int)Floor(paths.size()*u2);
  Real s1 = paths.size()*u1 - Real(e1);
  Real s2 = paths.size()*u2 - Real(e2);
  Assert(e1 < (int)paths.size());
  Assert(e2 >= 0);
  KinodynamicMilestonePath spath;
  if(e1 >= 0 && s1 > 0) {
    spath.milestones.resize(2);
    spath.milestones[0] = milestones[e1];
    spath.milestones[1] = path.Start();
    spath.controls.push_back(controls[e1]);
    spath.controls[0] *= s1;  ///assume it the control scales
    spath.paths.push_back( make_shared<TimeRemappedInterpolator>(paths[e1],0,s1));
  }
  spath.Concat(path);
  if(e2 < (int)paths.size() && s2 > 0) {
    spath.milestones.push_back(milestones[e2+1]);
    spath.controls.push_back(controls[e2]);
    spath.controls.back()[0] *= (1.0-s2);  ///assume it the control scales
    spath.paths.push_back( make_shared<TimeRemappedInterpolator>(paths[e2],s2,1));
  }
  if(!edges.empty() && space) spath.MakeEdges(space);
  Splice2(e1,e2+1,spath);
}

void KinodynamicMilestonePath::Splice2(int start,int goal,const KinodynamicMilestonePath& path)
{
  assert(start >= 0 && start < (int)milestones.size());
  assert(goal >= 0 && goal < (int)milestones.size());
  assert(start < goal);
  assert(milestones[start] == path.Start());
  assert(milestones[goal] == path.End());
  assert(edges.empty() == path.edges.empty());
  if(goal - start == (int)path.milestones.size()) { //great, we don't have to do any resizing
    for(size_t i=0;i<path.milestones.size();i++)
      milestones[start+i] = path.milestones[i];
    for(size_t i=0;i<path.paths.size();i++) {
      controls[start+i] = path.controls[i];
      paths[start+i] = path.paths[i];
      if(!edges.empty())
        edges[start+i] = path.edges[i];
    }
  }
  else {
    milestones.erase(milestones.begin()+start+1,milestones.begin()+goal);
    controls.erase(controls.begin()+start,controls.begin()+goal);
    paths.erase(paths.begin()+start,paths.begin()+goal);
    if(!edges.empty())
      edges.erase(edges.begin()+start,edges.begin()+goal);
    milestones.insert(milestones.begin()+start+1,path.milestones.begin()+1,path.milestones.end()-1);
    controls.insert(controls.begin()+start,path.controls.begin(),path.controls.end());
    paths.insert(paths.begin()+start,path.paths.begin(),path.paths.end());
    if(!edges.empty())
      edges.insert(edges.begin()+start,path.edges.begin(),path.edges.end());
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, can't properly split before the beginning of a path\n");
      KrisLibrary::loggerWait();
    }
    before.Clear();
    before.milestones.resize(1);
    before.milestones[0] = milestones.front();
    after = *this;
    return;
  }
  KinodynamicSpace* space = dynamic_cast<KinodynamicSpace*>(Space());
  Assert(space != NULL);
  std::shared_ptr<SteeringFunction> sf = space->controlSpace->GetSteeringFunction();

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
      Vector endMilestone = End();
      endMilestone(timeIndex) = t;
      KinodynamicMilestonePath extension;
      if(!sf || sf->Connect(End(),before.milestones.back(),extension)) {
	//hack it?
	ControlInput zeroControl;
	space->controlSpace->GetControlSet(before.milestones.back())->Sample(zeroControl);
	zeroControl.setZero();
	zeroControl(timeIndex) = t - End()(timeIndex);
  extension = KinodynamicMilestonePath(zeroControl,InterpolatorPtr(space->Simulate(End(),zeroControl)));
      }
      before.Concat(extension);
      
      after.milestones[0](timeIndex) = t;
    }

    //sanity check
    Assert(before.IsValid());
    Assert(before.IsValidTime(timeIndex));
    Assert(after.IsValid());
    Assert(after.IsValidTime(timeIndex));
  }
  else {
    //assume uniformity in the time index
    Assert((*n)(timeIndex) >= (*i)(timeIndex));
    Real u = (t - (*i)(timeIndex)) / ((*n)(timeIndex) - (*i)(timeIndex));
    Config q;
    paths[index]->Eval(u,q);
    Assert(FuzzyEquals(q(timeIndex),t));
    q(timeIndex) = t;
    int index = i-milestones.begin();

    //do the messy job of splitting
    before.milestones.resize(index+2);
    if(!edges.empty()) before.edges.resize(index+1);
    before.paths.resize(index+1);
    before.controls.resize(index+1);
    after.milestones.resize(milestones.size()-index);
    if(!edges.empty()) after.edges.resize(milestones.size()-index-1);
    after.paths.resize(milestones.size()-index-1);
    after.controls.resize(milestones.size()-index-1);
    copy(milestones.begin(),milestones.begin()+index+1,before.milestones.begin());
    copy(controls.begin(),controls.begin()+index+1,before.controls.begin());
    copy(paths.begin(),paths.begin()+index+1,before.paths.begin());
    if(!edges.empty()) copy(edges.begin(),edges.begin()+index+1,before.edges.begin());
    copy(milestones.begin()+index,milestones.end(),after.milestones.begin());
    copy(controls.begin()+index,controls.end(),after.controls.begin());
    copy(paths.begin()+index,paths.end(),after.paths.begin());
    if(!edges.empty()) copy(edges.begin()+index,edges.end(),after.edges.begin());
    //fix up edge i-1
    before.milestones.back() = q;
    after.milestones.front() = q;
    //assume controls(timeIndex) is dt?
    before.controls.back()(timeIndex) *= u;
    before.paths.back().reset(new TimeRemappedInterpolator(paths[index],0,u));
    if(!edges.empty()) before.edges.back() = space->TrajectoryChecker(before.controls.back(),before.paths.back());

    before.controls.front()(timeIndex) *= (1.0-u);
    before.paths.front().reset(new TimeRemappedInterpolator(paths[index],u,1));
    if(!edges.empty()) before.edges.front() = space->TrajectoryChecker(after.controls.front(),after.paths.front());

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
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValidTime: Edge "<<(int)i<<" has time reversed: "<<milestones[i](timeIndex)<<" > "<<milestones[i+1](timeIndex));
      return false;
    }
  }
  for(size_t i=0;i<paths.size();i++) {
    if(!(paths[i]->Start()(timeIndex) <= paths[i]->End()(timeIndex))) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicMilestonePath::IsValidTime: Path "<<(int)i<<" has time reversed: "<<paths[i]->Start()(timeIndex)<<" > "<<paths[i]->End()(timeIndex));
	return false;
    }
  }
  return true;
}


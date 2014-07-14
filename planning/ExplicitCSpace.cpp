#include "ExplicitCSpace.h"
using namespace std;

string ExplicitCSpace::ObstacleName(int obstacle)
{
  char buf[64];
  sprintf(buf,"Obs[%d]",obstacle);
  return buf;
}

bool ExplicitCSpace::IsFeasible(const Config& q)
{
  int n=NumObstacles();
  for(int i=0;i<n;i++)
    if(!IsFeasible(q,i)) return false;
  return true;
}

void ExplicitCSpace::CheckObstacles(const Config& q,vector<bool>& infeasible)
{
  infeasible.resize(NumObstacles());
  for(size_t i=0;i<infeasible.size();i++)
    infeasible[i] = !IsFeasible(q,i);
}

void ExplicitCSpace::GetFeasibleNames(const Config& q,vector<std::string>& names)
{
  names.resize(0);
  vector<bool> constraints;
  CheckObstacles(q,constraints);
  for(size_t i=0;i<constraints.size();i++) {
    if(!constraints[i])
      names.push_back(ObstacleName(i));
  }
}

void ExplicitCSpace::GetInfeasibleNames(const Config& q,vector<std::string>& names)
{
  names.resize(0);
  vector<bool> constraints;
  CheckObstacles(q,constraints);
  for(size_t i=0;i<constraints.size();i++) {
    if(constraints[i])
      names.push_back(ObstacleName(i));
  }
}

void ExplicitCSpace::PrintInfeasibleNames(const Config& q,std::ostream& out,const char* prefix,const char* suffix)
{
  vector<bool> constraints;
  CheckObstacles(q,constraints);
  for(size_t i=0;i<constraints.size();i++) {
    if(constraints[i])
      out<<prefix<<ObstacleName(i)<<suffix;
  }
}

EdgePlanner* ExplicitCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new ExplicitEdgePlanner(this,a,b);
}

SingleObstacleCSpace::SingleObstacleCSpace(ExplicitCSpace* _baseSpace,int _obstacle)
  :baseSpace(_baseSpace),obstacle(_obstacle)
{}

SubsetExplicitCSpace::SubsetExplicitCSpace(ExplicitCSpace* _baseSpace)
  :baseSpace(_baseSpace)
{}

SubsetExplicitCSpace::SubsetExplicitCSpace(ExplicitCSpace* _baseSpace,int obstacle)
  :baseSpace(_baseSpace)
{
  activeSubset.resize(1);
  activeSubset[0] = obstacle;
}

void SubsetExplicitCSpace::EnableAll()
{
  activeSubset.resize(baseSpace->NumObstacles());
  for(size_t i=0;i<activeSubset.size();i++)
    activeSubset[i] = (int)i;
}

void SubsetExplicitCSpace::EnableNone()
{
  activeSubset.clear();
}

bool SubsetExplicitCSpace::IsFeasible(const Config& q,int obstacle)
{
  return baseSpace->IsFeasible(q,activeSubset[obstacle]);
}

EdgePlanner* SubsetExplicitCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new ExplicitEdgePlanner(this,a,b);
  //TODO: hijacking the space pointer is a problem
  //EdgePlanner* e=baseSpace->LocalPlanner(a,b);
  //dynamic_cast<ExplicitEdgePlanner*>(e)->space = this;
  //return e;
}

EdgePlanner* SubsetExplicitCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  return baseSpace->LocalPlanner(a,b,activeSubset[obstacle]);
}

std::string SubsetExplicitCSpace::ObstacleName(int obstacle)
{
  return baseSpace->ObstacleName(activeSubset[obstacle]);
}

Real SubsetExplicitCSpace::ObstacleDistance(const Config& a,int obstacle)
{
  return baseSpace->ObstacleDistance(a,activeSubset[obstacle]);
}

SubgroupExplicitCSpace::SubgroupExplicitCSpace(ExplicitCSpace* _baseSpace)
  :baseSpace(_baseSpace)
{
  Ungroup();
}

void SubgroupExplicitCSpace::Ungroup()
{
  groups.resize(baseSpace->NumObstacles());
  groupNames.resize(baseSpace->NumObstacles());
  for(size_t i=0;i<groups.size();i++) {
    groups[i].resize(1,(int)i);
    groupNames[i] = baseSpace->ObstacleName(i);
  }
}

void SubgroupExplicitCSpace::Group(int i,int j)
{
  int gi=-1,gj=-1;
  for(size_t g=0;g<groups.size();g++) {
    for(size_t k=0;k<groups[g].size();k++) {
      if(groups[g][k] == i) gi = (int)g;
      if(groups[g][k] == j) gj = (int)g;
    }
    if(gi >= 0 && gj >= 0) break;
  }
  if(gi == gj) return;
  groups[gi].insert(groups[gi].end(),groups[gj].begin(),groups[gj].end());
  groupNames[gi] = groupNames[gi]+string("+")+groupNames[gj];
  groups[gj] = groups.back();
  groupNames[gj] = groupNames.back();
  groups.resize(groups.size()-1);
  groupNames.resize(groups.size()-1);
}

bool SubgroupExplicitCSpace::IsFeasible(const Config& q,int obstacle)
{
  for(size_t i=0;i<groups[obstacle].size();i++)
    if(!baseSpace->IsFeasible(q,groups[obstacle][i])) return false;
  return true;
}

EdgePlanner* SubgroupExplicitCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new ExplicitEdgePlanner(this,a,b);
  //TODO: hijack edge planner pointer?
  //EdgePlanner* e=baseSpace->LocalPlanner(a,b);
  //dynamic_cast<ExplicitEdgePlanner*>(e)->space = this;
  //return e;
}

EdgePlanner* SubgroupExplicitCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  SubsetExplicitCSpace* ospace = new SubsetExplicitCSpace(baseSpace);
  ospace->activeSubset=groups[obstacle];
  return new EdgePlannerWithCSpaceContainer(ospace,ospace->LocalPlanner(a,b));

}

Real SubgroupExplicitCSpace::ObstacleDistance(const Config& a,int obstacle)
{
  Real mind = Inf;
  for(size_t i=0;i<groups[obstacle].size();i++)
    mind = Min(mind,baseSpace->ObstacleDistance(a,groups[obstacle][i]));
  return mind;
}



ExplicitEdgePlanner::ExplicitEdgePlanner(ExplicitCSpace* _space,const Config& _a,const Config& _b,bool init)
  :space(_space),a(_a),b(_b)
{
  if(init) {
    obsPlanners.resize(space->NumObstacles());
    for(size_t i=0;i<obsPlanners.size();i++)
      obsPlanners[i] = space->LocalPlanner(a,b,i);
  }
}

bool ExplicitEdgePlanner::IsVisible()
{
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    if(!IsVisible(i))
      return false;
  return true;
}

void ExplicitEdgePlanner::CheckVisible(vector<bool>& visible)
{
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    visible[i] = IsVisible(i);
}

Real ExplicitEdgePlanner::Priority() const
{
  Real p=0;
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    p=Max(p,Priority(i));
  return p;
}

bool ExplicitEdgePlanner::Plan()
{
  bool res=true;
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    if(!Plan(i)) res=false;
  return res;
}

bool ExplicitEdgePlanner::Done() const
{
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    if(!Done(i)) return false;
  return true;
}

bool ExplicitEdgePlanner::Failed() const
{
  int n=space->NumObstacles();
  for(int i=0;i<n;i++)
    if(!Failed(i)) return true;
  return false;
}

bool ExplicitEdgePlanner::IsVisible(int i)
{
  return obsPlanners[i]->IsVisible(); 
}

Real ExplicitEdgePlanner::Priority(int i) const
{
  return obsPlanners[i]->Priority();
}

bool ExplicitEdgePlanner::Plan(int i)
{
  return obsPlanners[i]->Plan();
}

bool ExplicitEdgePlanner::Done(int i) const
{
  return obsPlanners[i]->Done();
}

bool ExplicitEdgePlanner::Failed(int i) const
{
  return obsPlanners[i]->Failed();
}

EdgePlanner* ExplicitEdgePlanner::Copy() const
{
  ExplicitEdgePlanner* e=new ExplicitEdgePlanner(space,a,b,false);
  e->obsPlanners.resize(obsPlanners.size());
  for(size_t i=0;i<e->obsPlanners.size();i++)
    e->obsPlanners[i] = obsPlanners[i]->Copy();
  return e;
}

EdgePlanner* ExplicitEdgePlanner::ReverseCopy() const
{
  ExplicitEdgePlanner* e=new ExplicitEdgePlanner(space,b,a,false);
  e->obsPlanners.resize(obsPlanners.size());
  for(size_t i=0;i<e->obsPlanners.size();i++)
    e->obsPlanners[i] = obsPlanners[i]->ReverseCopy();
  return e;
}


BisectionEpsilonExplicitEdgePlanner::BisectionEpsilonExplicitEdgePlanner(ExplicitCSpace* space,const Config& a,const Config& b,Real eps)
  :ExplicitEdgePlanner(space,a,b,false)
{
  epsilon = eps;
  path.push_back(a);
  path.push_back(b);
  Segment s;
  s.prev = path.begin();
  s.length = space->Distance(a,b);
  q.push(s);
  foundInfeasible.resize(space->NumObstacles(),false);
}

bool BisectionEpsilonExplicitEdgePlanner::IsVisible()
{
  while(!Done()) {
    if(!Plan()) return false;
  }
  return true;
}

bool BisectionEpsilonExplicitEdgePlanner::IsVisible(int i)
{
  if(foundInfeasible[i]) return false;
  //can't do this efficiently!
  vector<bool> visible;
  CheckVisible(visible);
  return visible[i];
}

void BisectionEpsilonExplicitEdgePlanner::CheckVisible(vector<bool>& visible)
{
  while(!Done())
    PlanAll();
  visible.resize(foundInfeasible.size());
  for(size_t i=0;i<foundInfeasible.size();i++)
    visible[i] = !foundInfeasible[i];
}

EdgePlanner* BisectionEpsilonExplicitEdgePlanner::Copy() const
{
  BisectionEpsilonExplicitEdgePlanner* p=new BisectionEpsilonExplicitEdgePlanner(space,path.front(),path.back(),epsilon);
  p->foundInfeasible = foundInfeasible;
  p->path = path;
  if(path.size() == 2 && q.size()==1) {  //uninitialized
    return p;
  }
  if(!Done()) {
    cout<<"Warning: making a copy of a bisection edge planner that is not done!"<<endl;
    Segment s;
    s.prev = p->path.begin();
    s.length = space->Distance(path.front(),path.back());
    p->q.push(s);
    Assert(!p->Done());
    //cout<<"Press any key to continue..."<<endl;
    //getchar();
  }
  return p;
}

EdgePlanner* BisectionEpsilonExplicitEdgePlanner::ReverseCopy() const
{
  BisectionEpsilonExplicitEdgePlanner* p=new BisectionEpsilonExplicitEdgePlanner(space,b,a,epsilon);
  p->foundInfeasible = foundInfeasible;
  p->path.resize(path.size());
  reverse_copy(path.begin(),path.end(),p->path.begin());
  return p;
}

Real BisectionEpsilonExplicitEdgePlanner::Priority() const
{
  if(q.empty()) return 0;
  return q.top().length;
}

bool BisectionEpsilonExplicitEdgePlanner::PlanAll()
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Midpoint(*a,*b,x);
  bool anyFeasible = false;
  for(size_t i=0;i<foundInfeasible.size();i++)
    if(!foundInfeasible[i]) {
      if(!space->IsFeasible(x,i)) { 
	foundInfeasible[i] = true;
      }
      else
	anyFeasible = true;
    }
  if(!anyFeasible) return false;
  list<Config>::iterator m=path.insert(b,x);

  if(q.size()%100 == 0 &&
     Real(q.size())*epsilon > 4.0*space->Distance(Start(),Goal())) {
    s.length = Inf;
    q.push(s);
    cout<<"BisectionEpsilonEdgePlanner: Over 4 times as many iterations as needed, quitting."<<endl;
    cout<<"Original length "<<space->Distance(Start(),Goal())<<", epsilon "<<epsilon<<endl;
    return false;
  }
  //insert the split segments back in the queue
  Real l1=space->Distance(*a,x);
  Real l2=space->Distance(x,*b);
  if(l1 > 0.9*s.length || l2 > 0.9*s.length) {
    //printf("Midpoint exceeded 0.9 time segment distance\n");
    s.length = Inf;
    q.push(s);
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonExplicitEdgePlanner::Plan()
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Midpoint(*a,*b,x);
  for(size_t i=0;i<foundInfeasible.size();i++)
    if(!space->IsFeasible(x,i)) { 
      foundInfeasible[i] = true;
      return false;
    }
  list<Config>::iterator m=path.insert(b,x);

  if(q.size()%100 == 0 &&
     Real(q.size())*epsilon > 4.0*space->Distance(Start(),Goal())) {
    s.length = Inf;
    q.push(s);
    cout<<"BisectionEpsilonEdgePlanner: Over 4 times as many iterations as needed, quitting."<<endl;
    cout<<"Original length "<<space->Distance(Start(),Goal())<<", epsilon "<<epsilon<<endl;
    return false;
  }
  //insert the split segments back in the queue
  Real l1=space->Distance(*a,x);
  Real l2=space->Distance(x,*b);
  if(l1 > 0.9*s.length || l2 > 0.9*s.length) {
    //printf("Midpoint exceeded 0.9 time segment distance\n");
    s.length = Inf;
    q.push(s);
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonExplicitEdgePlanner::Done() const
{
  return q.empty() || q.top().length <= epsilon;
}

bool BisectionEpsilonExplicitEdgePlanner::Failed() const
{
  for(size_t i=0;i<foundInfeasible.size();i++)
    if(foundInfeasible[i]) return true;
  return false;
}

Real BisectionEpsilonExplicitEdgePlanner::Priority(int i) const
{
  return Priority();
}

bool BisectionEpsilonExplicitEdgePlanner::Plan(int i)
{
  if(!foundInfeasible[i])
    return Plan();
  return false;
}

bool BisectionEpsilonExplicitEdgePlanner::Done(int i) const
{
  return foundInfeasible[i] || q.empty() || q.top().length <= epsilon;
}
bool BisectionEpsilonExplicitEdgePlanner::Failed(int i) const
{
  return foundInfeasible[i];
}

EdgePlannerWithCSpaceContainer::EdgePlannerWithCSpaceContainer(SmartPointer<CSpace> space,const SmartPointer<EdgePlanner>& e)
  :PiggybackEdgePlanner(space,e->Start(),e->Goal(),e),spacePtr(space)
{
}

EdgePlanner* EdgePlannerWithCSpaceContainer::Copy() const
{
  return new EdgePlannerWithCSpaceContainer(spacePtr,e->Copy());
}

EdgePlanner* EdgePlannerWithCSpaceContainer::ReverseCopy() const
{
  return new EdgePlannerWithCSpaceContainer(spacePtr,e->ReverseCopy());
}


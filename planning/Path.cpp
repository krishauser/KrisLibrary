#include <KrisLibrary/Logger.h>
#include "Path.h"
#include "Objective.h"
#include <math/random.h>
#include <Timer.h>
#include <errors.h>
using namespace std;

MilestonePath::MilestonePath()
{}

MilestonePath::~MilestonePath()
{}

void MilestonePath::Concat(const MilestonePath& path)
{
  edges.reserve(edges.size()+path.edges.size());
  for(size_t i=0;i<path.edges.size();i++) {
    edges.push_back(path.edges[i]);
  }
}

bool MilestonePath::InitializeEdgePlans()
{
  bool res=true;
  for(size_t i=0;i<edges.size();i++) {
    if(!edges[i]->IsVisible()) res=false;
  }
  return res;
}

void MilestonePath::CreateEdgesFromMilestones(CSpace* space,const vector<Config>& milestones)
{
  Assert(!milestones.empty());
  edges.resize(milestones.size()-1);
  for(size_t i=0;i+1<milestones.size();i++) {
    edges[i] = space->LocalPlanner(milestones[i],milestones[i+1]);
  }
}

bool MilestonePath::IsFeasible()
{
  if(edges.empty()) return true;
  //first check endpoints 
  CSpace* space=Space();
  if(!space->IsFeasible(edges[0]->Start())) return false;
  for(size_t i=0;i<edges.size();i++) {
    if(!space->IsFeasible(edges[i]->End())) return false;
  }
  //then check edges
  for(size_t i=0;i<edges.size();i++) 
    if(!edges[i]->IsVisible()) return false;
  return true;
}

int MilestonePath::Eval2(Real t, Config& c) const
{
  if(t <= Zero) { c = edges.front()->Start(); return 0; }
  else if(t >= One) { c = edges.back()->End(); return edges.size()-1; }
  else {
    Real u=t*(Real)edges.size();
    Real u0=Floor(u);
    int index = (int)u0;
    Assert(index >= 0 && index < (int)edges.size());
    edges[index]->Eval(u-u0,c);
    return index;
  }
}

int MilestonePath::Shortcut()
{
  int numShortcuts=0;
  size_t i=0;
  while(i+1 < edges.size()) {
    //try to connect milestone i to i+2
    const Config& x1=GetMilestone(i);
    const Config& x2=GetMilestone(i+2);
    EdgePlannerPtr e= IsVisible(edges[i]->Space(),x1,x2);
    if(e) {
      edges[i] = e;
      edges.erase(edges.begin()+i+1);
      numShortcuts++;
      //don't advance to next milestone
    }
    else //advance to next milestone
      i++; 
  }
  return numShortcuts;
}

int MilestonePath::Shortcut(ObjectiveFunctionalBase* objective)
{
  if(!objective) return Shortcut();
  int numShortcuts=0;
  size_t i=0;
  while(i+1 < edges.size()) {
    //try to connect milestone i to i+2
    const Config& x1=GetMilestone(i);
    const Config& x2=GetMilestone(i+2);
    EdgePlannerPtr e= IsVisible(edges[i]->Space(),x1,x2);
    if(e) {
      if(objective->IncrementalCost(e.get()) < objective->IncrementalCost(edges[i].get()) + objective->IncrementalCost(edges[i+1].get())) {
        edges[i] = e;
        edges.erase(edges.begin()+i+1);
        numShortcuts++;
        //don't advance to next milestone, keep trying to shortcut
        continue;
      }
    }
    //advance to next milestone
    i++; 
  }
  return numShortcuts;
}


int MilestonePath::Reduce(int numIters)
{
  CSpace* space=Space();
  //pick random points on the path, connect them if they're visible
  Config x1,x2;
  int i1,i2;
  int numsplices=0;
  for(int iters=0;iters<numIters;iters++) {
    i1 = rand()%edges.size();
    i2 = rand()%edges.size();
    if(i2 < i1) swap(i1,i2);
    else if(i1 == i2) {
      continue;  //if they're on the same segment, forget it
    }

    Real t1=Rand();
    Real t2=Rand();
    edges[i1]->Eval(t1,x1);
    edges[i2]->Eval(t2,x2);
    const Config& a=edges[i1]->Start();
    const Config& b=edges[i2]->End();
    EdgePlannerPtr e_x1x2=space->LocalPlanner(x1,x2);
    Timer timer;
    if(e_x1x2->IsVisible()) {
      EdgePlannerPtr e_ax1=space->LocalPlanner(a,x1);
      EdgePlannerPtr e_x2b=space->LocalPlanner(x2,b);
      if(e_ax1->IsVisible() && e_x2b->IsVisible()) {
	numsplices++;
	//LOG4CXX_INFO(KrisLibrary::logger(),"Visible subsegment "<<i1<<"->"<<i2);
	//replace edges a->a',...,b'->b with a->x1,x1->x2,x2->b
	edges.erase(edges.begin()+i1,edges.begin()+i2+1);

	edges.insert(edges.begin()+i1,e_ax1);
	edges.insert(edges.begin()+i1+1,e_x1x2);
	edges.insert(edges.begin()+i1+2,e_x2b);
      }
    }
  }
  return numsplices;
}

int MilestonePath::Reduce(int numIters,ObjectiveFunctionalBase* objective)
{
  if(!objective) return Reduce(numIters);
  CSpace* space=Space();
  //pick random points on the path, connect them if they're visible
  Config x1,x2;
  int i1,i2;
  int numsplices=0;
  for(int iters=0;iters<numIters;iters++) {
    i1 = rand()%edges.size();
    i2 = rand()%edges.size();
    if(i2 < i1) swap(i1,i2);
    else if(i1 == i2) {
      continue;  //if they're on the same segment, forget it
    }

    Real t1=Rand();
    Real t2=Rand();
    edges[i1]->Eval(t1,x1);
    edges[i2]->Eval(t2,x2);
    const Config& a=edges[i1]->Start();
    const Config& b=edges[i2]->End();
    EdgePlannerPtr e_x1x2=space->LocalPlanner(x1,x2);
    Timer timer;
    if(e_x1x2->IsVisible()) {
      EdgePlannerPtr e_ax1=space->LocalPlanner(a,x1);
      EdgePlannerPtr e_x2b=space->LocalPlanner(x2,b);
      if(e_ax1->IsVisible() && e_x2b->IsVisible()) {
        if(objective->IncrementalCost(e_ax1.get()) + objective->IncrementalCost(e_x1x2.get()) + objective->IncrementalCost(e_x2b.get()) < objective->IncrementalCost(edges[i1].get()) + objective->IncrementalCost(edges[i2].get())) {
          numsplices++;
          //LOG4CXX_INFO(KrisLibrary::logger(),"Visible subsegment "<<i1<<"->"<<i2);
          //replace edges a->a',...,b'->b with a->x1,x1->x2,x2->b
          edges.erase(edges.begin()+i1,edges.begin()+i2+1);

          edges.insert(edges.begin()+i1,e_ax1);
          edges.insert(edges.begin()+i1+1,e_x1x2);
          edges.insert(edges.begin()+i1+2,e_x2b);
        }
      }
    }
  }
  return numsplices;
}

void MilestonePath::Discretize(Real h)
{
  for(size_t i=0;i<edges.size();i++) {
    int n=DiscretizeEdge(i,h);
    i+=n-1;
  }
}

int MilestonePath::DiscretizeEdge(int i,Real h)
{
  const EdgePlannerPtr& e=edges[i];
  const Config& a=e->Start();
  const Config& b=e->End();
  CSpace* space=e->Space();
  int numDivs = (int)Ceil(e->Length()/h);
  //don't do anything...
  if(numDivs <= 1) return 1;

  //create a bunch of replacement edges
  Real du=One/numDivs;
  Real u=0;
  Config x1,x2;
  MilestonePath replacement;
  for(int k=0;k<numDivs;k++) {
    if(k==0) x1=a;
    else e->Eval(u,x1);
    if(k+1==numDivs) x2=b;
    else e->Eval(u+du,x2);
    EdgePlannerPtr e2(space->PathChecker(x1,x2));
    if(e2->IsVisible()) 
      replacement.edges.push_back(e2);
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, reparameterized edge "<<i<<" is infeasible");
      replacement.edges.push_back(e2);
    }
    u += du;
  }
  Assert(!replacement.edges.empty());
  Splice(i,i+1,replacement);
  return replacement.edges.size();
}

void MilestonePath::DiscretizeEdge(int i,const vector<Real>& u)
{
  Assert(u.front()==0 && u.back()==1);
  const EdgePlannerPtr& e=edges[i];
  CSpace* space=e->Space();
  if(u.size()==2) return;

  Config x1,x2;
  MilestonePath replacement;
  x1 = e->Start();
  for(size_t k=1;k<u.size();k++) {
    e->Eval(u[k],x2);
    EdgePlannerPtr e2(space->PathChecker(x1,x2));
    if(e2->IsVisible()) 
      replacement.edges.push_back(e2);
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, reparameterized edge "<<i<<" is infeasible");
      replacement.edges.push_back(e2);
    }
    x1 = x2;
  }
  Splice(i,i+1,replacement);
}

bool MilestonePath::IsValid()
{
  if(edges.empty()) return false;
  CSpace* space=Space();
  for(size_t i=0;i<edges.size();i++) {
    if(Space(i) != space) return false;
    if(i!=0) 
      if(edges[i]->Start() != edges[i-1]->End()) return false;
  }
  return true;
}

Real MilestonePath::Length() const
{
  Real len=0;
  for(size_t i=0;i<edges.size();i++)
    len += edges[i]->Length();
  return len;
}

const Config& MilestonePath::GetMilestone(int i) const
{
  Assert(i>=0 && i<=(int)edges.size());
  if(i==(int)edges.size())
    return edges.back()->End();
  return edges[i]->Start();
}

void MilestonePath::SetMilestone(int i,const Config& x)
{
  if(i == 0) {
    //first milestone
    const Config& b=edges[i]->End();
    edges[i] = Space(i)->LocalPlanner(x,b);
  }
  else if(i==(int)edges.size()) {
    Assert(!edges.empty());
    //last milestone
    const Config& a=edges[i-1]->Start();
    edges[i-1] = Space(i-1)->LocalPlanner(a,x);
  }
  else {
    const Config& a=edges[i-1]->Start();
    const Config& b=edges[i]->End();
    edges[i-1] = Space(i-1)->LocalPlanner(a,x);
    edges[i]  = Space(i)->LocalPlanner(x,b);
  }
}

bool MilestonePath::CheckSetMilestone(int i,const Config& x)
{
  if(i == 0) {
    //first milestone
    const Config& b=edges[i]->End();
    EdgePlannerPtr e2=IsVisible(Space(i),x,b);
    if(!e2) return false;
    edges[i] = e2;
    return true;
  }
  else if(i==(int)edges.size()) {
    Assert(!edges.empty());
    //last milestone
    const Config& a=edges[i-1]->Start();
    EdgePlannerPtr e1=IsVisible(Space(i-1),a,x);
    if(!e1) return false;
    edges[i-1] = e1;
    return true;
  }
  else {
    const Config& a=edges[i-1]->Start();
    const Config& b=edges[i]->End();
    EdgePlannerPtr e1=IsVisible(Space(i-1),a,x);
    if(!e1) return false;
    EdgePlannerPtr e2=IsVisible(Space(i),x,b);
    if(!e2) return false; 
    
    edges[i-1] = e1;
    edges[i] = e2;
    return true;
  }
}

void MilestonePath::Splice(int start,int goal,const MilestonePath& path)
{
  if(start >= 0) Assert(start <= (int)edges.size());
  if(goal >= 0) Assert(goal <= (int)edges.size());
  if(start < 0) start=0;
  if(goal < 0) goal=(int)edges.size();
  edges.erase(edges.begin()+start,edges.begin()+goal);
  edges.insert(edges.begin()+start,path.edges.begin(),path.edges.end());
}

bool MilestonePath::Load(istream& in,CSpace* space)
{
  Assert(space != NULL);
  vector<Config> configs;
  int n;
  in>>n;
  if(in.bad()) return false;
  Assert(n > 0);
  configs.reserve(n);
  Config temp;
  for(int i=0;i<n;i++) {
    in>>temp;
    configs.push_back(temp);
  }
  CreateEdgesFromMilestones(space,configs);
  return true;
}

bool MilestonePath::Save(ostream& out)
{
  Assert(!edges.empty());
  out<<edges.size()+1<<endl;
  for(size_t i=0;i<edges.size();i++)
    out<<edges[i]->Start()<<endl;
  out<<edges.back()->End()<<endl;
  return true;
}

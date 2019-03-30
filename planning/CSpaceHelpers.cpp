#include <KrisLibrary/Logger.h>
#include "CSpaceHelpers.h"
#include "CSetHelpers.h"
#include "EdgePlanner.h"
#include "EdgePlannerHelpers.h"
#include "InterpolatorHelpers.h"
#include <KrisLibrary/structs/FixedSizeHeap.h>
#include <KrisLibrary/graph/IO.h>
#include <KrisLibrary/graph/DirectedGraph.h>
#include <KrisLibrary/graph/Callback.h>
#include <KrisLibrary/math/AABB.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math/stacking.h>
#include <KrisLibrary/Timer.h>
using namespace Math;
using namespace std;

GeodesicCSpaceAdaptor::GeodesicCSpaceAdaptor(const shared_ptr<GeodesicSpace>& _geodesic)
:geodesic(_geodesic)
{}


CartesianCSpace::CartesianCSpace(int _d)
:d(_d)
{}

void CartesianCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  vector<double> d(c.size());
  SampleHyperBall(r,d);
  x = c + Vector(d);
}

PiggybackCSpace::PiggybackCSpace(CSpace* _baseSpace)
    :baseSpace(_baseSpace)
{
}
int PiggybackCSpace::NumDimensions() { assert(baseSpace!=NULL); return baseSpace->NumDimensions(); }
int PiggybackCSpace::NumConstraints() {
  if(baseSpace) return baseSpace->NumConstraints(); 
  return CSpace::NumConstraints();
}
std::string PiggybackCSpace::VariableName(int i) {
  if(baseSpace) return baseSpace->VariableName(i);
  return CSpace::VariableName(i);
}

std::string PiggybackCSpace::ConstraintName(int i) {
  if(baseSpace) return baseSpace->ConstraintName(i);
  return CSpace::ConstraintName(i);
}

shared_ptr<CSet> PiggybackCSpace::Constraint(int i) 
{
  if(baseSpace) return baseSpace->Constraint(i);
  return CSpace::Constraint(i);
}
void PiggybackCSpace::Sample(Config& x) {
  Assert(baseSpace!=NULL);
  baseSpace->Sample(x);
}
void PiggybackCSpace::SampleNeighborhood(const Config& c,Real r,Config& x) {
  if(baseSpace!=NULL) baseSpace->SampleNeighborhood(c,r,x);
  else CSpace::SampleNeighborhood(c,r,x);
}
EdgePlannerPtr PiggybackCSpace::LocalPlanner(const Config& a,const Config& b) {
  Assert(baseSpace != NULL);
  return baseSpace->LocalPlanner(a,b);
}
EdgePlannerPtr PiggybackCSpace::PathChecker(const Config& a,const Config& b) {
  Assert(baseSpace != NULL);
  return baseSpace->PathChecker(a,b);
}
EdgePlannerPtr PiggybackCSpace::PathChecker(const Config& a,const Config& b,int obstacle) {
  Assert(baseSpace != NULL);
  return baseSpace->PathChecker(a,b,obstacle);
}
bool PiggybackCSpace::IsFeasible(const Config& x) {
  if(baseSpace) return baseSpace->IsFeasible(x);
  else return true;
}
bool PiggybackCSpace::IsFeasible(const Config& x,int constraint) {
  if(baseSpace) return baseSpace->IsFeasible(x,constraint);
  else return true;
}
bool PiggybackCSpace::ProjectFeasible(Config& x) {
  if(baseSpace) return baseSpace->ProjectFeasible(x);
  else return false;
}
Optimization::NonlinearProgram* PiggybackCSpace::FeasibleNumeric() {
  if(baseSpace) return baseSpace->FeasibleNumeric();
  else return NULL;
}
Real PiggybackCSpace::Distance(const Config& x, const Config& y) {
  if(baseSpace) return baseSpace->Distance(x,y);
  else return CSpace::Distance(x,y);
}
void PiggybackCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out) {
  if(baseSpace) baseSpace->Interpolate(x,y,u,out);
  else CSpace::Interpolate(x,y,u,out);
}
void PiggybackCSpace::Midpoint(const Config& x,const Config& y,Config& out) {
  if(baseSpace) baseSpace->Midpoint(x,y,out);
  else CSpace::Midpoint(x,y,out);
}
void PiggybackCSpace::Properties(PropertyMap& map) {
  if(baseSpace) baseSpace->Properties(map);
  else CSpace::Properties(map);
}


BoxCSpace::BoxCSpace(Real xmin,Real xmax,int d)
:CartesianCSpace(d),bmin(d,xmin),bmax(d,xmax)
{
  for(int i=0;i<d;i++) 
    AddConstraint(VariableName(i)+"_bound",new AxisRangeSet(i,xmin,xmax));
}

BoxCSpace::BoxCSpace(const Vector& _bmin,const Vector& _bmax)
:CartesianCSpace(_bmin.n),bmin(_bmin),bmax(_bmax)
{
  for(int i=0;i<bmin.n;i++) 
    AddConstraint(VariableName(i)+"_bound",new AxisRangeSet(i,bmin[i],bmax[i]));
}

void BoxCSpace::SetDomain(const Vector& _bmin,const Vector& _bmax)
{
  Assert(_bmin.n == bmin.n);
  Assert(_bmax.n == bmax.n);
  bmin = _bmin;
  bmax = _bmax;
  for(size_t i=0;i<constraints.size();i++) {
    AxisRangeSet* a = dynamic_cast<AxisRangeSet*>(&*constraints[i]);
    a->low = _bmin[i];
    a->high = _bmax[i];
  }
}

void BoxCSpace::GetDomain(Vector& _bmin,Vector& _bmax)
{
  _bmin.setRef(bmin);
  _bmax.setRef(bmax);
}

void BoxCSpace::Sample(Config& x) 
{
  x.resize(bmin.n);
  for(int i=0;i<bmin.n;i++)
    x(i) = Rand(bmin[i],bmax[i]);
}

bool BoxCSpace::ProjectFeasible(Config& x)
{
  AABBClamp(x,bmin,bmax);
  return true;
}

void BoxCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  CartesianCSpace::SampleNeighborhood(c,r,x);
  AABBClamp(x,bmin,bmax);
}

void BoxCSpace::Properties(PropertyMap& pmap)
{
  CartesianCSpace::Properties(pmap);
  pmap.setArray("minimum",vector<double>(bmin));
  pmap.setArray("maximum",vector<double>(bmax));
  pmap.set("diameter",Distance(bmin,bmax));
}

EdgePlannerPtr BoxCSpace::PathChecker(const Config& a,const Config& b)
{
  return make_shared<TrueEdgeChecker>(this,a,b);
}

MultiCSpace::MultiCSpace()
{}

MultiCSpace::MultiCSpace(const shared_ptr<CSpace>& space1,const shared_ptr<CSpace>& space2)
{
  components.resize(2);
  components[0] = space1;
  components[1] = space2;
  componentNames.resize(2);
  componentNames[0] = "space1";
  componentNames[1] = "space2";
}
  
MultiCSpace::MultiCSpace(const std::vector<shared_ptr<CSpace> >& _components)
:components(_components)
{
  componentNames.resize(components.size());
  for(size_t i=0;i<components.size();i++) {
    stringstream ss;
    ss<<"space"<<i;
    componentNames[i] = ss.str();
  }
}

void MultiCSpace::Add(const std::string& componentName,const shared_ptr<CSpace>& space,Real distanceWeight)
{
  componentNames.push_back(componentName);
  components.push_back(space);
  if(distanceWeight != 1 && distanceWeights.empty()) 
    distanceWeights.resize(components.size()-1);
  if(!distanceWeights.empty())
    distanceWeights.push_back(distanceWeight);
}

void MultiCSpace::FlattenConstraints()
{
  constraintNames.resize(0);
  constraints.resize(0);
  int n=0;
  for(size_t i=0;i<components.size();i++) {
    int d=components[i]->NumDimensions();
    for(int j=0;j<components[i]->NumConstraints();j++) {
      std::string name;
      if(componentNames[i].empty()) name = components[i]->ConstraintName(j);
      else name = componentNames[i]+"-"+components[i]->ConstraintName(j);
      CSpace::AddConstraint(name,new SubspaceSet(components[i]->Constraint(j),n,n+d));
    }
    n += d;
  }
}

int MultiCSpace::NumConstraints()
{
  if(!constraints.empty()) return (int)constraints.size();
  int nc=0;
  for(size_t i=0;i<components.size();i++)
    nc += components[i]->NumConstraints();
  return nc;
}

std::string MultiCSpace::ConstraintName(int index)
{
  if(!constraints.empty()) return constraintNames[index];
  for(size_t i=0;i<components.size();i++) {
    int nc = components[i]->NumConstraints();
    if(index < nc) {
      if(componentNames[i].empty()) return components[i]->ConstraintName(index);
      else return componentNames[i]+"-"+components[i]->ConstraintName(index);
    }
    index -= nc;
  }
  return "ERROR, INVALID CONSTRAINT INDEX";
}

std::shared_ptr<CSet> MultiCSpace::Constraint(int index)
{
  if(!constraints.empty()) return constraints[index];
  int n=0;
  for(size_t i=0;i<components.size();i++) {
    int nc = components[i]->NumConstraints();
    int d = components[i]->NumDimensions();
    if(index < nc) {
      return make_shared<SubspaceSet>(components[i]->Constraint(index),n,n+d);
    }
    index -= nc;
    n += d;
  }
  return NULL;
}

void MultiCSpace::AddConstraint(int spaceIndex,const std::string& name,CSet::CPredicate test)
{
  AddConstraint(spaceIndex,name,new CSet(test));
}

void MultiCSpace::AddConstraint(int spaceIndex,const std::string& name,CSet* constraint)
{
  AddConstraint(spaceIndex,name,shared_ptr<CSet>(constraint));
}

void MultiCSpace::AddConstraint(int spaceIndex,const std::string& name,const shared_ptr<CSet>& constraint)
{
  Assert(!constraints.empty());
  Assert(spaceIndex >= 0 && spaceIndex < (int)components.size());
  components[spaceIndex]->AddConstraint(name,constraint);
  int n=0;
  for(int i=0;i<spaceIndex;i++) 
    n += components[i]->NumDimensions();
  int d = components[spaceIndex]->NumDimensions();
  std::string cname;
  if(componentNames[spaceIndex].empty()) cname = name;
  else cname = componentNames[spaceIndex]+"-"+name;
  CSpace::AddConstraint(cname,new SubspaceSet(constraint,n,n+d));
}

void MultiCSpace::Split(const Vector& x,std::vector<Vector>& items)
{
  items.resize(components.size());
  int n=0;
  for(size_t i=0;i<items.size();i++) {
    int d = components[i]->NumDimensions();
    Assert(n + d <= x.n);
    items[i].resize(d);
    x.getSubVectorCopy(n,items[i]);
    n += d;
  }
  Assert(n == x.n);
}


void MultiCSpace::SplitRef(const Vector& x,std::vector<Vector>& items)
{
  items.resize(components.size());
  int n=0;
  for(size_t i=0;i<items.size();i++) {
    int d = components[i]->NumDimensions();
    Assert(n + d <= x.n);
    items[i].setRef(x,n,1,d);
    n += d;
  }
  Assert(n == x.n);
}

void MultiCSpace::Join(const std::vector<Vector>& items,Vector& x)
{
  Stack(items,x);
}

int MultiCSpace::NumDimensions() 
{
  int n=0;
  for(size_t i=0;i<components.size();i++) 
    n += components[i]->NumDimensions();
  return n;
}

int MultiCSpace::NumIntrinsicDimensions()
{
  int n=0;
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(g) n += g->NumIntrinsicDimensions();
    else n += components[i]->NumDimensions();
  }
  return n;
}

std::string MultiCSpace::VariableName(int i)
{
  for(size_t j=0;j<components.size();j++) {
    if(i < components[j]->NumDimensions()) {
      if(componentNames[j].empty()) return components[j]->VariableName(i);
      return componentNames[j]+"."+components[j]->VariableName(i);
    }
    i -= components[j]->NumDimensions();
  }
  return "Invalid variable specified";
}
  

void MultiCSpace::Sample(Config& x)
{
  x.resize(NumDimensions());
  vector<Config> items;
  SplitRef(x,items);
  for(size_t i=0;i<components.size();i++) 
    components[i]->Sample(items[i]);
}

void MultiCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(NumDimensions());
  vector<Config> citems,xitems;
  SplitRef(c,citems);
  SplitRef(x,xitems);
  for(size_t i=0;i<components.size();i++) 
    components[i]->SampleNeighborhood(citems[i],r,xitems[i]);
}

bool MultiCSpace::IsFeasible(const Config& x)
{
  if(constraints.empty()) return IsFeasible_Independent(x);
  else return CSpace::IsFeasible(x);
}

bool MultiCSpace::IsFeasible_Independent(const Config& x)
{
  vector<Config> xitems;
  SplitRef(x,xitems);
  for(size_t i=0;i<components.size();i++) 
    if(!components[i]->IsFeasible(xitems[i])) return false;
  return true;
}

bool MultiCSpace::ProjectFeasible(Config& x)
{
  vector<Config> xitems;
  SplitRef(x,xitems);
  for(size_t i=0;i<components.size();i++) 
    if(!components[i]->ProjectFeasible(xitems[i])) return false;
  return true;
}

Optimization::NonlinearProgram* MultiCSpace::FeasibleNumeric()
{
  //TODO
  return NULL;
}

EdgePlannerPtr MultiCSpace::LocalPlanner(const Config& a,const Config& b)
{
  if(constraints.empty()) return LocalPlanner_Independent(a,b);
  else return CSpace::LocalPlanner(a,b);
}

EdgePlannerPtr MultiCSpace::LocalPlanner_Independent(const Config& a,const Config& b)
{
  vector<Config> aitems,bitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  vector<EdgePlannerPtr > items(components.size());
  for(size_t i=0;i<components.size();i++)
    items[i] = components[i]->LocalPlanner(aitems[i],bitems[i]);
  return make_shared<MultiEdgePlanner>(this,make_shared<CSpaceInterpolator>(this,a,b),items);
}

EdgePlannerPtr MultiCSpace::PathChecker(const Config& a,const Config& b)
{
  if(constraints.empty()) return PathChecker_Independent(a,b);
  else return CSpace::PathChecker(a,b);
}

EdgePlannerPtr MultiCSpace::PathChecker_Independent(const Config& a,const Config& b)
{
  vector<Config> aitems,bitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  vector<EdgePlannerPtr > items(components.size());
  for(size_t i=0;i<components.size();i++)
    items[i] = components[i]->PathChecker(aitems[i],bitems[i]);
  return make_shared<MultiEdgePlanner>(this,make_shared<CSpaceInterpolator>(this,a,b),items);
}


EdgePlannerPtr MultiCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  if(constraints.empty()) return PathChecker_Independent(a,b,obstacle);
  else return CSpace::PathChecker(a,b,obstacle);
}

EdgePlannerPtr MultiCSpace::PathChecker_Independent(const Config& a,const Config& b,int constraint)
{
  int n=0;
  for(size_t i=0;i<components.size();i++) {
    int nc = components[i]->NumConstraints();
    int d = components[i]->NumDimensions();
    if(constraint < nc) {
      Config ai,bi;
      ai.setRef(a,n,1,d);
      bi.setRef(b,n,1,d);
      EdgePlannerPtr e = components[i]->PathChecker(ai,bi,constraint);
      return make_shared<PiggybackEdgePlanner>(this,a,b,e);
    }
    constraint -= nc;
    n += d;
  }
  FatalError("Invalid constraint specified");
  return NULL;
}

Real MultiCSpace::Distance(const Config& x, const Config& y)
{
  vector<Config> aitems,bitems;
  SplitRef(x,aitems);
  SplitRef(y,bitems);
  Real d = 0;
  for(size_t i=0;i<components.size();i++) {
    Real di = components[i]->Distance(aitems[i],bitems[i]);
    if(distanceWeights.empty()) d += Sqr(di);
    else d += distanceWeights[i]*Sqr(di);
  }
  return Sqrt(d);
}

void MultiCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  out.resize(NumDimensions());
  vector<Config> aitems,bitems,outitems;
  SplitRef(x,aitems);
  SplitRef(y,bitems);
  SplitRef(out,outitems);
  for(size_t i=0;i<components.size();i++) 
    components[i]->Interpolate(aitems[i],bitems[i],u,outitems[i]);
}

void MultiCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

Real MultiCSpace::ObstacleDistance(const Config& a)
{
  vector<Config> aitems;
  SplitRef(a,aitems);
  Real dmin = Inf;
  for(size_t i=0;i<components.size();i++) {
    Real di = components[i]->ObstacleDistance(aitems[i]);
    dmin = Min(di,dmin);
  }
  return dmin;
}

void MultiCSpace::Properties(PropertyMap& map)
{
  //TODO: how do you combine common properties?
  Real volume = 1.0;
  Real diameter = 0.0;
  bool cartesian = true;
  bool geodesic = true;
  bool euclidean = true;
  vector<Vector> cspaceweights(components.size());
  vector<Vector> cspaceminimum(components.size());
  vector<Vector> cspacemaximum(components.size());
  for(size_t i=0;i<components.size();i++) {
    PropertyMap cmap;
    components[i]->Properties(cmap);
    int ccartesian;
    int cgeodesic;
    double cvolume;
    if(!cmap.get("cartesian",ccartesian) || ccartesian==0)
      cartesian = false;
    if(!cmap.get("geodesic",cgeodesic) || cgeodesic==0)
      geodesic = false;
    if(!cmap.get("volume",cvolume))
      volume = 0;
    else
      volume *= cvolume;
    string m;
    Real wi = (distanceWeights.empty() ? 1 : distanceWeights[i]);
    vector<double> w;
    if(cmap.getArray("metricWeights",w))
      cspaceweights[i] = Vector(w)*wi;
    else
      cspaceweights[i].resize(components[i]->NumDimensions(),wi);
    if(!cmap.get("metric",m) || m=="euclidean") {
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"MultiCSpace: component "<<i<<" (dim "<<components[i]->NumDimensions()<<") uses non-euclidean metric");
      euclidean = false;
    }
    vector<double> cmin,cmax;
    if(cmap.getArray("minimum",cmin))
      cspaceminimum[i] = cmin;
    if(cmap.getArray("maximum",cmax))
      cspacemaximum[i] = cmax;
    if(!cmap.get("diameter",cvolume))
      diameter = -Inf;
    else if(!distanceWeights.empty())
      diameter += distanceWeights[i]*cvolume;
  }
  if(volume > 0) map.set("volume",volume);
  if(diameter >= 0) map.set("diameter",diameter);
  if(cartesian) map.set("cartesian",1);
  if(geodesic) map.set("geodesic",1);
  if(euclidean) map.set("metric","euclidean");
  for(size_t i=0;i<cspaceweights.size();i++)
    if(cspaceweights[i].empty()) { cspaceweights.clear(); break;}
  for(size_t i=0;i<cspaceminimum.size();i++)
    if(cspaceminimum[i].empty()) { cspaceminimum.clear(); break;}
  for(size_t i=0;i<cspacemaximum.size();i++)
    if(cspacemaximum[i].empty()) { cspacemaximum.clear(); break;}
  if(!cspaceweights.empty()) {
    Vector temp;
    Join(cspaceweights,temp);
    map.setArray("metricWeights",vector<double>(temp));
  }
  else
    LOG4CXX_INFO(KrisLibrary::logger(),"No weighting?\n");
  if(!cspaceminimum.empty()) {
    Vector temp;
    Join(cspaceminimum,temp);
    map.setArray("minimum",vector<double>(temp));
  }
  if(!cspacemaximum.empty()) {
    Vector temp;
    Join(cspacemaximum,temp);
    map.setArray("maximum",vector<double>(temp));
  }
}

void MultiCSpace::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx)
{
  dx.resize(NumDimensions());
  vector<Config> aitems,bitems,outitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  SplitRef(dx,outitems);
  CartesianCSpace gdefault(0);
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(!g) { g = &gdefault; gdefault.d = components[i]->NumDimensions(); }
    g->InterpolateDeriv(aitems[i],bitems[i],u,outitems[i]);
  }
}
void MultiCSpace::InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx)
{
  dx.resize(NumDimensions());
  vector<Config> aitems,bitems,citems,outitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  SplitRef(da,citems);
  SplitRef(dx,outitems);
  CartesianCSpace gdefault(0);
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(!g) { g = &gdefault; gdefault.d = components[i]->NumDimensions(); }
    g->InterpolateDerivA(aitems[i],bitems[i],u,citems[i],outitems[i]);
  }
}

void MultiCSpace::InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx)
{
  dx.resize(NumDimensions());
  vector<Config> aitems,bitems,citems,outitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  SplitRef(db,citems);
  SplitRef(dx,outitems);
  CartesianCSpace gdefault(0);
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(!g) { g = &gdefault; gdefault.d = components[i]->NumDimensions(); }
    g->InterpolateDerivB(aitems[i],bitems[i],u,citems[i],outitems[i]);
  }
}

void MultiCSpace::InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx)
{
  ddx.resize(NumDimensions());
  vector<Config> aitems,bitems,outitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  SplitRef(ddx,outitems);
  CartesianCSpace gdefault(0);
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(!g) { g = &gdefault; gdefault.d = components[i]->NumDimensions(); }
    g->InterpolateDeriv2(aitems[i],bitems[i],u,outitems[i]);
  } 
}

void MultiCSpace::Integrate(const Config& a,const Vector& da,Config& b)
{
  b.resize(NumDimensions());
  vector<Config> aitems,bitems,daitems;
  SplitRef(a,aitems);
  SplitRef(b,bitems);
  SplitRef(da,daitems);
  CartesianCSpace gdefault(0);
  for(size_t i=0;i<components.size();i++) {
    GeodesicCSpace* g = dynamic_cast<GeodesicCSpace*>(&*components[i]);
    if(!g) { g = &gdefault; gdefault.d = components[i]->NumDimensions(); }
    g->Integrate(aitems[i],daitems[i],bitems[i]);
  } 
}




void UpdateStats(AdaptiveCSpace::PredicateStats& s,double testcost,bool testtrue,double strength=1.0)
{
  double newcount = s.count+strength;
  if(newcount == 0) newcount=1;
  double oldweight = s.count/newcount;
  double newweight = 1.0-oldweight;
  s.cost = oldweight*s.cost + newweight*testcost;
  if(testtrue)
    s.probability = oldweight*s.probability + newweight;
  else
    s.probability = oldweight*s.probability;
  s.count += strength;
}

Real ExpectedANDTestCost(const vector<AdaptiveCSpace::PredicateStats>& stats,const vector<int>& order)
{
  double c=0;
  double p=1;
  if(order.empty()) {
    for(size_t i=0;i<stats.size();i++) {
      c += p*stats[i].cost;
      p *= stats[i].probability;
    }
    return c;
  }
  else {
    for(size_t i=0;i<order.size();i++) {
      int k=order[i];
      Assert(k >= 0 && k < (int)stats.size());
      c += p*stats[k].cost;
      p *= stats[k].probability;
    }
  }
  return c;
}

Real ExpectedANDTestProbability(const vector<AdaptiveCSpace::PredicateStats>& stats)
{
  double p=1;
  for(size_t i=0;i<stats.size();i++) 
    p *= stats[i].probability;
  return p;
}

void OptimizeTestingOrder(const vector<AdaptiveCSpace::PredicateStats>& stats,const vector<vector<int> >& deps,vector<int>& order)
{
  vector<pair<double,int> > priority(stats.size());
  for(size_t i=0;i<stats.size();i++) {
    priority[i].first = stats[i].cost / (1.0-stats[i].probability);
    if(IsNaN(priority[i].first)) priority[i].first = 0.0;
    priority[i].second = (int)i;
  }
  if(deps.empty()) {
    sort(priority.begin(),priority.end());
    order.resize(stats.size());
    for(size_t i=0;i<stats.size();i++) 
      order[i] = priority[i].second;
  }
  else {
    //more complicated version with dependency graph
    Graph::DirectedGraph<int,int> G;
    for(size_t i=0;i<stats.size();i++) 
      G.AddNode(int(i));
    for(size_t i=0;i<stats.size();i++) 
      for(size_t j=0;j<deps[i].size();j++)
        G.AddEdge(deps[i][j],i);
    //compute topological sort
    Graph::TopologicalSortCallback<int> callback;
    G.DFS(callback);
    if(callback.hasCycle) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"motionplanning: WARNING: Test dependency order has cycles... breaking arbitrarily\n");
    }
    //revise priorities from bottom up based off of children
    //a parent node's priority is the minimum of (cost of this + cost of best child) / (1-probability of this*probability of best child)
    vector<int> bestChild(stats.size(),-1);
    vector<double> depcosts(stats.size());
    vector<double> depprobs(stats.size());
    for(size_t i=0;i<stats.size();i++) {
      depcosts[i] = stats[i].cost;
      depprobs[i] = stats[i].probability;
    }
    //reverse gives bottom up traversal
    std::reverse(callback.list.begin(),callback.list.end());
    for(list<int>::const_iterator i=callback.list.begin();i!=callback.list.end();i++) {
      if(G.OutDegree(*i) > 0) {
        //has dependencies...
        double bestPriority = Inf;
        int best = -1;
        Graph::EdgeIterator<int> e;
        for(G.Begin(*i,e);!e.end();e++) {
          int j=e.target();
          if(G.InDegree(j) > 1) //has more than one dependency
            LOG4CXX_ERROR(KrisLibrary::logger(),"motionplanning: WARNING: Constraint "<< j <<" has multiple dependencied including "<< *i<<", can't really optimize yet\n");          
          double priority = (depcosts[*i]+depcosts[j])/(1.0-depprobs[*i]*depprobs[j]);
          if(priority < bestPriority || best < 0) {
            best = j;
            bestPriority = priority;
          }
        }
        depcosts[*i] = depcosts[*i] + depcosts[best];
        depprobs[*i] = depprobs[*i] * depprobs[best];
        priority[*i].first = bestPriority;
      }
    }
    //now search the graph from top down, extracting the lowest priority items from fringe
    order.resize(0);
    order.reserve(stats.size());
    FixedSizeHeap<double> queue(stats.size());
    vector<bool> visited(stats.size(),false);
    for(size_t i=0;i<stats.size();i++) 
      if(G.InDegree(i) == 0) {
        queue.push(int(i),-priority[i].first);
      }
    while(true) {
      int i;
      if(!queue.empty()) {
        i=queue.top();
        queue.pop();
      }
      else {
        //should only get here with cycles... break them in arbitrary order
        i = -1;
        for(size_t j=0;j<visited.size();j++)
          if(!visited[j]) {
            i=(int)j;
            break;
          }
        if(i < 0) break; //done
      }
      visited[i] = true;
      order.push_back(i);
      //go through constraints dependent on i and put them on the queue
      Graph::EdgeIterator<int> e;
      for(G.Begin(i,e);!e.end();e++) {
        int j=e.target();
        queue.push(j,-priority[j].first);
      }
    }
  }
  /*
  //Debugging
  LOG4CXX_INFO(KrisLibrary::logger(),"Costs:");
  for(size_t i=0;i<stats.size();i++) 
    LOG4CXX_INFO(KrisLibrary::logger()," "<<stats[i].cost);
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  LOG4CXX_INFO(KrisLibrary::logger(),"Probabilities:");
  for(size_t i=0;i<stats.size();i++) 
    LOG4CXX_INFO(KrisLibrary::logger()," "<<stats[i].probability);
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  LOG4CXX_INFO(KrisLibrary::logger(),"Dependencies:");
  for(size_t i=0;i<deps.size();i++)  {
    LOG4CXX_INFO(KrisLibrary::logger()," [");
    for(size_t j=0;j<deps[i].size();j++) {
      if(j > 0) LOG4CXX_INFO(KrisLibrary::logger(),",");
      LOG4CXX_INFO(KrisLibrary::logger(),""<<deps[i][j]);
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"]");
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  LOG4CXX_INFO(KrisLibrary::logger(),"Order:");
  for(size_t i=0;i<order.size();i++) 
    LOG4CXX_INFO(KrisLibrary::logger()," "<<order[i]);
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  //KrisLibrary::loggerWait();
  */
}

AdaptiveCSpace::AdaptiveCSpace(CSpace* baseSpace)
:PiggybackCSpace(baseSpace),adaptive(true),useBaseVisibleTest(true)
{
  CopyConstraints(baseSpace);
  baseVisibleStats.cost = 0;
  baseVisibleStats.count = 0;
  baseVisibleStats.probability = 0;
}

bool AdaptiveCSpace::IsFeasible(const Config& x)
{
  if(feasibleTestOrder.empty()) return PiggybackCSpace::IsFeasible(x);
  for(size_t i=0;i<feasibleTestOrder.size();i++)
    if(!IsFeasible_NoDeps(x,feasibleTestOrder[i])) return false;
  return true;
}

void AdaptiveCSpace::GetFeasibleDependencies(int obstacle,vector<int>& deps,bool recursive) const
{
  if(recursive) {
    deps.resize(0);
    vector<int> dfs(1,obstacle);
    vector<bool> used(feasibleTestDeps.size(),false);
    used[obstacle] = true;
    while(!dfs.empty()) {
      int n=dfs.back();
      dfs.resize(dfs.size()-1);
      deps.push_back(n);
      for(size_t i=0;i<feasibleTestDeps[n].size();i++) {
        int c = feasibleTestDeps[obstacle][i];
        if(!used[c]) {
          dfs.push_back(c);
          used[c] = true;
        }
      }
    }
    std::reverse(deps.begin(),deps.end());
    //tkake out obstacle
    deps.resize(deps.size()-1);
  }
  else {
    deps = feasibleTestDeps[obstacle];
  }
}

void AdaptiveCSpace::GetVisibleDependencies(int obstacle,vector<int>& deps,bool recursive) const
{
  if(recursive) {
    deps.resize(0);
    vector<int> dfs(1,obstacle);
    vector<bool> used(visibleTestDeps.size(),false);
    used[obstacle] = true;
    while(!dfs.empty()) {
      int n=dfs.back();
      dfs.resize(dfs.size()-1);
      deps.push_back(n);
      for(size_t i=0;i<visibleTestDeps[n].size();i++) {
        int c = visibleTestDeps[obstacle][i];
        if(!used[c]) {
          dfs.push_back(c);
          used[c] = true;
        }
      }
    }
    std::reverse(deps.begin(),deps.end());
    //tkake out obstacle
    deps.resize(deps.size()-1);
  }
  else {
    deps = visibleTestDeps[obstacle];
  }
}

bool AdaptiveCSpace::IsFeasible_NoDeps(const Config& x,int obstacle)
{
  if(!adaptive) return PiggybackCSpace::IsFeasible(x,obstacle);
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();  
  Timer timer;
  bool res = PiggybackCSpace::IsFeasible(x,obstacle);
  UpdateStats(feasibleStats[obstacle],timer.ElapsedTime(),res);
  return res;
}

bool AdaptiveCSpace::IsFeasible(const Config& x,int obstacle)
{
  for(size_t i=0;i<feasibleTestDeps[obstacle].size();i++)
    if(!IsFeasible(x,feasibleTestDeps[obstacle][i])) return false;
  return IsFeasible_NoDeps(x,obstacle);
}

void AdaptiveCSpace::CheckConstraints(const Config& x,std::vector<bool>& satisfied)
{
  satisfied.resize(constraints.size());
  for(size_t i=0;i<constraints.size();i++)
    satisfied[i] = IsFeasible_NoDeps(x,i);
}

class StatUpdatingEdgePlanner : public PiggybackEdgePlanner
{
 public:
  AdaptiveCSpace::PredicateStats* stats;
  StatUpdatingEdgePlanner(const EdgePlannerPtr& _base,AdaptiveCSpace::PredicateStats* _stats)
  : PiggybackEdgePlanner(_base),stats(_stats)
  {}
  virtual bool IsVisible()
  { 
    Timer timer;
    bool res = PiggybackEdgePlanner::IsVisible();
    if(stats) UpdateStats(*stats,timer.ElapsedTime(),res);
    return res;
  }
};

EdgePlannerPtr AdaptiveCSpace::PathChecker(const Config& a,const Config& b)
{
  if(!adaptive) {
    return PiggybackCSpace::PathChecker(a,b);
  }
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();  
  if(useBaseVisibleTest) {
    return make_shared<StatUpdatingEdgePlanner>(PiggybackCSpace::PathChecker(a,b),&baseVisibleStats);
  }
  std::vector<EdgePlannerPtr> obstacleEdges(constraints.size());
  for(size_t i=0;i<visibleTestOrder.size();i++) {
    obstacleEdges[i] = PathChecker(a,b,visibleTestOrder[i]);
  }
  return make_shared<PathEdgeChecker>(this,obstacleEdges);
}

EdgePlannerPtr AdaptiveCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  if(!visibleTestDeps.empty()) {
    if(!visibleTestDeps[obstacle].empty()) {LOG4CXX_ERROR(KrisLibrary::logger(),"AdaptiveCSpace: Warning, single-obstacle path checker has dependent visibility tests\n");}
    else if(!feasibleTestDeps[obstacle].empty()) {LOG4CXX_ERROR(KrisLibrary::logger(),"AdaptiveCSpace: Warning, single-obstacle path checker has dependent feasibility tests\n");}
  }
  return PathChecker_NoDeps(a,b,obstacle);
}

EdgePlannerPtr AdaptiveCSpace::PathChecker_NoDeps(const Config& a,const Config& b,int obstacle)
{
  if(!adaptive) return PiggybackCSpace::PathChecker(a,b,obstacle);
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();  
  if(useBaseVisibleTest) return make_shared<StatUpdatingEdgePlanner>(PiggybackCSpace::PathChecker(a,b,obstacle),&baseVisibleStats);
  return make_shared<StatUpdatingEdgePlanner>(PiggybackCSpace::PathChecker(a,b,obstacle),&visibleStats[obstacle]);
}

void AdaptiveCSpace::SetupAdaptiveInfo()
{
  constraintMap.clear();
  for(size_t i=0;i<constraints.size();i++) {
    if(constraintMap.count(constraintNames[i])!= 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AdaptiveCSpace: warning, constraint name "<<constraintNames[i].c_str());
      LOG4CXX_INFO(KrisLibrary::logger(),"  named dependencies may not work correctly\n");
    }
    constraintMap[constraintNames[i]] = (int)i;
  }
  feasibleStats.resize(constraints.size());
  visibleStats.resize(constraints.size());
  for(size_t i=0;i<feasibleStats.size();i++) {
    feasibleStats[i].cost = 0;
    feasibleStats[i].probability = 0.5;
    feasibleStats[i].count = 0;
  }
  for(size_t i=0;i<visibleStats.size();i++) {
    visibleStats[i].cost = 0;
    visibleStats[i].probability = 0.5;
    visibleStats[i].count = 0;
  }
  feasibleTestOrder.resize(constraints.size());
  for(size_t i=0;i<feasibleTestOrder.size();i++)
    feasibleTestOrder[i] = (int)i;
  for(size_t i=0;i<visibleTestOrder.size();i++)
    visibleTestOrder[i] = (int)i;
  baseVisibleStats.cost = 0;
  baseVisibleStats.probability = 0.5;
  baseVisibleStats.count = 0;
}

bool AdaptiveCSpace::AddFeasibleDependency(int cindex,int dindex)
{
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();
  if(feasibleTestDeps.empty())
    feasibleTestDeps.resize(constraints.size());
  if(dindex <= cindex) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"AdaptiveCSpace: Warning, added dependency of feasibility test "<<cindex<<" on "<<dindex);
  }
  feasibleTestDeps[cindex].push_back(dindex);
  return true;
}


bool AdaptiveCSpace::AddVisibleDependency(int cindex,int dindex)
{
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();
  if(visibleTestDeps.empty())
    visibleTestDeps.resize(constraints.size());
  visibleTestDeps[cindex].push_back(dindex);
  return true;
}

bool AdaptiveCSpace::AddFeasibleDependency(const char* name,const char* dependency)
{
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();
  if(constraintMap.count(name)==0 || constraintMap.count(dependency)==0) return false;
  if(feasibleTestDeps.empty())
    feasibleTestDeps.resize(constraints.size());
  int cindex = constraintMap[name];
  int dindex = constraintMap[dependency]; 
  feasibleTestDeps[cindex].push_back(dindex);
  return true;
}

bool AdaptiveCSpace::AddVisibleDependency(const char* name,const char* dependency)
{
  if(feasibleStats.size() != constraints.size()) SetupAdaptiveInfo();
  if(constraintMap.count(name)==0 || constraintMap.count(dependency)==0) return false;
  if(visibleTestDeps.empty())
    visibleTestDeps.resize(constraints.size());
  int cindex = constraintMap[name];
  int dindex = constraintMap[dependency]; 
  visibleTestDeps[cindex].push_back(dindex);
  return true;
}

void AdaptiveCSpace::OptimizeQueryOrder() {
  if(!adaptive) return;
  OptimizeTestingOrder(feasibleStats,feasibleTestDeps,feasibleTestOrder);
  OptimizeTestingOrder(visibleStats,visibleTestDeps,visibleTestOrder);
  useBaseVisibleTest = (ExpectedANDTestCost(visibleStats,visibleTestOrder) > baseVisibleStats.cost);
}

void AdaptiveCSpace::GetStats(PropertyMap& stats) const
{
  for(size_t i=0;i<feasibleStats.size();i++) {
    if(feasibleStats[i].count > 0) {
      stats.set(constraintNames[i]+"_feasible_time",feasibleStats[i].cost);
      stats.set(constraintNames[i]+"_feasible_probability",feasibleStats[i].probability);
      stats.set(constraintNames[i]+"_feasible_count",feasibleStats[i].count);
    }
  }
  for(size_t i=0;i<visibleStats.size();i++) {
    if(visibleStats[i].count > 0) {
      stats.set(constraintNames[i]+"_visible_time",visibleStats[i].cost);
      stats.set(constraintNames[i]+"_visible_probability",visibleStats[i].probability);
      stats.set(constraintNames[i]+"_visible_count", visibleStats[i].count);
    }
  }
}

void AdaptiveCSpace::LoadStats(const PropertyMap& stats)
{
  for(size_t i=0;i<feasibleStats.size();i++) {
    if(!stats.get(constraintNames[i]+"_feasible_time",feasibleStats[i].cost))
      feasibleStats[i].cost = 0;
    if(!stats.get(constraintNames[i]+"_feasible_probability",feasibleStats[i].probability))
      feasibleStats[i].probability = 0.5;
    if(!stats.get(constraintNames[i]+"_feasible_count",feasibleStats[i].count))
      feasibleStats[i].count = 0;
  }
  for(size_t i=0;i<visibleStats.size();i++) {
    if(!stats.get(constraintNames[i]+"_visible_time",visibleStats[i].cost))
      visibleStats[i].cost = 0;
    if(!stats.get(constraintNames[i]+"_visible_probability",visibleStats[i].probability))
      visibleStats[i].probability = 0.5;
    if(!stats.get(constraintNames[i]+"_visible_count", visibleStats[i].count))
      visibleStats[i].count = 0;
  }
}

class CSpaceConstraintSet : public CSet
{
public:
  CSpace* space;
  int constraint;
  CSpaceConstraintSet(CSpace* _space,int _constraint)
  :space(_space),constraint(_constraint)
  {}
  virtual ~CSpaceConstraintSet () {}
  virtual int NumDimensions() const { return space->constraints[constraint]->NumDimensions(); }
  virtual bool Contains(const Config& x) { return space->IsFeasible(x,constraint); }
  virtual bool Project(Config& x) { return space->constraints[constraint]->Project(x); }
  virtual bool IsSampleable() const { return space->constraints[constraint]->IsSampleable(); }
  virtual void Sample(Config& x) { space->constraints[constraint]->Sample(x); }
  virtual Optimization::NonlinearProgram* Numeric() { return space->constraints[constraint]->Numeric(); }
  virtual bool IsConvex() const { return space->constraints[constraint]->IsConvex(); }
  virtual Real ObstacleDistance(const Config& x) { return space->constraints[constraint]->ObstacleDistance(x); }
};

SubsetConstraintCSpace::SubsetConstraintCSpace(CSpace* baseSpace,const std::vector<int>& constraintIndices)
:PiggybackCSpace(baseSpace),activeConstraints(constraintIndices)
{
  for(size_t i=0;i<activeConstraints.size();i++)
    AddConstraint(baseSpace->ConstraintName(activeConstraints[i]),new CSpaceConstraintSet(baseSpace,activeConstraints[i]));
}

SubsetConstraintCSpace::SubsetConstraintCSpace(CSpace* baseSpace,int constraint)
:PiggybackCSpace(baseSpace),activeConstraints(1,constraint)
{
  AddConstraint(baseSpace->ConstraintName(constraint),new CSpaceConstraintSet(baseSpace,constraint));
}

EdgePlannerPtr SubsetConstraintCSpace::PathChecker(const Config& a,const Config& b)
{
  vector<EdgePlannerPtr > ocheckers(activeConstraints.size());
  for(size_t i=0;i<activeConstraints.size();i++)
    ocheckers[i] = baseSpace->PathChecker(a,b,activeConstraints[i]);
  return make_shared<PathEdgeChecker>(this,ocheckers);
}

EdgePlannerPtr SubsetConstraintCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  return baseSpace->PathChecker(a,b,activeConstraints[obstacle]);
}




/** @brief Create a single-obstacle edge checker that uses discretization.
 */
EdgePlannerPtr MakeSingleConstraintEpsilonChecker(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon)
{
  auto ospace = make_shared<SubsetConstraintCSpace>(space,obstacle);
  return make_shared<EdgePlannerWithCSpaceContainer>(ospace,make_shared<EpsilonEdgeChecker>(ospace.get(),a,b,epsilon));
};


/** @brief Create a single-obstacle edge checker that uses repeated bisection.
 */
EdgePlannerPtr MakeSingleConstraintBisectionPlanner(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon)
{
  auto ospace = make_shared<SubsetConstraintCSpace>(space,obstacle);
  return make_shared<EdgePlannerWithCSpaceContainer>(ospace,make_shared<BisectionEpsilonEdgePlanner>(ospace.get(),a,b,epsilon));
};

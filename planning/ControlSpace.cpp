#include <KrisLibrary/Logger.h>
#include "ControlSpace.h"
#include "CSetHelpers.h"
#include "EdgePlanner.h"
#include "KinodynamicPath.h"
#include "InterpolatorHelpers.h"
#include <math/diffeq.h>
#include <math/random.h>
#include <sstream>
using namespace std;

std::string ControlSpace::VariableName(int i)
{
  stringstream ss;
  ss<<"u"<<i;
  return ss.str();
}

/* 
class MultiControlSpace : public ControlSpace
{
public:
  MultiControlSpace();
  MultiControlSpace(const std::vector<int>& istateStarts,const std::vector<std::shared_ptr<ControlSpace> >& spaces);
  MultiControlSpace(const MultiCSpace* space,const std::vector<std::shared_ptr<ControlSpace> >& spaces);
  void Add(int istatemin,int istatemax,const std::shared_ptr<ControlSpace>& item);
  void Add(CSpace* space,const std::shared_ptr<ControlSpace>& item);
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  std::vector<pair<int,int>  istateRanges;
  std::vector<std::shared_ptr<ControlSpace> > components;
};
*/

IntegratedControlSet::IntegratedControlSet(const std::shared_ptr<CSet>& _base,Real _dtmax)
:timeSelection(Biased),base(_base),dtmax(_dtmax)
{}

int IntegratedControlSet::NumDimensions() const
{
  int nd = base->NumDimensions();
  if(nd < 0) return -1;
  return nd+1;
}

bool IntegratedControlSet::Project(Config& u)
{
  Vector ubase;
  ubase.setRef(u,1,1,u.n-1);
  if(!base->Project(ubase)) return false;
  if(u(0) < 0) u(0) = 0;
  return true;
}

bool IntegratedControlSet::IsSampleable() const
{
  return base->IsSampleable();
}

void IntegratedControlSet::Sample(ControlInput& u)
{
  ControlInput ubase;
  base->Sample(ubase);
  u.resize(ubase.n+1);
  if(timeSelection == Uniform)
    u(0) = Rand()*dtmax;
  else if(timeSelection == Maximum)
    u(0) = dtmax;
  else
    u(0) = Pow(Rand(),1.0/ubase.n)*dtmax;
  u.copySubVector(1,ubase);
}
 bool IntegratedControlSet::Contains(const ControlInput& u)
 {
  Vector ubase;
  ubase.setRef(u,1,1,u.n-1);
  if(!base->Contains(ubase)) return false;
  //NOTE: no limit on maximum time step
  return u(0) >= 0;
}



IntegratedControlSpace::IntegratedControlSpace(const std::shared_ptr<CSet>& _controlSet,Real _dt,Real _dtmax)
:myDynamics(NULL),type(Euler),space(NULL),controlSet(_controlSet),dt(_dt),dtmax(_dtmax)
{
  myControlSet.reset(new IntegratedControlSet(controlSet,dtmax));
}

IntegratedControlSpace::IntegratedControlSpace(DynamicsFn f,const std::shared_ptr<CSet>& _controlSet,Real _dt,Real _dtmax)
:myDynamics(f),type(Euler),space(NULL),controlSet(_controlSet),dt(_dt),dtmax(_dtmax)
{
  myControlSet.reset(new IntegratedControlSet(controlSet,dtmax));
}

std::string IntegratedControlSpace::VariableName(int i)
{
  if(i==0) return "time_step";
  else return ControlSpace::VariableName(i-1);
}

void IntegratedControlSpace::SetBaseControlSet(std::shared_ptr<CSet> baseControlSet)
{
  IntegratedControlSet* iset = dynamic_cast<IntegratedControlSet*>(&*myControlSet);
  iset->base = baseControlSet;
}

std::shared_ptr<CSet> IntegratedControlSpace::GetBaseControlSet()
{
  IntegratedControlSet* iset = dynamic_cast<IntegratedControlSet*>(&*myControlSet);
  return iset->base;
}

void IntegratedControlSpace::SetGeodesicSpace(GeodesicSpace* _space)
{
  space = _space;
}

//hooks for the math library integrators
struct IntegrationFunction : public DiffEqFunction
{
  IntegrationFunction(IntegratedControlSpace* _space,const ControlInput& _u)
    :space(_space),u(_u)
  {}

  virtual void Eval(Real t,const Vector& y,Vector& dy) {
    space->Derivative(y,u,dy);
  }

  IntegratedControlSpace* space;
  const ControlInput& u;
};


InterpolatorPtr IntegratedControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  UpdateIntegrationParameters(x0);

  std::vector<State> p;
  Real udt = u(0);
  ControlInput ubase;
  ubase.setRef(u,1,1,u.n-1);
  IntegrationFunction func(this,ubase);
  Real h;
  int numSteps = int(Ceil(udt / dt));
  h = udt/numSteps;
  State temp;
  p.push_back(x0);
  int i;
  switch(type) {
  case Euler:
    for(i=0;i<numSteps;i++) {
      Euler_step(&func,0,h,p.back(),temp);
      p.push_back(temp);
    }
    break;
  case RK4:
    for(i=0;i<numSteps;i++) {
      RungeKutta4_step(&func,0,h,p.back(),temp);
      p.push_back(temp);
    }
    break;
  default:
    FatalError("Unknown integrator type!");
    break;
  }
  //TODO: differential equations on geodesics?
  //if(space)
  //  return new PiecewiseLinearCSpaceInterpolator(space,p);
  return make_shared<PiecewiseLinearInterpolator>(p);
}

std::shared_ptr<CSet> IntegratedControlSpace::GetControlSet(const Config& x)
{
  UpdateIntegrationParameters(x);
  IntegratedControlSet* s = dynamic_cast<IntegratedControlSet*>(&*myControlSet);
  Assert(s != NULL);
  s->dtmax = dtmax;
  return myControlSet;
}

void IntegratedControlSpace::Derivative(const State& x, const ControlInput& u,State& dx)
{
  if(myDynamics) myDynamics(x,u,dx);
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"IntegratedControlSpace: no dynamics function provided\n");
    dx.resize(x.n);
    dx.set(0.0);
  }
}


class KinematicControlSet : public NeighborhoodSet
{
public:
  KinematicControlSet(CSpace* space,const Config& center,Real radius)
  :NeighborhoodSet(space,center,radius)
  {}
  virtual bool Contains( const Config& x) { return true; }
};

class KinematicSteeringFunction : public SteeringFunction
{
public:
  std::shared_ptr<CSpace> space;
  KinematicSteeringFunction(const std::shared_ptr<CSpace>& _space) :space(_space) {}
  virtual bool Connect(const Config& x,const Config& y,KinodynamicMilestonePath& path) {
    path = KinodynamicMilestonePath(y,make_shared<CSpaceInterpolator>(space.get(),x,y));
    return true; 
  }
};


class ReverseKinematicControlSpace : public ControlSpace
{
public:
  ReverseKinematicControlSpace(KinematicControlSpace* space);
  virtual std::shared_ptr<CSet> GetControlSet(const Config& x);
  virtual InterpolatorPtr Simulate(const State& x1, const ControlInput& u);
  virtual void Successor(const State& x1, const ControlInput& u,State& x0);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  KinematicControlSpace* space;
};

ReverseKinematicControlSpace::ReverseKinematicControlSpace(KinematicControlSpace* _space)
:space(_space)
{}
std::shared_ptr<CSet> ReverseKinematicControlSpace::GetControlSet(const Config& x)
{ return std::shared_ptr<CSet>(new KinematicControlSet(space->base.get(),x,space->maxNeighborhoodRadius));}
InterpolatorPtr ReverseKinematicControlSpace::Simulate(const State& x1, const ControlInput& u) { return make_shared<CSpaceInterpolator>(space->base.get(),x1,u); }
void ReverseKinematicControlSpace::Successor(const State& x1, const ControlInput& u,State& x0) { x0 = u;}
Math::VectorFieldFunction* ReverseKinematicControlSpace::SuccessorNumeric() { return NULL; }


KinematicControlSpace::KinematicControlSpace(const std::shared_ptr<CSpace>& _base,Real _maxNeighborhoodRadius)
:base(_base),maxNeighborhoodRadius(_maxNeighborhoodRadius)
{
  mySteeringFunction = make_shared<KinematicSteeringFunction>(_base);
  reverseControlSpace = make_shared<ReverseKinematicControlSpace>(this);
}

std::string KinematicControlSpace::VariableName(int i) 
{
  return base->VariableName(i)+"-target";
}

std::shared_ptr<CSet> KinematicControlSpace::GetControlSet(const Config& x)
{
  return std::shared_ptr<CSet>(new KinematicControlSet(base.get(),x,maxNeighborhoodRadius));
}

InterpolatorPtr KinematicControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  return make_shared<CSpaceInterpolator>(base.get(),x0,u);
}

void KinematicControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  x1 = u;
}

Math::VectorFieldFunction* KinematicControlSpace::SuccessorNumeric() { return NULL; }

bool KinematicControlSpace::ReverseControl(const State& x0,const State& x1,ControlInput& u) { u = x0; return true; }


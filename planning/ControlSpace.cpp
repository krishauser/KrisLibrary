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
  MultiControlSpace(const std::vector<int>& istateStarts,const std::vector<SmartPointer<ControlSpace> >& spaces);
  MultiControlSpace(const MultiCSpace* space,const std::vector<SmartPointer<ControlSpace> >& spaces);
  void Add(int istatemin,int istatemax,const SmartPointer<ControlSpace>& item);
  void Add(CSpace* space,const SmartPointer<ControlSpace>& item);
  virtual Interpolator* Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  std::vector<pair<int,int>  istateRanges;
  std::vector<SmartPointer<ControlSpace> > components;
};
*/

IntegratedControlSet::IntegratedControlSet(const SmartPointer<CSet>& _base,Real _dtmax)
:timeSelection(Biased),base(_base),dtmax(_dtmax)
{}

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



IntegratedControlSpace::IntegratedControlSpace(const SmartPointer<CSet>& _controlSet,Real _dt,Real _dtmax)
:myDynamics(NULL),type(Euler),space(NULL),controlSet(_controlSet),dt(_dt),dtmax(_dtmax)
{
  myControlSet = new IntegratedControlSet(controlSet,dtmax);
}

IntegratedControlSpace::IntegratedControlSpace(DynamicsFn f,const SmartPointer<CSet>& _controlSet,Real _dt,Real _dtmax)
:myDynamics(f),type(Euler),space(NULL),controlSet(_controlSet),dt(_dt),dtmax(_dtmax)
{
  myControlSet = new IntegratedControlSet(controlSet,dtmax);
}

std::string IntegratedControlSpace::VariableName(int i)
{
  if(i==0) return "time_step";
  else return ControlSpace::VariableName(i-1);
}

void IntegratedControlSpace::SetBaseControlSet(SmartPointer<CSet> baseControlSet)
{
  IntegratedControlSet* iset = dynamic_cast<IntegratedControlSet*>(&*myControlSet);
  iset->base = baseControlSet;
}

SmartPointer<CSet> IntegratedControlSpace::GetBaseControlSet()
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


Interpolator* IntegratedControlSpace::Simulate(const State& x0, const ControlInput& u)
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
  return new PiecewiseLinearInterpolator(p);
}

SmartPointer<CSet> IntegratedControlSpace::GetControlSet(const Config& x)
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
    fprintf(stderr,"IntegratedControlSpace: no dynamics function provided\n");
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
  SmartPointer<CSpace> space;
  KinematicSteeringFunction(const SmartPointer<CSpace>& _space) :space(_space) {}
  virtual bool Connect(const Config& x,const Config& y,KinodynamicMilestonePath& path) {
    path = KinodynamicMilestonePath(y,new CSpaceInterpolator(space,x,y));
    return true; 
  }
};


class ReverseKinematicControlSpace : public ControlSpace
{
public:
  ReverseKinematicControlSpace(KinematicControlSpace* space);
  virtual SmartPointer<CSet> GetControlSet(const Config& x);
  virtual Interpolator* Simulate(const State& x1, const ControlInput& u);
  virtual void Successor(const State& x1, const ControlInput& u,State& x0);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  KinematicControlSpace* space;
};

ReverseKinematicControlSpace::ReverseKinematicControlSpace(KinematicControlSpace* _space)
:space(_space)
{}
SmartPointer<CSet> ReverseKinematicControlSpace::GetControlSet(const Config& x)
{ return SmartPointer<CSet>(new KinematicControlSet(space->base,x,space->maxNeighborhoodRadius));}
Interpolator* ReverseKinematicControlSpace::Simulate(const State& x1, const ControlInput& u) { return new CSpaceInterpolator(space->base,x1,u); }
void ReverseKinematicControlSpace::Successor(const State& x1, const ControlInput& u,State& x0) { x0 = u;}
Math::VectorFieldFunction* ReverseKinematicControlSpace::SuccessorNumeric() { return NULL; }


KinematicControlSpace::KinematicControlSpace(const SmartPointer<CSpace>& _base,Real _maxNeighborhoodRadius)
:base(_base),maxNeighborhoodRadius(_maxNeighborhoodRadius)
{
  mySteeringFunction = new KinematicSteeringFunction(_base);
  reverseControlSpace = new ReverseKinematicControlSpace(this);
}

std::string KinematicControlSpace::VariableName(int i) 
{
  return base->VariableName(i)+"-target";
}

SmartPointer<CSet> KinematicControlSpace::GetControlSet(const Config& x)
{
  return SmartPointer<CSet>(new KinematicControlSet(base,x,maxNeighborhoodRadius));
}

Interpolator* KinematicControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  return new CSpaceInterpolator(base,x0,u);
}

void KinematicControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  x1 = u;
}

Math::VectorFieldFunction* KinematicControlSpace::SuccessorNumeric() { return NULL; }

bool KinematicControlSpace::ReverseControl(const State& x0,const State& x1,ControlInput& u) { u = x0; return true; }


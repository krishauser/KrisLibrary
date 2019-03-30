#include <KrisLibrary/Logger.h>
#include "DoubleIntegrator.h"
#include "InterpolatorHelpers.h"
#include "ParabolicRamp.h"
#include "KinodynamicPath.h"
#include "CSetHelpers.h"
#include <iostream>
#include <sstream>
using namespace std;


void Convert(const ParabolicRamp::ParabolicRamp1D& ramp,Spline::PiecewisePolynomial& res)
{
  res.segments.resize(3);
  res.timeShift.resize(3);
  res.times.resize(4);
  res.times[0] = 0;
  res.times[1] = ramp.tswitch1;
  res.times[2] = ramp.tswitch2;
  res.times[3] = ramp.ttotal;
  res.segments[0].Resize(3);
  res.segments[0].coef[0] = ramp.x0;
  res.segments[0].coef[1] = ramp.dx0;
  res.segments[0].coef[2] = 0.5*ramp.a1;
  res.timeShift[0] = 0;
  res.segments[1].Resize(2);
  res.segments[1].coef[0] = ramp.Evaluate(ramp.tswitch1);
  res.segments[1].coef[1] = ramp.Derivative(ramp.tswitch1);
  res.timeShift[1] = ramp.tswitch1;
  res.segments[2].Resize(3);
  res.segments[2].coef[0] = ramp.x1;
  res.segments[2].coef[1] = ramp.dx1;
  res.segments[2].coef[2] = 0.5*ramp.a2;
  res.timeShift[2] = ramp.ttotal;

  if(ramp.ttotal == ramp.tswitch2) {
    res.times.erase(--res.times.end());
    res.segments.erase(--res.segments.end());
    res.timeShift.erase(--res.timeShift.end());
  }
  if(ramp.tswitch1 == ramp.tswitch2) {
    res.times.erase(++res.times.begin());
    res.segments.erase(++res.segments.begin());
    res.timeShift.erase(++res.timeShift.begin());
  }
  if(ramp.tswitch1 == 0 && res.segments.size()>1) {
    res.times.erase(res.times.begin());
    res.segments.erase(res.segments.begin());
    res.timeShift.erase(res.timeShift.begin());
  }
}

void Convert(const ParabolicRamp::ParabolicRampND& ramp,Spline::PiecewisePolynomialND& res)
{
  res.elements.resize(ramp.ramps.size());
  for(size_t i=0;i<ramp.ramps.size();i++)
    Convert(ramp.ramps[i],res.elements[i]);
}

//concatenates the ramps
void Convert(const std::vector<ParabolicRamp::ParabolicRamp1D>& ramps,Spline::PiecewisePolynomial& res)
{
  assert(!ramps.empty());
  Convert(ramps[0],res);
  for(size_t i=1;i<ramps.size();i++) {
    Spline::PiecewisePolynomial suffix;
    Convert(ramps[i],suffix);
    res.Concat(suffix,true);
  }
}

void Convert(const std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> >& ramps,Spline::PiecewisePolynomialND& res)
{
  res.elements.resize(ramps.size());
  for(size_t i=0;i<ramps.size();i++)
    Convert(ramps[i],res.elements[i]);
}





DoubleIntegratorControlSpace::DoubleIntegratorControlSpace(const std::shared_ptr<CSet>& uset,Real dtmax)
:IntegratedControlSpace(uset,dtmax,dtmax)
{}

InterpolatorPtr DoubleIntegratorControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  //do we want to have the interpolator in the range [0,1]?
  Vector q0,v0;
  CVSpace::GetState(x0,q0,v0);
  Real udt = u(0);
  ControlInput ubase;
  ubase.setRef(u,1,1,u.n-1);
  vector<Spline::Polynomial<double> > elements(x0.n);
  assert(q0.n == ubase.n);
  for(int i=0;i<q0.n;i++) {
    elements[i].Resize(3);
    elements[i].SetCoef(0,q0[i]);
    elements[i].SetCoef(1,v0[i]);
    elements[i].SetCoef(2,0.5*ubase[i]);
  }
  for(int i=0;i<q0.n;i++) {
    elements[i+q0.n].Resize(2);
    elements[i+q0.n].SetCoef(0,v0[i]);
    elements[i+q0.n].SetCoef(1,ubase[i]);
  }
  Spline::PiecewisePolynomialND path(elements,0,udt);
  return make_shared<PiecewisePolynomialInterpolator>(path);
}

std::string DoubleIntegratorControlSpace::VariableName(int i)
{
  if(i==0) return "time_step";
  stringstream ss; ss<<"ddx"<<i-1; return ss.str(); 
}



void DoubleIntegratorControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  Real udt = u(0);
  ControlInput ubase;
  ubase.setRef(u,1,1,u.n-1);
  EvalDynamics(x0,ubase,udt,x1);
}

void DoubleIntegratorControlSpace::Derivative(const State& x, const ControlInput& u,State& dx)
{
  Vector q,v;
  CVSpace::GetState(x,q,v);
  CVSpace::SetState(v,u,dx);
}

void DoubleIntegratorControlSpace::EvalDynamics(const State& x0, const ControlInput& ddx,Real t,State& x1)
{
  Vector q0,v0,q1,v1;
  CVSpace::GetState(x0,q0,v0);
  q1 = q0 + t*v0 + (0.5*t*t)*ddx;
  v1 = v0 + t*ddx;
  CVSpace::SetState(q1,v1,x1);
}





DoubleIntegratorKinodynamicSpace::DoubleIntegratorKinodynamicSpace(std::shared_ptr<CSpace> qspace,std::shared_ptr<CSpace> vspace,std::shared_ptr<CSet> uset,Real dtmax)
:KinodynamicSpace(make_shared<CVSpace>(qspace,vspace),make_shared<DoubleIntegratorControlSpace>(uset,dtmax)),visibilityEpsilon(1e-2)
{
  BoxSet* abset = dynamic_cast<BoxSet*>(uset.get());
  if(!abset) {
    LOG4CXX_INFO(KrisLibrary::logger(),"DoubleIntegratorKinodynamicSpace: No steering function available; accelerations are not box-bounded\n");
  }
  else {
    //create steering function
    PropertyMap props;
    vspace->Properties(props);
    vector<Real> vmin,vmax;
    if(!props.getArray("minimum",vmin) || !props.getArray("maximum",vmax)) {
      vmin.resize(abset->bmin.size(),-Inf);
      vmax.resize(abset->bmax.size(),Inf);
    }
    controlSpace->mySteeringFunction = make_shared<DoubleIntegratorBoxBoundedSteeringFunction>(abset->bmax,Vector(vmax));
  }
}

EdgePlannerPtr DoubleIntegratorKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const std::shared_ptr<Interpolator>& path)
{
  return make_shared<EpsilonEdgeChecker>(GetStateSpace().get(),path,visibilityEpsilon);
}

void DoubleIntegratorKinodynamicSpace::SetVisibilityEpsilon(Real tol)
{
  visibilityEpsilon = tol;
}




DoubleIntegratorBoxBoundedSteeringFunction::DoubleIntegratorBoxBoundedSteeringFunction(const Vector& _amax,const Vector& _vmax)
:amax(vector<double>(_amax)),vmax(vector<double>(_vmax))
{}

void DoubleIntegratorBoxBoundedSteeringFunction::SetConfigurationBounds(const Config& _qmin,const Config& _qmax)
{
  qmin = _qmin;
  qmax = _qmax;
}

bool DoubleIntegratorBoxBoundedSteeringFunction::Connect(const State& x,const State& y,KinodynamicMilestonePath& path)
{
  Vector qx,vx,qy,vy;
  CVSpace::GetState(x,qx,vx);
  CVSpace::GetState(y,qy,vy);
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0 = qx;
  ramp.x1 = qy;
  ramp.dx0 = vx;
  ramp.dx1 = vy;
  if(!ramp.SolveMinTime(vector<double>(amax),vector<double>(vmax))) return false;
  path.milestones.resize(2);
  path.milestones[0] = x;
  path.milestones[1] = y;
  //NOTE: THE CONTROL IS INVALID.  SHOULD WE SPLIT UP THE PATH INTO PIECEWISE QUADRATIC SEGMENTS?
  path.controls.resize(1);
  path.controls[0].resize(qx.n+1,0.0);
  path.controls[0][0] = ramp.endTime;
  Spline::PiecewisePolynomialND ppath;
  Convert(ramp,ppath);
  Spline::PiecewisePolynomialND dpath = ppath.Differentiate();
  //the polynomial needs derivative information too...
  ppath.elements.insert(ppath.elements.end(),dpath.elements.begin(),dpath.elements.end());
  path.paths.push_back(make_shared<PiecewisePolynomialInterpolator>(ppath));
  return true;
}
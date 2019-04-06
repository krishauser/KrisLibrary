#include <KrisLibrary/Logger.h>
#include "Objective.h"
#include "KinodynamicPath.h"
#include "Path.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <string.h>
using namespace std;

struct ErrorAccumulator
{
  /// Sets an L-p norm
  ErrorAccumulator(Real norm=1.0);
  /// Can accept L1, L2, Linf, MAE (mean absolute error)
  /// MSE (mean squared error), RMSE (root mean squared error)
  ErrorAccumulator(const char* type);
  void Add(Real error);
  void Add(Real error,Real weight);
  Real Value() const;

  Real norm;
  bool mean,root;
  Real accumulator;
  Real sumWeights;
};

ErrorAccumulator::ErrorAccumulator(Real _norm)
  :norm(_norm),mean(false),root(true),accumulator(0.0),sumWeights(0.0)
{}

ErrorAccumulator::ErrorAccumulator(const char* type)
  :accumulator(0.0),sumWeights(0.0)
{
  if(0==strcmp(type,"L1")) {
    norm=1.0;
    mean=false;
    root=false;
  }
  else if(0==strcmp(type,"L2")) {
    norm=2.0;
    mean=false;
    root=true;
  }
  else if(0==strcmp(type,"Linf")) {
    norm=Inf;
    mean=false;
    root=false;
  }
  else if(0==strcmp(type,"RMSE")) {
    norm=2.0;
    mean=true;
    root=true;
  }
  else if(0==strcmp(type,"MSE")) {
    norm=2.0;
    mean=true;
    root=false;
  }
  else if(0==strcmp(type,"MAE")) {
    norm=1.0;
    mean=true;
    root=false;
  }
}

void ErrorAccumulator::Add(Real error)
{
  if(norm == 1.0)
    accumulator += Abs(error);
  else if(norm == 2.0)
    accumulator += error*error;
  else if(IsInf(norm))
    accumulator = Max(accumulator,Abs(error));
  else
    accumulator += Pow(Abs(error),norm);
  sumWeights += 1.0;
}

void ErrorAccumulator::Add(Real error,Real weight)
{
  if(norm == 1.0)
    accumulator += Abs(error)*weight;
  else if(norm == 2.0)
    accumulator += error*error*weight;
  else if(IsInf(norm))
    accumulator = Max(accumulator,weight*Abs(error));
  else
    accumulator += weight*Pow(Abs(error),norm);
  sumWeights += weight;
}

Real ErrorAccumulator::Value() const
{
  if(IsInf(norm)) return accumulator;
  if(norm == 1.0) {
    if(mean) return accumulator/sumWeights;
    return accumulator;
  }
  if(norm == 2.0) {
    if(mean && !root) return accumulator/sumWeights;
    else if(mean && root) return Sqrt(accumulator/sumWeights);
    else if(root) return Sqrt(accumulator);
    return accumulator;
  }
  if(mean && !root) return accumulator/sumWeights;
  else if(mean && root) return Pow(accumulator/sumWeights,1.0/norm);
  else if(root) return Pow(accumulator,1.0/norm);
  return accumulator;
}


Real ObjectiveFunctionalBase::IncrementalCost(const KinodynamicMilestonePath& path)
{
  Real c = 0.0;
  for(size_t i=0;i<path.paths.size();i++) {
    c += IncrementalCost(path.controls[i],path.paths[i].get());
  }
  return c;
}

Real ObjectiveFunctionalBase::PathCost(const MilestonePath& path)
{
  if(path.edges.size()==0) {
    FatalError("ObjectiveFunctionalBase::PathCost: Asking for cost of an empty path?");
  }
  if(PathInvariant()) return TerminalCost(path.End());
  Real c = TerminalCost(path.End());
  for(size_t i=0;i<path.edges.size();i++)
    c += IncrementalCost(path.edges[i].get());
  return c;
}

Real ObjectiveFunctionalBase::PathCost(const KinodynamicMilestonePath& path)
{
  if(PathInvariant()) return TerminalCost(path.End());
  return IncrementalCost(path) + TerminalCost(path.End());
}

IntegratorObjectiveFunctional::IntegratorObjectiveFunctional(Real _dt,int _timeIndex)
:dt(_dt),timeIndex(_timeIndex)
{}

Real IntegratorObjectiveFunctional::Domain(const ControlInput& u,const Interpolator* path)
{
  if(timeIndex < 0) FatalError("IntegratorObjectiveFunctional::Domain needs to be overridden, or time index specified");
  return path->End()[timeIndex] - path->Start()[timeIndex];
}

//Constants for Gaussian quadrature with n=5
const static Real g5points [5] = {0.0,Sqrt(5.0-2*Sqrt(10.0/7.0))/3.0,-Sqrt(5.0-2*Sqrt(10.0/7.0))/3.0,Sqrt(5.0+2*Sqrt(10.0/7.0))/3.0,-Sqrt(5.0+2*Sqrt(10.0/7.0))/3.0};
const static Real g5weights [5] = {128.0/225.0,(322.0+13*Sqrt(70.0))/900,(322.0+13*Sqrt(70.0))/900,(322.0-13*Sqrt(70.0))/900,(322.0-13*Sqrt(70.0))/900};


Real IntegratorObjectiveFunctional::IncrementalCost(const ControlInput& u,const Interpolator* path)
{
  Real integral = 0.0;
  Real length = Domain(u,path);
  int numSegs = int(Ceil(length/dt));
  Config qt;
  for(int s=0;s<numSegs;s++) {
    Real u1 = Real(s)/numSegs;
    Real u2 = Real(s+1)/numSegs;
    Real sum=0.0;
    for(int i=0;i<5;i++) {
      Real t = (u1+u2)*0.5 + g5points[i]*(u2-u1)*0.5;
      path->Eval(t,qt);
      sum += g5weights[i] * DifferentialCost(qt,u);
    }
    integral += sum*(u2-u1)*0.5;
  }
  return integral;
}


TimeObjective::TimeObjective(int _timeIndex)
:timeIndex(_timeIndex)
{}

Real TimeObjective::IncrementalCost(const Interpolator* path)
{
  return path->End()[timeIndex] - path->Start()[timeIndex];
}

ConfigObjective::ConfigObjective(const Config& _qgoal,CSpace* _cspace)
    : qgoal(_qgoal),cspace(_cspace)
{
}

ConfigObjective::ConfigObjective(const Config& _qgoal,const Vector& _weights)
:qgoal(_qgoal),weights(_weights),cspace(NULL)
{}

Real ConfigObjective::TerminalCost(const Vector& qend)
{
  if(cspace) return cspace->Distance(qend,qgoal);
  else if(weights.empty()) return qgoal.distance(qend);
  else {
    Real s = 0;
    for(int i=0;i<qend.n;i++)
      s += weights[i]*Sqr(qend[i]-qgoal[i]);
    return Sqrt(s);
  }
}


QuadraticObjective::QuadraticObjective(int _timeIndex)
:IntegratorObjectiveFunctional(0.1,_timeIndex)
{}

Real QuadraticObjective::TerminalCost(const Vector& qend)
{
  Real t = qend[timeIndex];
  Vector diff = desiredPath->End()-qend;
  Vector temp;
  terminalCostMatrix.mul(diff,temp);
  return diff.dot(temp);
}

Real QuadraticObjective::DifferentialCost(const State& x,const ControlInput& u)
{
  Real t = x[timeIndex];
  Vector xdes;
  desiredPath->Eval(t,xdes);
  xdes -= x;
  Vector temp,temp2;
  stateCostMatrix.mul(xdes,temp);
  controlCostMatrix.mul(u,temp2);
  return xdes.dot(temp)+u.dot(temp2);
}


CompositeObjective::CompositeObjective()
: norm(1.0)
{}

CompositeObjective::~CompositeObjective()
{
}

void CompositeObjective::Add(const std::shared_ptr<ObjectiveFunctionalBase>& obj,Real weight)
{
  components.push_back(obj);
  if(!weights.empty())
    weights.push_back(weight);
  else if(weight!=1.0) {
    weights.resize(components.size(),1.0);
    weights.back()=weight;
  }
}

string CompositeObjective::Description()
{ 
  string desc = TypeString();
  desc += "(";
  for(size_t i=0;components.size();i++) {
    if(i > 0) desc += ",";
    desc += components[i]->Description();
    if(!weights.empty() && weights[i] != 1.0) {
      char buf[64];
      snprintf(buf,64,"*%g\n",weights[i]);
      desc += buf;
    }
  }
  desc += ")";
  return desc;
}

Real CompositeObjective::TerminalCost(const Vector& qend) 
{ 
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->TerminalCost(qend);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}

Real CompositeObjective::IncrementalCost(const Interpolator* path)
{
  //TODO: this isn't exactly the same as the sum of differential costs except
  //for norm = 1
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->IncrementalCost(path);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}

bool CompositeObjective::PathInvariant() const
{
  for(size_t i=0;i<components.size();i++)
    if(!components[i]->PathInvariant()) return false;
  return true;
}






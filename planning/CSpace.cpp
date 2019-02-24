#include <KrisLibrary/Logger.h>
#include "CSpace.h"
#include "CSpaceHelpers.h"
#include "EdgePlanner.h"
#include "EdgePlannerHelpers.h"
#include <math/random.h>
#include <optimization/NonlinearProgram.h>
#include <math/vectorfunction.h>
#include <sstream>
using namespace std;

CSet::CSet()
:test()
{}

#if __cplusplus > 199711L  || _MSC_VER >= 1900
  CSet::CSet(PREDICATE_FUNCTION_PTR f)
  :test(ptr_fun(f))
  {}
#endif //C++11

CSet::CSet(CPredicate _test)
:test(_test)
{}

bool CSet::Contains(const Config& x)
{
  return test(x);
}


void CSpace::AddConstraint(const std::string& name,CSet::CPredicate test)
{
  AddConstraint(name,new CSet(test));
}

void CSpace::AddConstraint(const std::string& name,CSet* constraint)
{
  constraints.push_back(std::shared_ptr<CSet>(constraint));
  constraintNames.push_back(name);
}

void CSpace::AddConstraint(const std::string& name,const std::shared_ptr<CSet>& constraint)
{
  constraints.push_back(constraint);
  constraintNames.push_back(name);
}

void CSpace::CopyConstraints(const CSpace* space,const std::string& prefix)
{
  constraints = space->constraints;
  constraintNames.resize(space->constraintNames.size());
  for(size_t i=0;i<constraintNames.size();i++)
    constraintNames[i] = prefix + space->constraintNames[i];
}

std::string CSpace::VariableName(int i)
{
  stringstream ss;
  ss<<"x"<<i;
  return ss.str();
}

bool CSpace::IsFeasible(const Config& q)
{
  for(size_t i=0;i<constraints.size();i++) {
    if(!IsFeasible(q,i)) return false;
  }
  return true;
}

bool CSpace::IsFeasible(const Config& q,int constraint)
{
  return constraints[constraint]->Contains(q);
}

EdgePlannerPtr CSpace::PathChecker(const Config& a,const Config& b)
{
  for(size_t i=0;i<constraints.size();i++)
    if(!constraints[i]->IsConvex())
      FatalError("Cannot instantiate PathChecker, your CSpace subclass needs to override this method\n");
  return make_shared<EndpointEdgeChecker>(this,a,b);
}

EdgePlannerPtr CSpace::PathChecker(const Config& a,const Config& b,int constraint)
{
  if(constraints[constraint]->IsConvex()) return make_shared<EndpointEdgeChecker>(this,a,b);
  FatalError("Cannot instantiate PathChecker for obstacle %d, your CSpace subclass needs to override this method\n");
  return NULL;
}



int CSpace::NumDimensions()
{
  Config q;
  Sample(q);
  return q.n;
}

void CSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(c.n);
  for(int i=0;i<c.n;i++)
    x(i) = c(i) + Rand(-r,r);
}

void CSpace::Interpolate(const Config& x, const Config& y, Real u, Config& out)
{
  out.mul(x,One-u);
  out.madd(y,u);
}

void CSpace::Midpoint(const Config& x, const Config& y, Config& out)
{
  Interpolate(x,y,0.5,out);
  //out.add(x,y);
  //out.inplaceMul(Half);
}

EdgePlannerPtr CSpace::LocalPlanner(const Config& a,const Config& b)
{
  return PathChecker(a,b);
}


void CSpace::Properties(PropertyMap& map)
{
  map.set("cartesian",1);
  map.set("geodesic",1);
  map.set("metric","euclidean");
  bool convex = true;
  for(size_t i=0;i<constraints.size();i++)
    if(!constraints[i]->IsConvex())
      convex = false;
  if(convex)
    map.set("convex",1);
}


bool CSpace::ProjectFeasible(Config& x)
{
  for(size_t i=0;i<constraints.size();i++)
    if(!constraints[i]->Project(x)) return false;
  for(size_t i=0;i+1<constraints.size();i++)
    if(!constraints[i]->Contains(x)) return false;
  return true;
}

class NegativeVectorFieldFunction : public VectorFieldFunction
{
public:
  std::shared_ptr<VectorFieldFunction> base;
  NegativeVectorFieldFunction(const std::shared_ptr<VectorFieldFunction>& _base) : base(_base) {}
  virtual std::string Label() const { return "-"+base->Label(); }
  virtual int NumDimensions() const { return base->NumDimensions(); }
  virtual void PreEval(const Vector& x) { base->PreEval(x); }
  virtual void Eval(const Vector& x,Vector& v) { base->Eval(x,v); v.inplaceNegative();  }
  virtual Real Eval_i(const Vector& x,int i) { return -base->Eval_i(x,i); }
  virtual Real Jacobian_ij(const Vector& x,int i,int j) { return -base->Jacobian_ij(x,i,j); }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji) { base->Jacobian_i(x,i,Ji); Ji.inplaceNegative(); }
  virtual void Jacobian_j(const Vector& x,int j,Vector& Jj) { base->Jacobian_j(x,j,Jj); Jj.inplaceNegative();}
  virtual void Jacobian(const Vector& x,Matrix& J) { base->Jacobian(x,J); J.inplaceNegative(); }
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v) { base->DirectionalDeriv(x,h,v); v.inplaceNegative(); }
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) { base->Hessian_i(x,i,Hi); Hi.inplaceNegative(); }
  virtual Real Hessian_ijk(const Vector& x,int i,int j,int k) { return -base->Hessian_ijk(x,i,j,k); }
};

Optimization::NonlinearProgram* Join(const vector<std::shared_ptr<Optimization::NonlinearProgram> >& nlps)
{
  std::shared_ptr<CompositeVectorFieldFunction> c(new CompositeVectorFieldFunction);
  std::shared_ptr<CompositeVectorFieldFunction> d(new CompositeVectorFieldFunction);
  for(size_t i=0;i<nlps.size();i++) {
        if(nlps[i]->f != NULL) LOG4CXX_ERROR(KrisLibrary::logger(),"Join(NonlinearProgram): Warning, NLP "<<(int)i);
    if(nlps[i]->c != NULL) c->functions.push_back(nlps[i]->c);
    if(nlps[i]->d != NULL) {
      if(!nlps[i]->inequalityLess)
        d->functions.push_back(std::shared_ptr<VectorFieldFunction>(new NegativeVectorFieldFunction(nlps[i]->d)));
      else
        d->functions.push_back(nlps[i]->d);
    }
  }
  if(c->functions.empty()) c=NULL;
  if(d->functions.empty()) d=NULL;
  Optimization::NonlinearProgram* res = new Optimization::NonlinearProgram(NULL,c,d);
  res->inequalityLess = true;
  return res;
}

Optimization::NonlinearProgram* CSpace::FeasibleNumeric()
{
  vector<std::shared_ptr<Optimization::NonlinearProgram> > nlps(constraints.size());
  for(size_t i=0;i<constraints.size();i++) {
    nlps[i].reset(constraints[i]->Numeric());
    if(!nlps[i]) 
      return NULL;
  }
  return Join(nlps);
}

Real CSpace::ObstacleDistance(const Config& a)
{
  Real dmin = Inf;
  for(size_t i=0;i<constraints.size();i++) {
    dmin = Min(dmin,constraints[i]->ObstacleDistance(a));
    if(IsInf(dmin) < 0) return dmin;
  }
  return dmin;
}

void CSpace::CheckConstraints(const Config& q,std::vector<bool>& satisfied)
{
  satisfied.resize(constraints.size());
  for(size_t i=0;i<constraints.size();i++)
    satisfied[i] = constraints[i]->Contains(q);
}

void CSpace::GetFeasibleNames(const Config& q,vector<std::string>& names)
{
  names.resize(0);
  vector<bool> feas;
  CheckConstraints(q,feas);
  for(size_t i=0;i<constraints.size();i++) {
    if(feas[i])
      names.push_back(constraintNames[i]);
  }
}

void CSpace::GetInfeasibleNames(const Config& q,vector<std::string>& names)
{
  names.resize(0);
  vector<bool> feas;
  CheckConstraints(q,feas);
  for(size_t i=0;i<constraints.size();i++) {
    if(!feas[i])
      names.push_back(constraintNames[i]);
  }
}

void CSpace::PrintInfeasibleNames(const Config& q,std::ostream& out,const char* prefix,const char* suffix)
{
  vector<bool> feas;
  CheckConstraints(q,feas);
  for(size_t i=0;i<feas.size();i++) {
    if(!feas[i])
      out<<prefix<<constraintNames[i]<<suffix;
  }
}


#include <KrisLibrary/Logger.h>
#include "KinodynamicOptimizer.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/function.h>
#include <KrisLibrary/math/vectorfunction.h>
#include <KrisLibrary/math/differentiation.h>
#include <KrisLibrary/math/stacking.h>
#include <KrisLibrary/math/LDL.h>
#include <KrisLibrary/Timer.h>
using namespace std;

KinodynamicLocalOptimizer::KinodynamicLocalOptimizer(KinodynamicSpace* s,std::shared_ptr<ObjectiveFunctionalBase> _objective)
:KinodynamicPlannerBase(s),objective(_objective),bestPathCost(Inf)
{
  const static int numMethods = 4;
  methodAvailable.resize(numMethods,false);
  methodCounts.resize(numMethods,1);
  methodCosts.resize(numMethods,0.0);
  methodRewards.resize(numMethods,1.0);
  std::shared_ptr<SteeringFunction> sf = s->GetControlSpace()->GetSteeringFunction();
  methodAvailable[Shortcut] = (sf != NULL && sf->IsExact());
  methodAvailable[RandomDescent] = false;
  methodAvailable[GradientDescent] = false;
  methodAvailable[DDP] = false;
}

KinodynamicLocalOptimizer::~KinodynamicLocalOptimizer()
{
}

void KinodynamicLocalOptimizer::Init(const KinodynamicMilestonePath& path,CSet* _goalSet)
{
  bestPath = path;
  if(bestPath.edges.empty()) bestPath.MakeEdges(space);
  ComputeCosts();
  goalSet = _goalSet;
  LOG4CXX_INFO(KrisLibrary::logger(),"Initial cost "<<bestPathCost);
}

void KinodynamicLocalOptimizer::Init(const State& xinit,CSet* goalSet)
{
  FatalError("KinodynamicLocalOptimizer::Init: must be provided with initial path");
}

bool KinodynamicLocalOptimizer::Plan(int maxIters)
{
  bool changed = false;
  Real oldPathCost = bestPathCost;
  for(int iters=0;iters<maxIters;iters++) {
    //update optimal order
    int bestMethod = -1;
    Real bestRate = Inf;
    for(size_t i=0;i<methodAvailable.size();i++) {
      if(!methodAvailable[i]) continue;
      Real rate = methodCosts[i] / methodRewards[i];
      LOG4CXX_INFO(KrisLibrary::logger(),"Method "<<i<<" cost "<<methodCosts[i]<<" reward "<<methodRewards[i]<<", rate "<<rate);
      if(rate < bestRate) {
        bestRate = rate;
        bestMethod = (int)i;
      }
    }
    if(bestMethod < 0) return false;
    Timer timer;
    switch(bestMethod) {
    case Shortcut:
      if(DoShortcut()) 
        changed = true;
      break;
    case RandomDescent:
      if(DoRandomDescent(0.01)) 
        changed = true;
      break;
    case GradientDescent:
      if(DoGradientDescent()) 
        changed = true;
      break;
    case DDP:
      if(DoDDP()) 
        changed = true;
      break;
    }
    Real t = timer.ElapsedTime();
    Real delta = (changed ? oldPathCost - bestPathCost : 0);
        if(delta < 0) LOG4CXX_ERROR(KrisLibrary::logger(),"Method "<<bestMethod<<" got worse solution than before? "<<oldPathCost<<" -> "<<bestPathCost);
    oldPathCost = bestPathCost;
    int n = methodCounts[bestMethod] + 1;
    methodCosts[bestMethod] = methodCosts[bestMethod] + 1.0/Real(n)*(t-methodCosts[bestMethod]);
    methodRewards[bestMethod] = methodRewards[bestMethod] + 1.0/Real(n)*(delta-methodRewards[bestMethod]);
    methodCounts[bestMethod] = n;
  }
  return changed;
}

void KinodynamicLocalOptimizer::ComputeCosts()
{
  cumulativeCosts.resize(bestPath.milestones.size());
  cumulativeCosts[0] = 0;
  for(size_t i=0;i<bestPath.paths.size();i++)
    cumulativeCosts[i+1] = cumulativeCosts[i] + objective->IncrementalCost(bestPath.controls[i],bestPath.paths[i].get());
  bestPathCost = cumulativeCosts.back()+objective->TerminalCost(bestPath.milestones.back());
}

bool KinodynamicLocalOptimizer::DoShortcut() 
{
  std::shared_ptr<SteeringFunction> sf = space->GetControlSpace()->GetSteeringFunction();
  if(!sf) return false;
  Real u1 = Rand()*bestPath.ParamEnd();
  Real u2 = Rand()*bestPath.ParamEnd();
  if(u2 < u1) Swap(u1,u2);
  Config x1,x2;
  int e1 = bestPath.Eval2(u1,x1);
  int e2 = bestPath.Eval2(u2,x2);
  if(e1 == e2) return false;
  KinodynamicMilestonePath shortcut;
  if(!sf->Connect(x1,x2,shortcut)) return false;
  //evaluate new cost
  Real cshortcut = objective->IncrementalCost(shortcut);
  Real s1 = u1*(Real)bestPath.edges.size() - Floor(u1*(Real)bestPath.edges.size());
  Real s2 = u2*(Real)bestPath.edges.size() - Floor(u2*(Real)bestPath.edges.size());
  Real cOrig = (1-s1)*(cumulativeCosts[e1+1]-cumulativeCosts[e1]) + cumulativeCosts[e2] - cumulativeCosts[e1+1] + s2*(cumulativeCosts[e2+1]-cumulativeCosts[e2]);
  if(cshortcut < cOrig) {
    shortcut.MakeEdges(space);
    if(shortcut.IsFeasible()) {
      //splice into best path
      bestPath.Splice(u1,u2,shortcut,space);
      ComputeCosts();
      return true;
    }
  }
  return false;
}

bool KinodynamicLocalOptimizer::DoRandomDescent(Real perturbationSize)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"Trying random descent\n");
  tempPath.milestones.resize(bestPath.milestones.size());
  tempPath.milestones[0] = bestPath.milestones[0];
  tempPath.controls.resize(bestPath.controls.size());
  tempPath.paths.resize(bestPath.controls.size());
  std::shared_ptr<CSpace> stateSpace = space->GetStateSpace();
  int numSamples = Max(100,10*(int)tempPath.controls.size());
  int numControlSampleFailures = 0;
  int numGoalFailures = 0;
  int numFeasibleMilestoneFailures = 0;
  for(size_t i=0;i<tempPath.controls.size();i++) {
    Real r = perturbationSize*Sqr(Real(i+1)/Real(tempPath.controls.size()));
    tempPath.controls[i].resize(bestPath.controls[i].size());
    std::shared_ptr<CSet> uset = space->GetControlSet(tempPath.milestones[i]);
    while(numSamples > 0) {
      numSamples--;
      //perturb control
      for(int j=0;j<tempPath.controls[i].size();j++)
        tempPath.controls[i][j] = bestPath.controls[i][j] + Rand(-r,r);
      if(uset->Contains(tempPath.controls[i]) || uset->Project(tempPath.controls[i])) {
        Assert(uset->Contains(tempPath.controls[i]));
        //got a valid control, check the path
        tempPath.paths[i] = space->Simulate(tempPath.milestones[i],tempPath.controls[i]);
        tempPath.milestones[i+1] = tempPath.paths[i]->End();
        if(goalSet && i+1==tempPath.controls.size()) {
          //need to lie in goal set
          if(!goalSet->Contains(tempPath.milestones[i+1])) {
            numFeasibleMilestoneFailures ++;
            continue;
          }
        }
        if(!stateSpace->IsFeasible(tempPath.milestones[i+1])) {
          numGoalFailures ++;
          continue;
        }
        else
          //done!
          break;
      }
      else 
        numControlSampleFailures ++;
    }
    if(numSamples==0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Unable to sample a good path adjustment, "<< numControlSampleFailures<<" sample control, "<<numFeasibleMilestoneFailures<<" feasible milestone, "<<numGoalFailures);
      return false;
    }
  }
  if(objective->PathCost(tempPath) >= bestPathCost) return false;
  //check edge feasibility
  tempPath.edges.resize(0);
  tempPath.MakeEdges(space);
  if(!tempPath.IsFeasible()) return false;

  //it's feasible and improves on the cost, we're done!
  bestPath = tempPath;
  ComputeCosts();
  return true;
}

class SimulateFunction : public VectorFieldFunction
{
public:
  KinodynamicSpace* kspace;
  Config x;
  SimulateFunction(KinodynamicSpace*_kspace,const Config& _x)
  :kspace(_kspace),x(_x)
  {
  }
  virtual int NumDimensions() const { return x.n; }
  virtual void Eval(const ControlInput& u,Vector& res) {
    kspace->Successor(x,u,res);
  }
  virtual void Jacobian(const ControlInput& u,Matrix& J) {
    Vector temp=u;
    JacobianCenteredDifference(*this,temp,0.001,J);
  }
};

class SimulateFunctionX : public VectorFieldFunction
{
public:
  KinodynamicSpace* kspace;
  ControlInput u;
  SimulateFunctionX(KinodynamicSpace*_kspace,const ControlInput& _u)
  :kspace(_kspace),u(_u)
  {
  }
  virtual int NumDimensions() const { return kspace->GetStateSpace()->NumDimensions(); }
  virtual void Eval(const State& x,Vector& res) {
    kspace->Successor(x,u,res);
  }
  virtual void Jacobian(const State& x,Matrix& J) {
    Vector temp=x;
    JacobianCenteredDifference(*this,temp,0.001,J);
  }
};

void AllHessiansCenteredDifferences(VectorFieldFunction& f,const Vector& x,Real h,vector<Matrix>& Hs)
{
  Vector temp=x;
  Matrix J1,J2;
  Hs.resize(f.NumDimensions());
  for(size_t i=0;i<Hs.size();i++)
    Hs[i].resize(x.n,x.n);
  Real scale = 0.5/h;
  for(int i=0;i<x.n;i++) {
    temp[i] += h;
    f.PreEval(temp);
    f.Jacobian(temp,J2);
    temp[i] -= 2*h;
    f.PreEval(temp);
    f.Jacobian(temp,J1);
    temp[i] = x[i];

    //start filling out row i of hessians
    for(int j=0;j<J1.n;j++) {
      Vector r1,r2,hi;
      J1.getRowRef(j,r1);
      J2.getRowRef(j,r2);
      Hs[j].getRowRef(i,hi);
      hi.sub(r2,r1);
      hi.inplaceMul(scale);
    }
  }
}

class IncrementalCostFunction : public ScalarFieldFunction
{
public:
  KinodynamicSpace* kspace;
  ObjectiveFunctionalBase* obj;
  Config x;
  IncrementalCostFunction(KinodynamicSpace*_kspace,ObjectiveFunctionalBase* _obj,const State& _x)
  :kspace(_kspace),obj(_obj),x(_x)
  {
  }
  virtual Real Eval(const ControlInput& u) {
    InterpolatorPtr i = kspace->Simulate(x,u);
    return obj->IncrementalCost(u,i.get());
  }
  virtual void Gradient(const ControlInput& u,Vector& grad) {
    Vector temp=u;
    GradientCenteredDifference(*this,temp,0.001,grad);
  }
  virtual void Hessian(const State& u,Matrix& H) {
    Vector temp=u;
    HessianCenteredDifference(*this,temp,0.001,H);
  }
};

class IncrementalCostFunctionX : public ScalarFieldFunction
{
public:
  KinodynamicSpace* kspace;
  ObjectiveFunctionalBase* obj;
  ControlInput u;
  IncrementalCostFunctionX(KinodynamicSpace*_kspace,ObjectiveFunctionalBase* _obj,const ControlInput& _u)
  :kspace(_kspace),obj(_obj),u(_u)
  {
  }
  virtual Real Eval(const State& x) {
    InterpolatorPtr i = kspace->Simulate(x,u);
    return obj->IncrementalCost(u,i.get());
  }
  virtual void Gradient(const State& x,Vector& grad) {
    Vector temp=x;
    GradientCenteredDifference(*this,temp,0.001,grad);
  }
  virtual void Hessian(const State& x,Matrix& H) {
    Vector temp=x;
    HessianCenteredDifference(*this,temp,0.001,H);
  }
};

class TerminalCostFunction : public ScalarFieldFunction
{
public:
  ObjectiveFunctionalBase* obj;
  TerminalCostFunction(ObjectiveFunctionalBase* _obj)
  :obj(_obj)
  {
  }
  virtual Real Eval(const State& x) {
    return obj->TerminalCost(x);
  }
  virtual void Gradient(const State& x,Vector& grad) {
    Vector temp=x;
    GradientCenteredDifference(*this,temp,0.001,grad);
  }
  virtual void Hessian(const State& x,Matrix& H) {
    Vector temp=x;
    HessianCenteredDifference(*this,temp,0.001,H);
  }
};

//on the combined parameter vector (u0,...,un-1,x1,...,xn)
//Returns 1 if the path is feasible, 0 otherwise
class PathFeasibilityFunction : public ScalarFieldFunction
{
public:
  KinodynamicSpace* space;
  KinodynamicMilestonePath temp;
  CSet* goalSet;
  PathFeasibilityFunction(KinodynamicSpace* _space,const KinodynamicMilestonePath& path,CSet* _goalSet)
  :space(_space),temp(path),goalSet(_goalSet)
  {
    for(size_t i=1;i<temp.milestones.size();i++) {
      temp.milestones[i].clear();
      temp.milestones[i].setRef(path.milestones[i]);
    }
    for(size_t i=0;i<temp.controls.size();i++) {
      temp.controls[i].clear();
      temp.controls[i].setRef(path.controls[i]);
    }
  }
  virtual int NumDimensions() const {
    return (temp.milestones.size()-1)*temp.milestones[0].n;
  }
  virtual void PreEval(const Vector& u) {
    Assert(u.n == (int)temp.controls.size()*(temp.controls[0].n+temp.milestones[0].n));
    int nu=temp.controls[0].n;
    int nx=temp.milestones[0].n;
    int xoffset = nu*(int)temp.controls.size();
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.controls[i].setRef(u,i*nu,1,nu);  
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.milestones[i+1].setRef(u,xoffset+i*nx,1,nx);
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.paths[i] = space->Simulate(temp.milestones[i],temp.controls[i]);
  }
  virtual Real Eval(const Vector& u)
  {
    if(goalSet && !goalSet->Contains(temp.End())) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Left goal region\n");
      return 0.0;
    }
    for(size_t i=0;i<temp.controls.size();i++)
      if(!space->IsValidControl(temp.milestones[i],temp.controls[i])) {
        LOG4CXX_INFO(KrisLibrary::logger(),"Control "<<i);
        return 0.0;
      }
    if(!temp.IsFeasible()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Edge became infeasible\n");
      return 0.0;
    }
    return 1.0;
  }
};

//on the combined parameter vector (u0,...,un-1,x1,...,xn)
//Returns the error between the prescribed milestones and the actual
//ones
class PathDynamicEqualityFunction : public VectorFieldFunction
{
public:
  KinodynamicSpace* space;
  KinodynamicMilestonePath temp;
  Matrix jacobian;
  PathDynamicEqualityFunction(KinodynamicSpace* _space,const KinodynamicMilestonePath& path)
  :space(_space),temp(path)
  {
    for(size_t i=1;i<temp.milestones.size();i++) {
      temp.milestones[i].clear();
      temp.milestones[i].setRef(path.milestones[i]);
    }
    for(size_t i=0;i<temp.controls.size();i++) {
      temp.controls[i].clear();
      temp.controls[i].setRef(path.controls[i]);
    }
  }
  virtual int NumDimensions() const {
    return (temp.milestones.size()-1)*temp.milestones[0].n;
  }
  virtual void PreEval(const Vector& u) {
    Assert(u.n == (int)temp.controls.size()*(temp.controls[0].n+temp.milestones[0].n));
    int nu=temp.controls[0].n;
    int nx=temp.milestones[0].n;
    int xoffset = nu*(int)temp.controls.size();
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.controls[i].setRef(u,i*nu,1,nu);  
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.milestones[i+1].setRef(u,xoffset+i*nx,1,nx);
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.paths[i] = space->Simulate(temp.milestones[i],temp.controls[i]);
  }
  virtual void Eval(const Vector& u,Vector& err)
  {
    err.resize(NumDimensions());
    for(size_t i=0;i<temp.paths.size();i++)
      err.copySubVector(i*temp.milestones[0].n,temp.milestones[i+1]-temp.paths[i]->End());
  }
  virtual void Jacobian(const Vector& u,Matrix& J) {
    J.resize(NumDimensions(),(temp.milestones.size()-1)*(temp.controls[0].n+temp.milestones[0].n));
    J.setZero();
    int ofs = (temp.milestones.size()-1)*temp.controls[0].n;
    for(int i=0;i<J.m;i++)
      J(i,ofs+i) = 1.0;
  }
};

//on the combined parameter vector (u0,...,un-1,x1,...,xn)
//Returns the path cost given the controls
class PathCostFunction : public ScalarFieldFunction
{
public:
  KinodynamicSpace* space;
  KinodynamicMilestonePath temp;
  ObjectiveFunctionalBase* obj;
  PathCostFunction(KinodynamicSpace* _space,const KinodynamicMilestonePath& path,ObjectiveFunctionalBase* _obj)
  :space(_space),temp(path),obj(_obj)
  {
    for(size_t i=1;i<temp.milestones.size();i++) {
      temp.milestones[i].clear();
      temp.milestones[i].setRef(path.milestones[i]);
    }
    for(size_t i=0;i<temp.controls.size();i++) {
      temp.controls[i].clear();
      temp.controls[i].setRef(path.controls[i]);
    }
  }
  virtual int NumDimensions() const {
    return (temp.milestones.size()-1)*temp.milestones[0].n;
  }
  virtual void PreEval(const Vector& u) {
    Assert(u.n == (int)temp.controls.size()*(temp.controls[0].n+temp.milestones[0].n));
    int nu=temp.controls[0].n;
    int nx=temp.milestones[0].n;
    int xoffset = nu*(int)temp.controls.size();
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.controls[i].setRef(u,i*nu,1,nu);  
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.milestones[i+1].setRef(u,xoffset+i*nx,1,nx);
    for(size_t i=0;i<temp.controls.size();i++) 
      temp.paths[i] = space->Simulate(temp.milestones[i],temp.controls[i]);
  }
  virtual Real Eval(const Vector& u)
  {
    return obj->PathCost(temp);
  }
};

bool KinodynamicLocalOptimizer::DoDDP()
{
  int n = space->GetStateSpace()->NumDimensions();
  LOG4CXX_INFO(KrisLibrary::logger(),"Trying DDP\n");
  Matrix Ak;
  Vector bk;
  //Real ck;
  TerminalCostFunction term(objective.get());
  term.PreEval(bestPath.milestones.back());
  //ck = term.Eval(bestPath.milestones.back());
  term.Gradient(bestPath.milestones.back(),bk);
  term.Hessian(bestPath.milestones.back(),Ak);
  vector<ControlInput> du(bestPath.controls.size());
  vector<State> dx(bestPath.controls.size());
  Vector gugrad,gxgrad;
  Matrix guhess,fugrad,gxhess,fxgrad;
  vector<Matrix> fuhess,fxhess;
  Matrix temp,temp2,temp3,temp4;
  Matrix Akprev;
  Vector bkprev;
  for(int i=bestPath.controls.size()-1;i>=0;i--) {
    Vector dJ;
    Matrix d2J;
    //gradient of future cost with respect to x,
    //2*Ak*x[i+1] + bk
    Vector costGradX;
    Ak.mul(bestPath.milestones[i+1],costGradX);
    costGradX *= 2;
    costGradX += bk;
    LOG4CXX_INFO(KrisLibrary::logger(),"Cost gradient w.r.t. x: "<<costGradX);

    SimulateFunction fi(space,bestPath.milestones[i]);
    fi.PreEval(bestPath.controls[i]);
    fi.Jacobian(bestPath.controls[i],fugrad);
    AllHessiansCenteredDifferences(fi,bestPath.controls[i],1e-3,fuhess);
    IncrementalCostFunction gi(space,objective.get(),bestPath.milestones[i]);
    gi.PreEval(bestPath.controls[i]);
    gi.Gradient(bestPath.controls[i],gugrad);
    gi.Hessian(bestPath.controls[i],guhess);

    //compute the gradient of the current cost w.r.t. u[i]
    fugrad.mulTranspose(costGradX,dJ);
    dJ += gugrad;

    //compute the hessian of the current cost w.r.t. u[i]
    d2J = guhess;
    for(int j=0;j<n;j++) 
      d2J.madd(fuhess[j],costGradX[j]);
    temp.mul(Ak,fugrad);
    temp2.mulTransposeA(fugrad,temp);
    d2J += temp2;

    LOG4CXX_INFO(KrisLibrary::logger(),d2J<<"* du + "<<dJ<<"\n");

    //damping helps singular matrices
    for(int j=0;j<d2J.n;j++)
      d2J(j,j) += 1e-2;

    //solve d2J*du = dJ
    LDLDecomposition<Real> ldl;
    ldl.set(d2J);
    if(!ldl.backSub(dJ,du[i])) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicLocalOptimizer: Hessian matrix of control "<< i<<" is degenerate, moving along gradient instead\n");
      du[i] = dJ;
    }
    du[i].inplaceNegative();
    /*
    if(du[i].norm() > optimizer->trustRegionSize)
      du[i] *= optimizer->trustRegionSize / du[i].norm();
      */
    LOG4CXX_INFO(KrisLibrary::logger(),"Du "<<i<<": "<<du[i]);
    //now that u[i] moves to u[i] + du[i], compute 2nd order approximation to the cost as a function of x[i].  Store in Ak, bk 
    Vector xnext;
    Vector u = bestPath.controls[i]+du[i];
    if(space->GetControlSet(bestPath.milestones[i])->Project(u)) {
      //shrink the step
      du[i] = u - bestPath.controls[i];
    }
    //simulate next step
    fi(u,xnext);
    dx[i] = xnext - bestPath.milestones[i+1];

    Ak.mul(xnext,costGradX);
    costGradX *= 2;
    costGradX += bk;
    
    SimulateFunctionX fui(space,u);
    fui.PreEval(bestPath.milestones[i]);
    fui.Jacobian(bestPath.milestones[i],fxgrad);
    AllHessiansCenteredDifferences(fui,bestPath.milestones[i],1e-3,fxhess);
    IncrementalCostFunctionX gui(space,objective.get(),u);
    gui.PreEval(bestPath.milestones[i]);
    gui.Gradient(bestPath.milestones[i],gxgrad);
    gui.Hessian(bestPath.milestones[i],gxhess);

    //compute the gradient of the current cost w.r.t. x[i]
    fxgrad.mulTranspose(costGradX,bkprev);
    bkprev += gxgrad;

    //compute the hessian of the current cost w.r.t. x[i]
    Akprev = gxhess;
    for(int j=0;j<n;j++) 
      Akprev.madd(fxhess[j],costGradX[j]);
    temp3.mul(Ak,fxgrad);
    temp4.mulTransposeA(fxgrad,temp3);
    Akprev += temp4;

    //recurse downward
    Ak = Akprev;
    bk = bkprev;
  }

  /*
  if(optimizer->x.n != (int)bestPath.controls.size()*bestPath.controls[0].n) {
    //need to redo x
    optimizer->baseProgram.f = new PathCostFunction(space,bestPath,objective);
    optimizer->baseProgram.c = new PathDynamicEqualityFunction(space,bestPath);
    //TODO: numerical representations of control constraints in NLP will help it converge faster... 
    optimizer->constraintFunc = new PathFeasibilityFunction(space,bestPath,goalSet);
    optimizer->fx = bestPathCost;
    optimizer->numSamplesPerStep = 100;
    Vector uall,xall;
    Stack(bestPath.controls,uall);
    Stack(bestPath.milestones,xall);
    Vector x1ton; x1ton.setRef(xall,n,1,xall.n-n);
    Stack(uall,x1ton,optimizer->x);
  }
  Vector duall,dxall,dux;
  Stack(du,duall);
  Stack(dx,dxall);
  Stack(duall,dxall,dux);
  ConvergenceResult res = optimizer->SolveStepCustom(dux);
  LOG4CXX_INFO(KrisLibrary::logger(),"New trust region size "<<optimizer->trustRegionSize<<", "<<optimizer->points.size());
  if(res != ConvergenceError) {
    PathFeasibilityFunction* pf = dynamic_cast<PathFeasibilityFunction*>(&*optimizer->constraintFunc);
    pf->PreEval(optimizer->x);
    bestPath = pf->temp;
    ComputeCosts();
    return true;
  }
  */
  return false;
}

bool KinodynamicLocalOptimizer::DoGradientDescent()
{
  LOG4CXX_INFO(KrisLibrary::logger(),"Trying gradient descent\n");
  //similar to ilqg except only first order
  vector<Matrix> fxgrad,fugrad;
  vector<Vector> gxgrad,gugrad;
  Vector terminalxgrad;
  fxgrad.resize(bestPath.controls.size());
  fugrad.resize(bestPath.controls.size());
  gxgrad.resize(bestPath.controls.size());
  gugrad.resize(bestPath.controls.size());
  for(size_t i=0;i<bestPath.controls.size();i++) {
    SimulateFunction fi(space,bestPath.milestones[i]);
    fi.PreEval(bestPath.controls[i]);
    fi.Jacobian(bestPath.controls[i],fugrad[i]);
    SimulateFunctionX fui(space,bestPath.controls[i]);
    fui.PreEval(bestPath.milestones[i]);
    fui.Jacobian(bestPath.milestones[i],fxgrad[i]);
    IncrementalCostFunction gi(space,objective.get(),bestPath.milestones[i]);
    gi.PreEval(bestPath.controls[i]);
    gi.Gradient(bestPath.controls[i],gugrad[i]);
    IncrementalCostFunctionX gui(space,objective.get(),bestPath.controls[i]);
    gui.PreEval(bestPath.milestones[i]);
    gui.Gradient(bestPath.milestones[i],gxgrad[i]);
  }
  TerminalCostFunction term(objective.get());
  term.PreEval(bestPath.milestones.back());
  term.Gradient(bestPath.milestones.back(),terminalxgrad);
  //compute overall cost functional gradients with respect to x and u, backwards recursion
  vector<Vector> jugrad(bestPath.controls.size());
  vector<Vector> jxgrad(bestPath.controls.size()+1);
  jxgrad.back() = terminalxgrad;
  for(int i=(int)bestPath.controls.size()-1;i>=0;i--) {
    fugrad[i].mulTranspose(jxgrad[i+1],jugrad[i]);
    jugrad[i] += gugrad[i];
    fxgrad[i].mulTranspose(jxgrad[i+1],jxgrad[i]);
    jxgrad[i] += gxgrad[i];
  }

  /*
  if(optimizer->x.n != (int)bestPath.controls.size()*bestPath.controls[0].n) {
    //need to redo x
    optimizer->baseProgram.f = new PathCostFunction(space,bestPath,objective);
    //TODO: numerical representations of control constraints in NLP will help it converge faster... 
    optimizer->constraintFunc = new PathFeasibilityFunction(space,bestPath,goalSet);
    optimizer->fx = bestPathCost;
    optimizer->numSamplesPerStep = 100;
    int n = bestPath.milestones[0].n;
    Vector uall,xall;
    Stack(bestPath.controls,uall);
    Stack(bestPath.milestones,xall);
    Vector x1ton; x1ton.setRef(xall,n,1,xall.n-n);
    Stack(uall,xall,optimizer->x);
  }
  Vector duall,dxall,dux;
  Stack(jugrad,duall);
  Stack(jxgrad,dxall);
  Stack(duall,dxall,dux);
  duall.inplaceNegative();
  ConvergenceResult res = optimizer->SolveStepCustom(dux);
  LOG4CXX_INFO(KrisLibrary::logger(),"New trust region size "<<optimizer->trustRegionSize<<", "<<optimizer->points.size());
  if(res != ConvergenceError) {
    PathFeasibilityFunction* pf = dynamic_cast<PathFeasibilityFunction*>(&*optimizer->constraintFunc);
    pf->PreEval(optimizer->x);
    bestPath = pf->temp;
    ComputeCosts();
    return true;
  }
  */
  return false;
  /*
  tempPath = bestPath;
  bool hadSuccessfulStep = false;
  Real stepSize = 1.0;
  Real minStepSize = 0.0, maxStepSize = Inf;
  while(stepSize > 1e-6 && (maxStepSize - stepSize > 1e-1)) {
    bool successful = true;
    for(size_t i=0;i<tempPath.controls.size();i++)  {
      tempPath.controls[i] = bestPath.controls[i];
      tempPath.controls[i].madd(jugrad[i],-stepSize);
    }
    tempPath.SimulateFromControls(space);
    for(size_t i=0;i<tempPath.controls.size();i++) {
      if(!space->IsValidControl(tempPath.milestones[i],tempPath.controls[i]))  {
        successful = false;
        break;
      }
      if(!space->GetStateSpace()->IsFeasible(tempPath.milestones[i])) {
        successful = false;
        break;
      }
    }
    if(successful) {
      if(goalSet && !goalSet->Contains(tempPath.milestones.back())) {
        successful = false;
      }
      else if(objective->PathCost(tempPath) > bestPathCost) {
        successful = false;
      }
      else {
        tempPath.MakeEdges(space);
        if(!tempPath.IsFeasible()) {
          successful = false;
        }
      }
    }

    if(successful) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Stepped to path with cost "<<objective->PathCost(tempPath));
      bestPath = tempPath;
      bestPathCost = objective->PathCost(bestPath);
      hadSuccessfulStep = true;
      minStepSize = stepSize;
      if(IsInf(maxStepSize))
        stepSize *= 2.0;
      else
        stepSize = (stepSize + maxStepSize)*0.5;
    }
    else {
      ///unsuccessful: shrink the step size
      if(hadSuccessfulStep) {
        maxStepSize = stepSize;
        stepSize = (stepSize + minStepSize)*0.5;
      }
      else {
        stepSize *= 0.5;
      }
    }
  }
  if(hadSuccessfulStep) {
    ComputeCosts();
    LOG4CXX_INFO(KrisLibrary::logger(),"Cost is "<<bestPathCost);
    return true;
  }
  return false;
  */
}

bool KinodynamicLocalOptimizer::Done() const
{
  return !bestPath.Empty();
}

bool KinodynamicLocalOptimizer::GetPath(KinodynamicMilestonePath& path)
{
  path = bestPath;
  return (!bestPath.Empty());
}


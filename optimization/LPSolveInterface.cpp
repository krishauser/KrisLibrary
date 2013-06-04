#include "LPSolveInterface.h"
#if HAVE_LP_SOLVE

#if defined(__APPLE__) || defined(MACOSX)
#include <lpsolve/lp_lib.h>
#else
#include <lp_lib.h>
#endif

#include <algorithm>

using namespace Optimization;
using namespace std;

#if MAJORVERSION!=5
  #error "Must use LPSolve version 5.0 or higher"
#endif

bool LPSolveInterface::Enabled() { return true; }

void LPSolveInterface::SelfTest()
{
  LinearProgram lp;
  lp.Resize(6,18);
  lp.minimize = false;

  //setup A
  Real vals[6][18] = {
    {9.8,0,-0.6,-0.6,-0.6,-0.6,-0.6,-0.6,-0.6,-0.6,-0.4,-0.4,-0.4,-0.4,-0.4,-0.4,-0.4,-0.4},
    {0,-9.8,-0.2,-0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.2,-0.2,-0.2,-0.2},
    {0,0,1,0,-1,0,1,0,-1,0,1,0,-1,0,1,0,-1,0},
    {0,0,0,1,0,-1,0,1,0,-1,0,1,0,-1,0,1,0,-1},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {0,0,0.2,0.6,-0.2,-0.6,0.3,0.6,-0.3,-0.6,0.3,0.4,-0.3,-0.4,0.2,0.4,-0.2,-0.4}};
  for(int i=0;i<6;i++)
    lp.A.copyRow(i,vals[i]);
  lp.q.setZero();
  lp.q(4) = 9.8;
  lp.p = lp.q;

  lp.c.setZero();
  for(int i=2;i<18;i++)
    lp.l(i) = 0;

  LPSolveInterface lps;
  lps.Set(lp);
  Vector xopt;

  cout<<"Self test problem: "<<endl;
  lp.Print();

  lp.c(0) = 1; lp.c(1) = 0;
  lps.SetObjective(lp.c);
  lps.Solve(xopt);
  cout<<"Self-test result 1:"<<endl;
  cout<<xopt<<endl;

  lp.c(0) = -1; lp.c(1) = -0;
  lps.SetObjective(lp.c);
  lps.Solve(xopt);
  cout<<"Self-test result 2:"<<endl;
  cout<<xopt<<endl;
}

LPSolveInterface::LPSolveInterface()
:lp(NULL)
{
}

LPSolveInterface::~LPSolveInterface()
{
  SafeDeleteProc(lp,delete_lp);
}

void LPSolveInterface::Set(const LinearProgram& LP)
{
  cout<<"LPSolve::Set()"<<endl;
  SafeDeleteProc(lp,delete_lp);
  lp = make_lp(LP.A.m,LP.A.n);

  if(LP.minimize) set_minim(lp);
  else set_maxim(lp);

  REAL* temp = new REAL[LP.A.n+1];
  temp[0] = 0;

  for(int j=0;j<LP.A.n;j++) temp[j+1] = LP.c[j];
  set_obj_fn(lp,temp);

  cout<<"1"<<endl;
  set_add_rowmode(lp,TRUE);
  assert(LP.q.n == LP.A.m);
  assert(LP.p.n == LP.A.m);
  for(int i=0;i<LP.A.m;i++) {
    cout<<"adding row "<<i<<endl;
    for(int j=0;j<LP.A.n;j++) temp[j+1] = LP.A(i,j);
    switch(LP.ConstraintType(i)) {
    case LinearProgram::Free:
      break;
    case LinearProgram::Fixed:
      add_constraint(lp,temp,EQ,LP.q[i]);
      break;
    case LinearProgram::Bounded:
      add_constraint(lp,temp,GE,LP.q[i]);
      add_constraint(lp,temp,LE,LP.p[i]);
      break;
    case LinearProgram::LowerBound:
      add_constraint(lp,temp,GE,LP.q[i]);
      break;
    case LinearProgram::UpperBound:
      add_constraint(lp,temp,LE,LP.p[i]);
      break;
    }
  }
  delete [] temp;

  cout<<"2"<<endl;
  for(int j=0;j<LP.A.n;j++) {
    set_bounds(lp,j+1,Max((REAL)LP.l(j),-1e30),Min((REAL)LP.u(j),1e30));
  }

  cout<<"3"<<endl;
  set_scaling(lp, SCALE_CURTISREID);
  //set_verbose(lp,NORMAL);
  set_verbose(lp, SEVERE);
#if MINORVERSION < 5
  set_presolve(lp,PRESOLVE_COLS | PRESOLVE_ROWS);
#else
  set_presolve(lp,PRESOLVE_COLS | PRESOLVE_ROWS,1);
#endif
  cout<<"4"<<endl;
  int res=set_BFP(lp,"bfp_LUSOL");
  if(res != 1) {
    cerr<<"LPSolveInterface: Error setting basis factorization package"<<endl;
  }
  set_epsb(lp,1e-4);
  set_epsd(lp,1e-4);
  set_epsel(lp,1e-5);
  //set_epsint(lp,1e-5);
  set_epsperturb(lp,1e-4);
  set_epspivot(lp,1e-4);
  cout<<"Done"<<endl;
}

//void LPSolveInterface::SetMatrix(const Matrix& A);
void LPSolveInterface::SetObjective(const Vector& c)
{
  cout<<"TODO: for some reason, this doesn't work with lpsolve 5.1"<<endl;
  abort();
  int n = get_Ncolumns(lp);
  assert(n == c.n);
  REAL* temp = new REAL[n+1];
  temp[0] = 0; for(int j=0;j<n;j++) temp[j+1] = c[j];
  set_obj_fn(lp,temp);
  delete [] temp;
}

void LPSolveInterface::SetObjectiveBreak(Real val)
{
  set_break_at_value(lp, val);
}

LinearProgram::Result LPSolveInterface::Solve(Vector& xopt)
{
  cout<<"LPSolve begin"<<endl;
  int res=solve(lp);
  cout<<"LPSolve end"<<endl;
  switch(res) {
  case OPTIMAL:
    break;
  case SUBOPTIMAL:
    printf("A suboptimal solution was found\n");
    break;
  case INFEASIBLE:
    return LinearProgram::Infeasible;
  case UNBOUNDED:
    return LinearProgram::Unbounded;
  case NUMFAILURE:
    printf("LPSolveInterface::Solve(): Numerical failure!\n");
    return LinearProgram::Error;
  default:
    printf("LPSolveInterface::Solve(): Got an unknown result, %d\n", res);
    return LinearProgram::Error;
  }
  int n = get_Ncolumns(lp);
  REAL* xd;
  get_ptr_variables(lp,&xd);
  xopt.resize(n);
  for(int i=0;i<n;i++) xopt[i] = xd[i];

  //no resetting
  //default_basis(lp);
  return LinearProgram::Feasible;
}


#else

using namespace Optimization;
using namespace std;

bool LPSolveInterface::Enabled() { return false; }

LPSolveInterface::LPSolveInterface()
{
}

LPSolveInterface::~LPSolveInterface()
{
}

#define SEGFAULT { *((int*)0) = 1; }

void LPSolveInterface::Set(const LinearProgram& LP)
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
}

void LPSolveInterface::SetMatrix(const Matrix& A)
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
}

void LPSolveInterface::SetObjective(const Vector& c)
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
}

void LPSolveInterface::SetObjectiveBreak(Real val)
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
}

LinearProgram::Result LPSolveInterface::Solve(Vector& xopt)
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
  return LinearProgram::Error;
}

void LPSolveInterface::SelfTest()
{
  cerr<<"Warning, LPSolve not defined"<<endl;
  SEGFAULT;
}

#endif // HAVE_LP_SOLVE

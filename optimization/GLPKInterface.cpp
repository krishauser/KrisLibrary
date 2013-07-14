#include "GLPKInterface.h"
#if HAVE_GLPK

#include <utils.h>
#include <utils/SignalHandler.h>
#include <signal.h>
#include <stdexcept>
extern "C"
{
#include <glpk.h>
}
using namespace Optimization;
using namespace std;

#define GLPK_FAULT_STYLE_4_15 0
#define GLPK_FAULT_STYLE_4_39 1

//workaround for GLPK 4.15
#if GLPK_FAULT_STYLE_4_15
extern "C" {
void _glp_lib_print_hook(int (*func)(void *info, const char *buf), void *info);
void _glp_lib_fault_hook(int (*func)(void *info, const char *buf), void *info);
}
#elif GLPK_FAULT_STYLE_4_39
//workaround for GLPK 4.39
extern "C" {
void glp_term_hook(int (*func)(void *info, const char *buf), void *info);
//void glp_fault_hook(int (*func)(void *info, const char *buf), void *info);
#define glp_fault_hook glp_term_hook
}
#endif


const static Real kZeroTol = 1e-6;

GLPKInterface::GLPKInterface()
:lp(NULL)
{}

GLPKInterface::~GLPKInterface()
{
  SafeDeleteProc(lp,lpx_delete_prob);
}

inline int BoundType(Real low,Real high)
{
  if(IsInf(low)==-1) {
    if(IsInf(high) == 1) return LPX_FR;
    return LPX_UP;
  }
  else if(IsInf(high) == 1) {
    return LPX_LO;
  }
  else {
    if(low==high) return LPX_FX;
    else return LPX_DB;
  }
}

int BoundTypeToLPX(LinearProgram::BoundType b)
{
  switch(b) {
  case LinearProgram::Free:       return LPX_FR;
  case LinearProgram::LowerBound: return LPX_LO;
  case LinearProgram::UpperBound: return LPX_UP;
  case LinearProgram::Bounded:    return LPX_DB;
  case LinearProgram::Fixed:      return LPX_FX;
  default: abort(); return LPX_FR;
  }
}

void GLPKInterface::Set(const LinearProgram& LP)
{
  SafeDeleteProc(lp,lpx_delete_prob);
  lp = lpx_create_prob();
  if(LP.minimize) lpx_set_obj_dir(lp,LPX_MIN);
  else lpx_set_obj_dir(lp,LPX_MAX);

  lpx_add_rows(lp,LP.A.m);
  for(int i=0;i<LP.A.m;i++) {
    lpx_set_row_bnds(lp,i+1,BoundTypeToLPX(LP.ConstraintType(i)),LP.q(i),LP.p(i)); 
  }
  lpx_add_cols(lp,LP.A.n);
  for(int i=0;i<LP.A.n;i++) {
    lpx_set_col_bnds(lp,i+1,BoundTypeToLPX(LP.VariableType(i)),LP.l(i),LP.u(i)); 
  }
  for(int i=0;i<LP.A.n;i++)
    lpx_set_obj_coef(lp,i+1,LP.c(i));

  vector<int> itemp(LP.A.n+1);
  dVector temp(LP.A.n+1);
  for(int i=0;i<LP.A.m;i++) {
    //pick nonzero entries
    int nnz=0;
    for(int j=0;j<LP.A.n;j++) {
      if(!FuzzyZero(LP.A(i,j),kZeroTol)) {
        itemp[nnz+1] = j+1;
        temp(nnz+1) = LP.A(i,j);
        nnz++;
      }
    }
    lpx_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
  }
  //set to only report errors
  lpx_set_int_parm(lp,LPX_K_MSGLEV,1);

  //lpx_scale_prob(lp);
  //lpx_adv_basis(lp);
  //lpx_set_int_parm(lp,LPX_K_PRESOL,1);
  lpx_set_int_parm(lp,LPX_K_PRESOL,0);
}

void GLPKInterface::Set(const LinearProgram_Sparse& LP)
{
  SafeDeleteProc(lp,lpx_delete_prob);
  lp = lpx_create_prob();
  if(LP.minimize) lpx_set_obj_dir(lp,LPX_MIN);
  else lpx_set_obj_dir(lp,LPX_MAX);

  lpx_add_rows(lp,LP.A.m);
  for(int i=0;i<LP.A.m;i++) {
    lpx_set_row_bnds(lp,i+1,BoundTypeToLPX(LP.ConstraintType(i)),LP.q(i),LP.p(i)); 
  }
  lpx_add_cols(lp,LP.A.n);
  for(int i=0;i<LP.A.n;i++) {
    lpx_set_col_bnds(lp,i+1,BoundTypeToLPX(LP.VariableType(i)),LP.l(i),LP.u(i)); 
  }
  for(int i=0;i<LP.A.n;i++)
    lpx_set_obj_coef(lp,i+1,LP.c(i));

  vector<int> itemp(LP.A.n+1);
  dVector temp(LP.A.n+1);
  for(int i=0;i<LP.A.m;i++) {
    //pick nonzero entries
    int nnz=0;
    for(SparseMatrix::RowT::const_iterator j=LP.A.rows[i].begin();j!=LP.A.rows[i].end();j++) {
      if(!FuzzyZero(j->second,kZeroTol)) {
        itemp[nnz+1] = j->first+1;
        temp(nnz+1) = j->second;
        nnz++;
      }
    }
    lpx_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
  }
  //set to only report errors
  lpx_set_int_parm(lp,LPX_K_MSGLEV,1);

  //lpx_scale_prob(lp);
  //lpx_adv_basis(lp);
  //lpx_set_int_parm(lp,LPX_K_PRESOL,1);
  lpx_set_int_parm(lp,LPX_K_PRESOL,0);
}

void GLPKInterface::Clear()
{
  SafeDeleteProc(lp,lpx_delete_prob);
}

void GLPKInterface::Create(int m,int n)
{
  SafeDeleteProc(lp,lpx_delete_prob);
  lp = lpx_create_prob();
  lpx_add_rows(lp,m);
  lpx_add_cols(lp,n);

  //set to only report errors
  lpx_set_int_parm(lp,LPX_K_MSGLEV,1);

  //lpx_scale_prob(lp);
  //lpx_adv_basis(lp);
  //lpx_set_int_parm(lp,LPX_K_PRESOL,1);
  lpx_set_int_parm(lp,LPX_K_PRESOL,0);

}

void GLPKInterface::SetObjective(const Vector& obj,bool minimize)
{
  for(int i=0;i<obj.n;i++)
    lpx_set_obj_coef(lp,i+1,obj(i));
  if(minimize) lpx_set_obj_dir(lp,LPX_MIN);
  else lpx_set_obj_dir(lp,LPX_MAX);
}

void GLPKInterface::SetRow(int i,const Vector& Ai)
{
  vector<int> itemp(Ai.n+1);
  dVector temp(Ai.n+1);
  //pick nonzero entries
  int nnz=0;
  for(int j=0;j<Ai.n;j++) {
    if(!FuzzyZero(Ai(j),kZeroTol)) {
      itemp[nnz+1] = j+1;
      temp(nnz+1) = Ai(j);
      nnz++;
    }
  }
  lpx_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
}

void GLPKInterface::SetRowBounds(int i,Real low,Real high)
{
  lpx_set_row_bnds(lp,i+1,BoundType(low,high),low,high); 
}

void GLPKInterface::SetVariableBounds(int j,Real low,Real high)
{
  lpx_set_col_bnds(lp,j+1,BoundType(low,high),low,high); 
}


void GLPKInterface::SetRowBasic(int i)
{
  lpx_set_row_stat(lp,i+1,LPX_BS);
}

bool GLPKInterface::GetRowBasic(int i)
{
  return lpx_get_row_stat(lp,i+1)==LPX_BS;
}

double GLPKInterface::GetRowDual(int i){
	return lpx_get_row_dual(lp, i+1);
}

double GLPKInterface::GetVariableDual(int j){
	return lpx_get_col_dual(lp, j+1);
}

void GLPKInterface::SetRowNonBasic(int i,bool upper)
{
  if(upper) lpx_set_row_stat(lp,i+1,LPX_NU);
  else lpx_set_row_stat(lp,i+1,LPX_NL);
}

void GLPKInterface::SetVariableBasic(int j)
{
  lpx_set_col_stat(lp,j+1,LPX_BS);
}

bool GLPKInterface::GetVariableBasic(int j)
{
  return lpx_get_col_stat(lp,j+1)==LPX_BS;
}

void GLPKInterface::SetVariableNonBasic(int j,bool upper)
{
  if(upper) lpx_set_col_stat(lp,j+1,LPX_NU);
  else lpx_set_col_stat(lp,j+1,LPX_NL);
}


int my_glpx_fault_handler(void* info,const char* msg)
{
  printf("GLPK error message %s\n",msg);
  //printf("GLPK fatal error %s\n",msg);
  //printf("jumping...\n");
  //throw(std::runtime_error(msg));
  /*
  printf("GLPK fatal error, dumping!\n");
  LPX* lp=(LPX*)info;
  lpx_write_cpxlp(lp,"temp_lp.txt");
  */
  return 0;
}

struct GLPKInterruptHandler : public SignalHandler
{
public:
  GLPKInterruptHandler(GLPKInterface* _glpk)
    :glpk(_glpk)
  {}

  virtual void OnRaise(int signum) 
  {
    printf("Interrupt called during GLPK solve... possible infinite loop\n");
    LPX* lp=glpk->lp;;
    lpx_write_cpxlp(lp,"temp_lp.txt");
    throw(std::runtime_error("Interrupt called during GLPK solve"));
    //exit(-1);
  }

  GLPKInterface* glpk;
};

LinearProgram::Result GLPKInterface::Solve(Vector& xopt)
{
  assert(lp != NULL);
  //lpx_write_cpxlp(lp,"temp_lp.txt");
  //lpx_print_prob(lp,"temp_lp.txt");
#if GLPK_FAULT_STYLE_4_15
  _glp_lib_fault_hook(my_glpx_fault_handler,0);
#elif GLPK_FAULT_STYLE_4_39
  glp_fault_hook(my_glpx_fault_handler,0);
#else  //older versions of GLPK
  lib_set_fault_hook(lp,my_glpx_fault_handler);
#endif // GLPK_FAULT_STYLE

  GLPKInterruptHandler handler(this);
  handler.SetCurrent(SIGINT);
  //handler.SetCurrent(SIGABRT);
  int res;
  try {
    res=lpx_simplex(lp);
  }
  catch(const std::exception& e) {
    printf("GLPK internal error: ");
    printf(e.what());
    return LinearProgram::Error;
  }
  catch (...) {
    printf("Unknown error occurred\n");
    return LinearProgram::Error;
  }
#if GLPK_FAULT_STYLE_4_15
  _glp_lib_fault_hook(0,0);
#elif GLPK_FAULT_STYLE_4_39
  glp_fault_hook(0,0);
#else
  lib_set_fault_hook(NULL,NULL);
#endif //GLP_FAULT_STYLE
  handler.UnsetCurrent(SIGINT);
  switch(res) {
  case LPX_E_OK:
    break;
  case LPX_E_FAULT:
    cout<<"Error in matrix construction!"<<endl;
    return LinearProgram::Error;
  case LPX_E_OBJLL:
    cout<<"Objective reached lower limit!"<<endl;
    return LinearProgram::Error;
  case LPX_E_OBJUL:
    cout<<"Objective reached upper limit!"<<endl;
    return LinearProgram::Error;
  case LPX_E_NOPFS:
    cout<<"Linear program has no primary feasible solution!"<<endl;
    return LinearProgram::Infeasible;
  case LPX_E_NODFS:
    cout<<"Linear program has no dual feasible solution!"<<endl;
    return LinearProgram::Infeasible;
  case LPX_E_ITLIM:
    cout<<"Max iterations reached!"<<endl;
    return LinearProgram::Error;
  case LPX_E_TMLIM:
    cout<<"Time limit reached!"<<endl;
    return LinearProgram::Error;
  case LPX_E_SING:
    cout<<"Singularity reached!"<<endl;
    return LinearProgram::Error;
  }

  int stat=lpx_get_status(lp);
  int n=lpx_get_num_cols(lp);
  xopt.resize(n);
  for(int i=0;i<n;i++)
    xopt(i) = (Real)lpx_get_col_prim(lp,i+1);

  switch(stat) {
  case LPX_OPT:
  case LPX_FEAS:
    return LinearProgram::Feasible;
  case LPX_INFEAS:
  case LPX_NOFEAS:
    return LinearProgram::Infeasible;
  case LPX_UNBND:
    return LinearProgram::Unbounded;
  case LPX_UNDEF:
    cout<<"Solution is undefined!"<<endl;
    return LinearProgram::Error;
  default:
    cout<<"Shouldn't get here!"<<endl;
    return LinearProgram::Error;
  }
}

bool GLPKInterface::Enabled() { return true; }

#else

using namespace Optimization;
using namespace std;

GLPKInterface::GLPKInterface()
{
}

GLPKInterface::~GLPKInterface()
{
}

void GLPKInterface::Set(const LinearProgram& LP)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::Set(const LinearProgram_Sparse& LP)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::Create(int m,int n)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::Clear()
{
}

void GLPKInterface::SetRow(int i,const Vector& Ai)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::SetRowBounds(int i,Real low,Real high)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::SetVariableBounds(int j,Real low,Real high)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::SetRowBasic(int i)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

bool GLPKInterface::GetRowBasic(int i)
{
  cerr<<"Warning, GLPK not defined"<<endl;
  return false;
}

double GLPKInterface::GetRowDual(int i){
  cerr<<"Warning, GLPK not defined"<<endl;
  return 0;
}

void GLPKInterface::SetRowNonBasic(int i,bool upper)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

void GLPKInterface::SetVariableBasic(int j)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

bool GLPKInterface::GetVariableBasic(int i)
{
  cerr<<"Warning, GLPK not defined"<<endl;
  return false;
}

void GLPKInterface::SetVariableNonBasic(int j,bool upper)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}


void GLPKInterface::SetObjective(const Vector& c,bool minimize)
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

LinearProgram::Result GLPKInterface::Solve(Vector& xopt)
{
  cerr<<"Warning, GLPK not defined"<<endl;
  return LinearProgram::Error;
}

bool GLPKInterface::Enabled() { return false; }

void GLPKInterface::SelfTest()
{
  cerr<<"Warning, GLPK not defined"<<endl;
}

#endif //HAVE_GLPK

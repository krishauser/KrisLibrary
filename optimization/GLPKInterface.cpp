#include <KrisLibrary/Logger.h>
#include "GLPKInterface.h"
#if HAVE_GLPK

#include <utils.h>
#include <utils/SignalHandler.h>
#include <signal.h>
#include <iostream>
#include <stdexcept>
extern "C"
{
#include <glpk.h>
}
using namespace Optimization;
using namespace std;

#if GLP_MAJOR_VERSION < 4 || GLP_MINOR_VERSION < 20
#error "Require GLPK 4.20 or above"
#endif

const static Real kZeroTol = 1e-6;

GLPKInterface::GLPKInterface()
:lp(NULL)
{}

GLPKInterface::~GLPKInterface()
{
  SafeDeleteProc(lp,glp_delete_prob);
}

inline int BoundType(Real low,Real high)
{
  if(IsInf(low)==-1) {
    if(IsInf(high) == 1) return GLP_FR;
    return GLP_UP;
  }
  else if(IsInf(high) == 1) {
    return GLP_LO;
  }
  else {
    if(low==high) return GLP_FX;
    else return GLP_DB;
  }
}

int BoundTypeToGLP(LinearProgram::BoundType b)
{
  switch(b) {
  case LinearProgram::Free:       return GLP_FR;
  case LinearProgram::LowerBound: return GLP_LO;
  case LinearProgram::UpperBound: return GLP_UP;
  case LinearProgram::Bounded:    return GLP_DB;
  case LinearProgram::Fixed:      return GLP_FX;
  default: abort(); return GLP_FR;
  }
}

void GLPKInterface::Set(const LinearProgram& LP)
{
  SafeDeleteProc(lp,glp_delete_prob);
  lp = glp_create_prob();
  if(LP.minimize) glp_set_obj_dir(lp,GLP_MIN);
  else glp_set_obj_dir(lp,GLP_MAX);

  glp_add_rows(lp,LP.A.m);
  for(int i=0;i<LP.A.m;i++) {
    glp_set_row_bnds(lp,i+1,BoundTypeToGLP(LP.ConstraintType(i)),LP.q(i),LP.p(i)); 
  }
  glp_add_cols(lp,LP.A.n);
  for(int i=0;i<LP.A.n;i++) {
    glp_set_col_bnds(lp,i+1,BoundTypeToGLP(LP.VariableType(i)),LP.l(i),LP.u(i)); 
  }
  for(int i=0;i<LP.A.n;i++)
    glp_set_obj_coef(lp,i+1,LP.c(i));

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
    glp_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
  }
}

void GLPKInterface::Set(const LinearProgram_Sparse& LP)
{
  SafeDeleteProc(lp,glp_delete_prob);
  lp = glp_create_prob();
  if(LP.minimize) glp_set_obj_dir(lp,GLP_MIN);
  else glp_set_obj_dir(lp,GLP_MAX);

  glp_add_rows(lp,LP.A.m);
  for(int i=0;i<LP.A.m;i++) {
    glp_set_row_bnds(lp,i+1,BoundTypeToGLP(LP.ConstraintType(i)),LP.q(i),LP.p(i)); 
  }
  glp_add_cols(lp,LP.A.n);
  for(int i=0;i<LP.A.n;i++) {
    glp_set_col_bnds(lp,i+1,BoundTypeToGLP(LP.VariableType(i)),LP.l(i),LP.u(i)); 
  }
  for(int i=0;i<LP.A.n;i++)
    glp_set_obj_coef(lp,i+1,LP.c(i));

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
    glp_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
  }
}

void GLPKInterface::Clear()
{
  SafeDeleteProc(lp,glp_delete_prob);
}

void GLPKInterface::Create(int m,int n)
{
  SafeDeleteProc(lp,glp_delete_prob);
  lp = glp_create_prob();
  glp_add_rows(lp,m);
  glp_add_cols(lp,n);
}

void GLPKInterface::SetObjective(const Vector& obj,bool minimize)
{
  for(int i=0;i<obj.n;i++)
    glp_set_obj_coef(lp,i+1,obj(i));
  if(minimize) glp_set_obj_dir(lp,GLP_MIN);
  else glp_set_obj_dir(lp,GLP_MAX);
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
  glp_set_mat_row(lp,i+1,nnz,&itemp[0],temp);
}

void GLPKInterface::SetRowBounds(int i,Real low,Real high)
{
  glp_set_row_bnds(lp,i+1,BoundType(low,high),low,high); 
}

void GLPKInterface::SetVariableBounds(int j,Real low,Real high)
{
  glp_set_col_bnds(lp,j+1,BoundType(low,high),low,high); 
}


void GLPKInterface::SetRowBasic(int i)
{
  glp_set_row_stat(lp,i+1,GLP_BS);
}

bool GLPKInterface::GetRowBasic(int i)
{
  return glp_get_row_stat(lp,i+1)==GLP_BS;
}

double GLPKInterface::GetRowDual(int i){
	return glp_get_row_dual(lp, i+1);
}

double GLPKInterface::GetVariableDual(int j){
	return glp_get_col_dual(lp, j+1);
}

void GLPKInterface::SetRowNonBasic(int i,bool upper)
{
  if(upper) glp_set_row_stat(lp,i+1,GLP_NU);
  else glp_set_row_stat(lp,i+1,GLP_NL);
}

void GLPKInterface::SetVariableBasic(int j)
{
  glp_set_col_stat(lp,j+1,GLP_BS);
}

bool GLPKInterface::GetVariableBasic(int j)
{
  return glp_get_col_stat(lp,j+1)==GLP_BS;
}

void GLPKInterface::SetVariableNonBasic(int j,bool upper)
{
  if(upper) glp_set_col_stat(lp,j+1,GLP_NU);
  else glp_set_col_stat(lp,j+1,GLP_NL);
}


int my_gglp_fault_handler(void* info,const char* msg)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"GLPK error message "<<msg);
  //LOG4CXX_ERROR(KrisLibrary::logger(),"GLPK fatal error "<<msg);
  //LOG4CXX_INFO(KrisLibrary::logger(),"jumping...\n");
  //throw(std::runtime_error(msg));
  /*
  LOG4CXX_ERROR(KrisLibrary::logger(),"GLPK fatal error, dumping!\n");
  GLP* lp=(GLP*)info;
  glp_write_cpxlp(lp,"temp_lp.txt");
  */
  return 0;
}

void my_gglp_fault_handler2(void* info)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"GLPK error, quitting\n");
}


struct GLPKInterruptHandler : public SignalHandler
{
public:
  GLPKInterruptHandler(GLPKInterface* _glpk)
    :glpk(_glpk)
  {}

  virtual void OnRaise(int signum) 
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Interrupt called during GLPK solve... possible infinite loop\n");
    glp_prob* lp=glpk->lp;
#if GLP_MAJOR_VERSION > 4 || (GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION >= 40)
    glp_write_lp(lp,NULL,"temp_lp.txt");
#endif
    throw(std::runtime_error("Interrupt called during GLPK solve"));
    //exit(-1);
  }

  GLPKInterface* glpk;
};

LinearProgram::Result GLPKInterface::Solve(Vector& xopt)
{
  assert(lp != NULL);
  //glp_write_cpxlp(lp,"temp_lp.txt");
  //glp_print_prob(lp,"temp_lp.txt");
#if GLP_MINOR_VERSION >= 43
  glp_error_hook(my_gglp_fault_handler2,0);
#endif

  glp_smcp params;
  glp_init_smcp(&params);
  params.msg_lev = GLP_MSG_ERR;
  params.presolve = GLP_OFF;

  GLPKInterruptHandler handler(this);
  handler.SetCurrent(SIGINT);
  //handler.SetCurrent(SIGABRT);
  int res;
  try {
    res=glp_simplex(lp,&params);
  }
  catch(const std::exception& e) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"GLPK internal error: ");
    LOG4CXX_INFO(KrisLibrary::logger(),e.what());
    return LinearProgram::Error;
  }
  catch (...) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown error occurred\n");
    return LinearProgram::Error;
  }
#if GLP_MINOR_VERSION >= 43
  glp_error_hook(0,0);
#endif
  handler.UnsetCurrent(SIGINT);
  switch(res) {
  case 0:
    break;
  case GLP_EFAIL:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in matrix construction!");
    return LinearProgram::Error;
  case GLP_EOBJLL:
    LOG4CXX_INFO(KrisLibrary::logger(),"Objective reached lower limit!");
    return LinearProgram::Error;
  case GLP_EOBJUL:
    LOG4CXX_INFO(KrisLibrary::logger(),"Objective reached upper limit!");
    return LinearProgram::Error;
  case GLP_ENOPFS:
    LOG4CXX_INFO(KrisLibrary::logger(),"Linear program has no primary feasible solution!");
    return LinearProgram::Infeasible;
  case GLP_ENODFS:
    LOG4CXX_INFO(KrisLibrary::logger(),"Linear program has no dual feasible solution!");
    return LinearProgram::Infeasible;
  case GLP_EITLIM:
    LOG4CXX_INFO(KrisLibrary::logger(),"Max iterations reached!");
    return LinearProgram::Error;
  case GLP_ETMLIM:
    LOG4CXX_INFO(KrisLibrary::logger(),"Time limit reached!");
    return LinearProgram::Error;
  case GLP_ESING:
    LOG4CXX_INFO(KrisLibrary::logger(),"Singularity reached!");
    return LinearProgram::Error;
  default:
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown error");
	  return LinearProgram::Error;
  }

  int stat=glp_get_status(lp);
  int n=glp_get_num_cols(lp);
  xopt.resize(n);
  for(int i=0;i<n;i++)
    xopt(i) = (Real)glp_get_col_prim(lp,i+1);

  switch(stat) {
  case GLP_OPT:
  case GLP_FEAS:
    return LinearProgram::Feasible;
  case GLP_INFEAS:
  case GLP_NOFEAS:
    return LinearProgram::Infeasible;
  case GLP_UNBND:
    return LinearProgram::Unbounded;
  case GLP_UNDEF:
    LOG4CXX_INFO(KrisLibrary::logger(),"Solution is undefined!");
    return LinearProgram::Error;
  default:
    LOG4CXX_INFO(KrisLibrary::logger(),"Shouldn't get here!");
    return LinearProgram::Error;
  }
}

bool GLPKInterface::Enabled() { return true; }

#else

#include <iostream>

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
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::Set(const LinearProgram_Sparse& LP)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::Create(int m,int n)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::Clear()
{
}

void GLPKInterface::SetRow(int i,const Vector& Ai)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::SetRowBounds(int i,Real low,Real high)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::SetVariableBounds(int j,Real low,Real high)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::SetRowBasic(int i)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

bool GLPKInterface::GetRowBasic(int i)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
  return false;
}

double GLPKInterface::GetRowDual(int i){
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
  return 0;
}

void GLPKInterface::SetRowNonBasic(int i,bool upper)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

void GLPKInterface::SetVariableBasic(int j)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

bool GLPKInterface::GetVariableBasic(int i)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
  return false;
}

void GLPKInterface::SetVariableNonBasic(int j,bool upper)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}


void GLPKInterface::SetObjective(const Vector& c,bool minimize)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

LinearProgram::Result GLPKInterface::Solve(Vector& xopt)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
  return LinearProgram::Error;
}

bool GLPKInterface::Enabled() { return false; }

void GLPKInterface::SelfTest()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLPK not defined");
}

#endif //HAVE_GLPK

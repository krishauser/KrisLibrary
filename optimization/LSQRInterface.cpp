#include "LSQRInterface.h"
#include "lsqr.h"
using namespace Optimization;
using namespace std;

struct SparseMatrixMultiplier : public lsqr_func
{
  SparseMatrixMultiplier(const dSparseMatrix_RM& _A) :A(_A) { }

  /* compute  y = y + A*x*/
  virtual void MatrixVectorProduct (const dVector& x, dVector& y)
  {
    A.madd(x,y);
  }
  /* compute  x = x + At*y*/
  virtual void MatrixTransposeVectorProduct (dVector& x, const dVector& y)
  {
    A.maddTranspose(y,x);
  }

  const dSparseMatrix_RM& A;
};

LSQRInterface::LSQRInterface()
  :dampValue(0),relError(0),condLimit(0),maxIters(0),verbose(1)
{}

bool LSQRInterface::Solve(const SparseMatrix& A,const Vector& b)
{
  dSparseMatrix_RM dA; dA.copy(A);
  SparseMatrixMultiplier func(dA);
  lsqr_input input;
  lsqr_work work;
  lsqr_output output;
  input.num_rows = A.m;
  input.num_cols = A.n;
  input.damp_val = dampValue;
  input.rel_mat_err = input.rel_rhs_err = relError;
  input.cond_lim = condLimit;
  if(maxIters)  input.max_iter = maxIters;
  else          input.max_iter = A.n*4;
  switch(verbose) {
  case 1: input.lsqr_fp_out = stdout;  break;
  case 2: input.lsqr_fp_out = stderr;  break;
  default: input.lsqr_fp_out = NULL;   break;
  }
  input.rhs.copy(b);
  if(x0.n == 0) input.sol.resize(A.n,Zero);
  else if(x0.n == A.n) input.sol.copy(x0);
  else {
    cerr<<"Initial guess doesn't have correct dimensions"<<endl;
    cerr<<"Using zeros for initial guess"<<endl;
    input.sol.resize(A.n,Zero);
  }

  lsqr(input,output,work,func);

  numIters = output.num_iters;
  condEstA = output.mat_cond_num;
  residualNorm = output.resid_norm;
  x.copy(output.sol);
  stdErr.copy(output.std_err);
  
  switch(output.term_flag) {
  case lsqr_output::X0Exact:
    if(verbose) cout<<"LSQR: X0 is the exact solution!"<<endl;
    break;
  case lsqr_output::ExactSolutionRelMat:
    if(verbose) cout<<"LSQR: Solved approximately the exact solution"<<endl;
    break;
  case lsqr_output::LSSolutionRelMat:
    if(verbose) cout<<"LSQR: Solved approximately a least-squares solution"<<endl;
    break;
  case lsqr_output::IllConditioned:
    if(verbose) cout<<"LSQR: The matrix is probably ill-conditioned"<<endl;
    return false;
  case lsqr_output::ExactSolution:
    if(verbose) cout<<"LSQR: Solved the exact solution"<<endl;
    break;
  case lsqr_output::LSSolution:
    if(verbose) cout<<"LSQR: Solved the least-squares solution"<<endl;
    break;
  case lsqr_output::ConditionError:
    if(verbose) cout<<"LSQR: The condition number became very large"<<endl;
    return false;
  case lsqr_output::MaxItersReached:
    if(verbose) cout<<"LSQR: The max # of iterations has been reached, residual "<<residualNorm<<endl;
    return false;
  default:
    cerr<<"LSQR: Unknown return value "<<output.term_flag<<endl;
    return false;
  }
  return true;
}

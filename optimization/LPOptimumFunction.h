#ifndef LP_OPTIMUM_FUNCTION
#define LP_OPTIMUM_FUNCTION

#include "LinearProgram.h"
#include "GLPKInterface.h"
#include <KrisLibrary/math/function.h>

namespace Optimization {
  using namespace Math;

/** @brief A scalar field v(x) = min_y c(x)^T y s.t. q(x) <= A(x)y <= p(x),
 * l(x) <= y <= u(x) where c,A,q,p,l, and u are smooth functions.
 *
 * Subclasses must overload UpdateLP and LPJacobian[Z] if Z depends on x.
 * For faster updating, use InitLP to set up all constant members of the LP
 * and in UpdateLP, only change the members that depend on x.
 */
class LPOptimumFunction : public ScalarFieldFunction
{
 public:
  LPOptimumFunction();
  //subclasses overload these
  virtual void InitLP(const Vector& x) {}
  virtual void UpdateLP(const Vector& x) { FatalError("Update not defined in subclas of LPOptimumFunction"); }
  virtual bool LPJacobianC(const Vector& x,Matrix& Jc) { return false; }
  virtual bool LPJacobianC(const Vector& x,SparseMatrix& Jc) { return false; }
  //derivative of row_i(A) w.r.t x 
  virtual bool LPJacobianA_i(const Vector& x,int i,Matrix& JAi) { return false; }
  virtual bool LPJacobianA_i(const Vector& x,int i,SparseMatrix& JAi) { return false; }
  virtual bool LPJacobianP_i(const Vector& x,int i,Vector& Jpi) { return false; }
  virtual bool LPJacobianQ_i(const Vector& x,int i,Vector& Jqi) { return false; }
  virtual bool LPJacobianL_j(const Vector& x,int j,Vector& Jlj) { return false; }
  virtual bool LPJacobianU_j(const Vector& x,int j,Vector& Juj) { return false; }

  ///Subclasses may overload this for a sparse update -- may also help
  ///with warm start.
  virtual void UpdateGLPK(const Vector& x);

  virtual std::string Label() const { return "LPOptimum"; }
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);

  bool initialized;
  bool sparse;
  LinearProgram lp;
  LinearProgram_Sparse lps;
  GLPKInterface glpk;

  LinearProgram::Result solveResult;
  Vector yopt;
};

/** @brief A vector field v(x) = arg min_y c(x)^T y s.t. q(x) <= A(x)y <= p(x),
 * l(x) <= y <= u(x) where c,A,q,p,l, and u are smooth functions.
 *
 * Subclasses must overload UpdateLP and LPJacobian[Z] if Z depends on x.
 * For faster updating, use InitLP to set up all constant members of the LP
 * and in UpdateLP, only change the members that depend on x.
 */
class LPOptimizerFunction : public VectorFieldFunction
{
 public:
  LPOptimizerFunction();
  virtual int NumDimensions() const;
  //subclasses overload these
  virtual void InitLP(const Vector& x) {}
  virtual void UpdateLP(const Vector& x) { FatalError("Update not defined in subclas of LPOptimumFunction"); }
  virtual bool LPJacobianA_i(const Vector& x,int i,Matrix& JAi) { return false; }
  virtual bool LPJacobianA_i(const Vector& x,int i,SparseMatrix& JAi) { return false; }
  virtual bool LPJacobianP_i(const Vector& x,int i,Vector& Jpi) { return false; }
  virtual bool LPJacobianQ_i(const Vector& x,int i,Vector& Jqi) { return false; }
  virtual bool LPJacobianL_j(const Vector& x,int j,Vector& Jlj) { return false; }
  virtual bool LPJacobianU_j(const Vector& x,int j,Vector& Juj) { return false; }

  ///Subclasses may overload this for a sparse update -- may also help
  ///with warm start.
  virtual void UpdateGLPK(const Vector& x);

  virtual std::string Label() const { return "LPOptimum"; }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x,Vector& v);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);

  bool initialized;
  bool sparse;
  LinearProgram lp;
  LinearProgram_Sparse lps;
  GLPKInterface glpk;

  LinearProgram::Result solveResult;
  Vector yopt;
};


} //namespace Optimization

#endif

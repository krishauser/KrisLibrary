#ifndef OPTIMIZATION_GLPK_INTERFACE_H
#define OPTIMIZATION_GLPK_INTERFACE_H

#include "LinearProgram.h"
#if HAVE_GLPK

#include <glpk.h>

#else
#define glp_prob void
#endif

namespace Optimization {

/** @ingroup Optimization
 * @brief An interface to the GLPK linear program solver.  Activated with the
 * HAVE_GLPK preprocessor define.
 */
struct GLPKInterface
{
  GLPKInterface();
  ~GLPKInterface();
  //easiest interface
  void Set(const LinearProgram& LP);
  void Set(const LinearProgram_Sparse& LP);
  LinearProgram::Result Solve(Vector& xopt);
  //low level commands
  void Create(int m,int n);
  void Clear();
  void SetObjective(const Vector& c,bool minimize=true);
  void SetRow(int i,const Vector& Ai);
  void SetRowBounds(int i,Real low,Real high);		    
  void SetVariableBounds(int j,Real low,Real high);

  //warm starting the solver
  void SetRowBasic(int i);  //inactive
  void SetRowNonBasic(int i,bool upper=false);  //active
  void SetVariableBasic(int i);  //inactive
  void SetVariableNonBasic(int i,bool upper=false);  //active
  bool GetRowBasic(int i);  //inactive
  bool GetVariableBasic(int i);  //inactive
  double GetRowDual(int i);
  double GetVariableDual(int j);

  static bool Enabled();
  static void SelfTest();

  glp_prob* lp;
};

} //namespace Optimization

#endif

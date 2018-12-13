#ifndef OPTIMIZATION_LCP_H
#define OPTIMIZATION_LCP_H

#include <KrisLibrary/math/matrix.h>
#include <vector>
#include <string>


namespace Optimization {

  using namespace Math;

/** @ingroup Optimization
 * @brief Solves a linear complementarity problem
 *
 * @verbatim
 * w = Mz + q
 * w,z >= 0
 * wk*zk = 0 for all indices k
 * @endverbatim
 *
 * Convergence is assured if M is copositive,
 * that is, x^t M x >= 0 if x >= 0, 
 * and satisfies (M+M^t)x = 0 for all x that satisfy 
 * x^t M x = 0 and M x >= 0
 */
class LemkeLCP
{
public:
  LemkeLCP(const Matrix& M,const Vector& q);
  bool Solve();
  void GetW(Vector& w) const;
  void GetZ(Vector& z) const;
  void Print(std::ostream& out) const;

private:
  //returns the index of z0 (-1 if it's initially feasible)
  int InitialPivot();
  int PickPivot(int enter) const;
  bool Pivot(int enter,int leave);

  const static int z0=-2;
  const static int constant=-1;
  bool IsVarW(int var) const { return 0 <= var && var < dictionary.m; }
  bool IsVarZ(int var) const { return var >= dictionary.m; }
  int WToZ(int var) const { return var+dictionary.m; }
  int ZToW(int var) const { return var-dictionary.m; }
  int VarToW(int var) const { return var; }
  int VarToZ(int var) const { return var-dictionary.m; }
  int WToVar(int index) const { return index; }
  int ZToVar(int index) const { return index+dictionary.m; }
  std::string VarName(int var) const;

 public:
  int verbose;

 private:
  //column 0 is the constant term
  Matrix dictionary;
  std::vector<int> basic,nonbasic;
};

bool IterativeLCP(const Matrix& M,const Vector& q,Vector& w,Vector& z,int& maxIters,Real tol);

} //namespace Optimization

#endif

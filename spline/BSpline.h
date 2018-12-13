#ifndef SPLINE_BSPLINE_H
#define SPLINE_BSPLINE_H

#include <KrisLibrary/math/sparsevector.h>
#include <KrisLibrary/math/sparsematrix.h>
#include <vector>

namespace Spline {

using namespace Math;

/** @ingroup Spline
 * @brief Defines the B-spline basis for a given knot vector, 
 * number of control points.
 *
 * Let m = knots.size()-1, n=numControlPoints-1.
 * The degree of the b-spline is p=m-n-1.
 *
 * Control points are x0,...,xn, the spline x(t) is defined for
 * t in [knots[p],knots[m-p])
 */
struct BSplineBasis
{
  ///Sets the knots to be uniformly spaced with valid t in [0,h*(numCps-degree)]
  void SetUniformKnots(size_t numCps,size_t degree,Real h=One);

  ///Sets the knots to be closed (interpolate endpoints) with t in [0,h*(numCps-degree)]
  void SetClosedKnots(size_t numCps,size_t degree,Real h=One);

  ///Returns the degree p
  int Degree() const { return (int)knots.size()-numControlPoints-1; }

  ///Returns the range of basis functions with support over the interval
  ///knots[k]<=t<knots[k+1]
  std::pair<int,int> Support(int k) const { return std::pair<int,int>(k-Degree(),k); }
  ///Returns the range of intervals knots[first]<=t<=knots[second+1] over
  ///which basis function k is defined
  std::pair<int,int> Domain(int k) const { return std::pair<int,int>(k,k+Degree()); }

  Real StartTime() const { return knots[Degree()]; }
  Real EndTime() const { return knots[knots.size()-Degree()-1]; }

  ///Returns the furthest knot k such that knots[k]<=t<knots[k+1]
  int GetKnot(Real t) const;

  /** @brief Returns a vector of length n+1 whose elements are the
   * spline coefficients x(t) = sum_i basis(i)*xi
   */
  void Evaluate(Real t,SparseVector& basis) const;

  /** @brief Returns a matrix B of dimension n+1 x p+1 whose elements give
   * the spline coefficients for knots[k] <= t <= knots[k+1]
   *
   * The spline is given as 
   * x(t) = X B T, where T=[1, t, t^2, ... t^p]
   * and X = [x0 | x1 | ... | xn]
   */
  void Evaluate(int k,SparseMatrix& basis) const;

  /** @brief Returns a vector of length n+1 whose elements are the
   * spline derivative coefficients x'(t) = sum_i basis(i)*xi
   */
  void Deriv(Real t,SparseVector& basis) const;

  /** @brief Returns a vector of length n+1 whose elements are the
   * spline acceleration coefficients x''(t) = sum_i basis(i)*xi
   */
  void Deriv2(Real t,SparseVector& basis) const;

  ///Saves some computation from calling Evaluate(),Deriv(), and Deriv2()
  void EvaluateWithDerivs(Real t,SparseVector& b,SparseVector& db) const;
  void EvaluateWithDerivs(Real t,SparseVector& b,SparseVector& db,SparseVector& ddb) const;

  ///If n=db.size(), evaluates the i'th derivative in db[i]
  //(db[0] contains the regular spline coefficients)
  void EvaluateWithDerivs(Real t,std::vector<SparseVector >& db) const;

  bool IsValid() const;

  int numControlPoints;
  std::vector<Real> knots;
};

struct BSpline
{
  BSpline();
  int Degree() const { return basis.Degree(); }
  void SetConstant(const Vector& value);
  void Evaluate(Real t,Vector& value) const;
  void Deriv(Real t,Vector& value) const;
  void Deriv(Real t,int k,Vector& value) const;
  bool IsValid() const;

  BSplineBasis basis;
  std::vector<Vector> cps;
};

/** Cox-DeBoor recursion formula for spline coefficients:
 * N(i,0) = 1 if u[i]<=t<u[i+1] and u[i] < u[i+1]
 *          0 otherwise
 * N(i,p) = N(i,p-1)*(t-u[i])/(u[i+1]-u[i])
 *        + N(i+1,p-1)*(u[i+p+1]-t)/(u[i+p+1]-u[i+1])
 * Only basis functions that are nonzero for u[i]<=t<u[i+1] are
 *   N(i-p,p)...N(i,p)
 */

/** @brief Recursively evaluates the basis function N(i,p)(t) for the
 * given knot points u.
 *
 * Compared to the other functions, this is fairly slow.  Running tim
 * is exponential in p.
 */
Real CoxDeBoor(int i,int p,Real t,const std::vector<Real>& u);

/** @brief Calculates the Cox-DeBoor basis coefficients
 * N(i,p)...N(i+p,p) = N[0]...N[p].
 *
 * Requires u[i+p]<=t<=u[i+p+1].  N must have size p+1.
 *
 * Running time is O(p^2)
 */
void CoxDeBoor(int i,int p,Real t,const std::vector<Real>& u,Real N[]);

///Same as above, but also produces coefficients for the spline derivative
///dN=N'.
void CoxDeBoorDeriv(int i,int p,Real t,const std::vector<Real>& u,Real N[],Real dN[]);

///Same as above but includes 2nd derivative ddN=N''
void CoxDeBoorDeriv2(int i,int p,Real t,const std::vector<Real>& u,Real N[],Real dN[],Real ddN[]);

/** @brief Same as above but includes n derivatives.  The n'th row
 * in the n+1 x p+1 matrix N contains the coefficients for the n'th derivative
 */
void CoxDeBoorDerivN(int i,int p,Real t,int n,const std::vector<Real>& u,Real** N);

/** @brief Returns the spline basis B for u[base] <= t < u[base+1] such that
 * N(i,p)(t) = sum(k=0...p) (B[k]*t^k) 
 * 
 * B must have size p+1.
 */
void CoxDeBoorBasis(int base,int i,int p,const std::vector<Real>& u,Real B[]);

/** @brief Returns the basis B[i,k] such that for u[base+p]<=t<u[base+p+1]
 * N(base+i,p) = sum(k=0...p) B[i,k]*t^k
 *
 * B must be a matrix of size p+1 x p+1
 */
void CoxDeBoorBasis2(int base,int p,const std::vector<Real>& u,Real** B);

std::istream& operator >> (std::istream& in,BSpline& spline);
std::ostream& operator << (std::ostream& in,const BSpline& spline);

} // namespace Spline

#endif

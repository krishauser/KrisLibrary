#ifndef GEOMETRY_LRS_INTERFACE_H
#define GEOMETRY_LRS_INTERFACE_H

#include <math/matrix.h>
#include <structs/array2d.h>
#include <vector>
using namespace Math;

template <class type>
struct Fraction
{
  type num,den;
};

/** @ingroup Geometry
 * @brief An interface to the LRS library.  Activated with the HAVE_LRS=1
 * preprocessor define.
 */
class LRSInterface
{
public:
  typedef Fraction<long> Value;
  typedef std::vector<Value> ValueVector;

  LRSInterface();
  //Halfspace to vertex/ray representation
  //A is a set of halfspace constraints a0 + a1*x1 + ... an*xn >= 0
  //pts is a list of ValueVectors, where p0 = 1 if it's a point, 0 if it's a ray
  void SolveH2VProblem(const Array2D<Value>& A, std::vector<ValueVector>& pts);
  void SolveH2VProblem(const Array2D<Value>& Aineq, const Array2D<Value>& Aeq,
		       std::vector<ValueVector>& pts);
  void SolveH2VProblem(const Matrix& A, std::vector<Vector>& pts);
  void SolveH2VProblem(const Matrix& Aineq, const Matrix& Aeq, std::vector<Vector>& pts);
  
  static bool initialized;
  bool nonnegative;
  long precisionL;
};

#endif

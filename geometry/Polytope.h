#ifndef GEOMETRY_POLYTOPE_H
#define GEOMETRY_POLYTOPE_H

#include <math/matrix.h>
#include <vector>
using namespace Math;

namespace Geometry {

class HPolytope;
class VPolytope;

/** @ingroup Geometry
 * @brief The halfspace-representation of a polytope, with
 * inequalities/equalities Ax+b >= 0
 */
class HPolytope
{
public:
	inline int Dimension() const { return A.n; }
	inline int NumConstraints() const { return A.m; }
	inline bool IsValid() const { return (A.n == b.n) && (equality.empty() || (int)equality.size()==A.n); }
	bool Contains(const Vector& x) const;
	void GetVPolytope(VPolytope&);

	inline bool HasEqualities() const { return !equality.empty(); }
	inline bool Inequality(int i) const { return equality.empty() || !equality[i]; }
	inline bool Equality(int i) const { return !equality.empty() && equality[i]; }
	int NumEqualities() const;
	int NumInequalities() const;
  void Print(std::ostream& out) const;

	Matrix A;
	Vector b;
	std::vector<bool> equality;		//flag to indicate if the given constraint is equality rather than inequality
	bool positiveX;                //flag to include x>=0 constraint
};

/** @ingroup Geometry
 * @brief The vertex-representation of a polytope.  The polyhedron is
 * the convex hull of the points and rays.
 *
 * The points matrix stores the points in its rows.
 * The rays matrix stores the rays in its rows.
 */
class VPolytope
{
public:
	inline int Dimension() const { return points.n; }
	inline int NumPoints() const { return points.m; }
	inline int NumRays() const { return rays.m; }
	inline bool IsValid() const { return points.n == rays.n; }
	Real Maximum(const Vector& d) const;
	Real Minimum(const Vector& d) const;

	Matrix points;
	Matrix rays;
};

} // namespace Geometry

#endif

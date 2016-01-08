#ifndef SPLINE_BASIS_H
#define SPLINE_BASIS_H

#include <KrisLibrary/math/math.h>
using namespace Math;

/*
 * SplineBasis (abstract)
 * A set of spline basis functions bi(u)
 * Note: The order of the basis functions varies for each basis.
 */
struct SplineBasis
{
	SplineBasis() {}
	virtual ~SplineBasis() {}

	virtual int Order() const = 0;

	virtual Real RangeMin() const { return Zero; }
	virtual Real RangeMax() const { return One; }

	//vals must be at least of size Order()+1
	virtual void EvalBasis(Real u, Real* vals) const = 0;
	virtual void EvalBasisDeriv(Real u, int deriv, Real* vals) const = 0;
};

/*
 * LinearSplineBasis
 * A linear basis function.
 * Interpolates x0 and x1
 */
struct LinearSplineBasis : public SplineBasis
{
	virtual int Order() const { return 1; }

	void EvalBasis(Real u, Real* vals) const;
	void EvalBasisDeriv(Real u, int deriv, Real* vals) const;
};


/*
 * CubicSplineBasis (abstract)
 * A basic arbitrary cubic spline basis function, given by a matrix of coefficients.
 * bi(u) = B[i][3]*u^3 + B[i][2]*u^2 + B[i][1]*u + B[i][0]
 * BasisCoeffs() must be defined.
 */
struct CubicSplineBasis : public SplineBasis
{
	virtual int Order() const { return 3; }
	void EvalBasis(Real u, Real* vals) const;
	void EvalBasisDeriv(Real u, int deriv, Real* vals) const;

	virtual const Real* BasisCoeffs(int i) const = 0;
};


/*
 * HermiteSplineBasis
 * A cubic hermite spline basis function.
 * The spline is b0*p0 + b1*p1 + b2*t0 + b3*t1, where pi are sample points and ti are tangents
 */
struct HermiteSplineBasis : public CubicSplineBasis
{
	virtual const Real* BasisCoeffs(int i) const;
};

/*
 * CardinalSplineBasis
 * A cardinal basis function 
 * The spline is b0*p-1 + b1*p0 + b2*p1 + b3*p2, where pi are interpolant points
 */
struct CardinalSplineBasis : public CubicSplineBasis
{
	virtual const Real* BasisCoeffs(int i) const { return B[i]; }

	void SetTension(Real tension);

	Real B[4][4];
};

/*
 * BezierCubicSplineBasis
 * A bezier basis function 
 * The spline is b0*p0 + b1*c0 + b2*c1 + b3*p1, where pi are interpolant points and ci are control points
 */
struct BezierCubicSplineBasis : public CubicSplineBasis
{
	virtual const Real* BasisCoeffs(int i) const;
};

#endif

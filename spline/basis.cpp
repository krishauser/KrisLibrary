#include "basis.h"

typedef Real realMat44 [4][4];

const Real Three = 3.0;
const Real Four = 4.0;
const Real Six = 6.0;

const realMat44 hermiteBasis = {
	{ 1, 0,-3, 2},		//x0
	{ 0, 0, 3,-2},		//x1
	{ 0, 1,-2, 1},		//t0
	{ 0, 0,-1, 1},		//t1
};

inline void cardinalBasis(Real s, realMat44 C)
{
	C[0][3]=-s;			C[0][2]=s*Two;		C[0][1] = -s;	C[0][0] = Zero;		//x-1
	C[1][3]=(Two-s);	C[1][2]=s-Three;	C[1][1] = Zero;	C[1][0] = One;		//x0
	C[2][3]=(s-Two);	C[2][2]=Three-Two*s;C[2][1] = s;	C[2][0] = Zero;		//x1
	C[3][3]=s;			C[3][2]=-s;			C[3][1] = Zero; C[3][0] = Zero;		//x2
}

const realMat44 cubicBezierBasis = {
	{ 1,-3, 3,-1},
	{ 0, 3,-6, 3},
	{ 0, 0, 3,-3},
	{ 0, 0, 0, 1},
};

inline void hermite_coeffs(Real t, Real H [])
{
	Real t2 = t*t, t3 = t2*t;

	H[1]=-Two*t3 + Three*t2;
	H[0]=-H[1] + One;
	H[2]=t3 - Two*t2 + t;
	H[3]=t3 - t2;
}

inline void hermite_tangent_coeffs(Real t, Real T[])
{
	Real t2 = t*t;

	T[0]=Six*t2 - Six*t;
	T[1]= -T[0];
	T[2]=Three*t2 - Four*t + One;
	T[3]=Three*t2 - Two*t;
}

inline void cardinal_coeffs(Real t, Real s, Real C[])
{
	Real t2 = t*t, t3 = t2*t;

	C[0]=s*(-t3 + Two*t2 - t);
	C[1]=(Two-s)*t3 + (s-Three)*t2 + One;
	C[2]=(s-Two)*t3 + (Three-Two*s)*t2 + s*t;
	C[3]=s*(t3 - t2);
}

inline void cardinal_tangent_coeffs(Real t, Real s, Real T[])
{
	Real t2 = t*t;

	T[0]=s*(-Three*t2 + Four*t - One);
	T[1]=(Six-Three*s)*t2 + (Two*s-Six)*t;
	T[2]=(Three*s-Six)*t2 + (Six-Four*s)*t + s;
	T[3]=s*(Three*t2 - Two*t);
}

inline void cubic_bezier_coeffs(Real t, Real C[])
{
	Real t2 = t*t, t3 = t2*t;

	C[0]=One-Three*t+Three*t2-t3;
	C[1]=Three*(t-Two*t2+t3);
	C[2]=Three*(t2-t3);
	C[3]=t3;
}


void LinearSplineBasis::EvalBasis(Real u, Real* vals) const
{
	vals[0] = u;
	vals[1] = One - u;
}

void LinearSplineBasis::EvalBasisDeriv(Real u, int deriv, Real* vals) const
{
	switch(deriv)
	{
	case 0:
		EvalBasis(u,vals);
		break;
	case 1:
		vals[0] = One;
		vals[1] = -One;
		break;
	default:
		vals[0] = vals[1] = Zero;
	}
}


void CubicSplineBasis::EvalBasis(Real u, Real* vals) const
{
	Real u2 = u*u;
	Real u3 = u*u2;

	for(int i=0; i<=3; i++)
	{
		const Real* b = BasisCoeffs(i);
		vals[i] = u3*b[3] + u2*b[2] + u*b[1] + b[0];
	}
}

void CubicSplineBasis::EvalBasisDeriv(Real u, int deriv, Real* vals) const
{
	Real u2 = u*u;
	int i;
	//Real u3 = u*u2;

	switch(deriv)
	{
	case 0:
		EvalBasis(u,vals);
		break;
	case 1:
		for(i=0; i<=3; i++)
		{
			const Real* b = BasisCoeffs(i);
			vals[i] = Three*u2*b[3] + Two*u*b[2] + b[1];
		}
		break;
	case 2:
		for(i=0; i<=3; i++)
		{
			const Real* b = BasisCoeffs(i);
			vals[i] = Six*u*b[3] + Two*b[2];
		}
		break;
	case 3:
		for(i=0; i<=3; i++)
		{
			const Real* b = BasisCoeffs(i);
			vals[i] = Six*b[3];
		}
		break;
	default:
		for(int i=0; i<=3; i++)
			vals[i] = Zero;
	}
}



const Real* HermiteSplineBasis::BasisCoeffs(int i) const
{
	return hermiteBasis[i];
};

void CardinalSplineBasis::SetTension(Real tension)
{
	cardinalBasis(tension, B);
};

const Real* BezierCubicSplineBasis::BasisCoeffs(int i) const
{
	return cubicBezierBasis[i];
}

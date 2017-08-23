#include "LinearAlgebra.h"
#include "misc.h"
#include <math/SVDecomposition.h>
#include <math/misc.h>

namespace Math3D {

void Copy(Real v,Vector& mat)
{
  mat.resize(1);
  mat(0)=v;
}

void Copy(const Vector2& v,Vector& mat)
{
  mat.resize(2);
  v.get(mat(0),mat(1));
}

void Copy(const Vector3& v,Vector& mat)
{
  mat.resize(3);
  v.get(mat(0),mat(1),mat(2));
}

void Copy(const Vector4& v,Vector& mat)
{
  mat.resize(4);
  v.get(mat(0),mat(1),mat(2),mat(3));
}

void Copy(const Vector& vec,Real& v)
{
  Assert(vec.n==1);
  v=vec(0);
}

void Copy(const Vector& vec,Vector2& v)
{
  Assert(vec.n==2);
  v.set(vec(0),vec(1));
}

void Copy(const Vector& vec,Vector3& v)
{
  Assert(vec.n==3);
  v.set(vec(0),vec(1),vec(2));
}

void Copy(const Vector& vec,Vector4& v)
{
  Assert(vec.n==4);
  v.set(vec(0),vec(1),vec(2),vec(3));
}


void Copy(Real m,Matrix& mat)
{
  mat.resize(1,1);
  mat(0,0)=m;
}

void Copy(const Matrix2& m,Matrix& mat)
{
  mat.resize(2,2);
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++)
      mat(i,j)=m(i,j);
}

void Copy(const Matrix3& m,Matrix& mat)
{
  mat.resize(3,3);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat(i,j)=m(i,j);
}

void Copy(const Matrix4& m,Matrix& mat)
{
  mat.resize(4,4);
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      mat(i,j)=m(i,j);
}

void Copy(const Matrix& mat,Real& m)
{
  Assert(mat.m==1 && mat.n==1);
  m = mat(0,0);
}

void Copy(const Matrix& mat,Matrix2& m)
{
  Assert(mat.m==2 && mat.n==2);
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++)
      m(i,j)=mat(i,j);
}

void Copy(const Matrix& mat,Matrix3& m)
{
  Assert(mat.m==3 && mat.n==3);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      m(i,j)=mat(i,j);
}

void Copy(const Matrix& mat,Matrix4& m)
{
  Assert(mat.m==4 && mat.n==4);
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      m(i,j)=mat(i,j);
}


//TODO: somehow integrate this?
#define EPS 2.1e-16 // double precsion
#define isign(i) ((i)<0 ? (-1) : (+1)) // int sign function
#define sign(x) ((x)<0.0 ? (-1) : (+1)) // float sign function
//----------------------------------------------------------------------
// Lasv2
// SVD of a 2x2 real upper triangular matrix
// [f g] = [cu -su]*[smax 0 ]*[ cv sv]
// [0 h] [su cu] [ 0 smin] [-sv cv]
// smax is the larger singular value and smin is the smaller
//
// "smin" and "smax" are singular values
// "cv" and "sv" are the entries in the right singular vector matrix
// "cu" and "su" are the entries in the left singular vector matrix
// "f", "g", and "h" are the entries in the upper triangular matrix
//
// This code is translated from the FORTRAN code SLASV2 listed in
// Z.Bai and J.Demmel,
// "Computing the Generalized Singular Value Decomposition",
// SIAM J. Sci. Comput., Vol. 14, No. 6, pp. 1464-1486, November 1993
//-----------------------------------------------------------------------
void
Lasv2(double *smin, double *smax, double *sv, double *cv,
      double *su, double *cu, double f, double g, double h)
{
  double svt, cvt, sut, cut; // temporary sv, cv, su, and cu
  double ft = f, gt = g, ht = h; // temporary f, g, h
  double fa = fabs(f), ga = fabs(g), ha = fabs(h);
  // |f|, |g|, and |h|
  int pmax = 1, // pointer to max abs entry
    swap = 0, // is swapped
    glarge = 0, // is g very large
    tsign; // tmp sign
  double fmh, // |f| - |h|
    d, // (|f| - |h|)/|f|
    dd, // d*d
    q, // g/f
    qq, // q*q
    s, // (|f| + |h|)/|f|
    ss, // s*s
    spq, // sqrt(ss + qq)
    dpq, // sqrt(dd + qq)
    a; // (spq + dpq)/2
  double tmp, // temporaries
    tt;
  // make fa>=ha
  if (fa<ha) {
    pmax = 3;
    tmp = ft; ft = ht; ht = tmp; // swap ft and ht
    tmp = fa; fa = ha; ha = tmp; // swap fa and ha
    swap = 1;
  } // if fa<ha
  if (ga==0.0) { // diagonal
    *smin = ha;
    *smax = fa;
    cut = 1.0; sut = 0.0; // identity
    cvt = 1.0; svt = 0.0;
  } else { // not diagonal
    if (ga>fa) { // g is the largest entry
      pmax = 2;
      if ((fa/ga)<EPS) { // g is very large
	glarge = 1;
	*smax = ga; // 1 ulp
	if (ha>1.0)
	  *smin = fa/(ga/ha); // 2 ulps
	else
	  *smin = (fa/ga)*ha; // 2 ulps
	cut = 1.0; sut = ht/gt;
	cvt = 1.0; svt = ft/gt;
      } // if g large
    } // if ga>fa
    if (glarge==0) { // normal case
      fmh = fa - ha; // 1 ulp
      if (fmh==fa) // cope with infinite f or h
	d = 1.0;
      else
	d = fmh/fa; // note 0<=d<=1.0, 2 ulps
      q = gt/ft; // note |q|<1/EPS, 1 ulp
      s = 2.0 - d; // note s>=1.0, 3 ulps
      qq = q*q; ss = s*s;
      spq = sqrt(ss + qq); // note 1<=spq<=1+1/EPS, 5 ulps
      if (d==0.0)
	dpq = fabs(q); // 0 ulp
      else
	dpq = sqrt(d*d + qq); // note 0<=dpq<=1+1/EPS, 3.5 ulps
      a = 0.5*(spq + dpq); // note 1<=a<=1 + |q|, 6 ulps
      *smin = ha/a; // 7 ulps
      *smax = fa*a; // 7 ulps
      if (qq==0.0) { // qq underflow
	if (d==0.0)
	  tmp = sign(ft)*2*sign(gt);
	// 0 ulp
	else
	  tmp = gt/(sign(ft)*fmh) + q/s;
	// 6 ulps
      } else {
	tmp = (q/(spq + s) + q/(dpq + d))*(1.0 + a);
	// 17 ulps
      } // if qq
      tt = sqrt(tmp*tmp + 4.0);
      // 18.5 ulps
      cvt = 2.0/tt; // 19.5 ulps
      svt = tmp/tt; // 36.5 ulps
      cut = (cvt + svt*q)/a; // 46.5 ulps
      sut = (ht/ft)*svt/a; // 45.5 ulps
    } // if g not large
  } // if ga
  if (swap==1) {
    *cu = svt; *su = cvt;
    *cv = sut; *sv = cut;
  } else {
    *cu = cut; *su = sut;
    *cv = cvt; *sv = svt;
  } // if swap
  // correct the signs of smax and smin
  if (pmax==1) tsign = sign(*cv)*sign(*cu)*sign(f);
  if (pmax==2) tsign = sign(*sv)*sign(*cu)*sign(g);
  if (pmax==3) tsign = sign(*sv)*sign(*su)*sign(h);
  *smax = isign(tsign)*(*smax);
  *smin = isign(tsign*sign(f)*sign(h))*(*smin);
} // Lasv2


bool SVD(const Matrix2& A,Matrix2& U,Vector2& W,Matrix2& V)
{
  Matrix mA;
  Copy(A,mA);
  Real scale=mA.maxAbsElement();
  mA /= scale;
  SVDecomposition<Real> svd;
  if(!svd.set(mA)) {
    return false;
  }
  svd.sortSVs();
  Copy(svd.U,U);
  Copy(svd.V,V);
  Copy(svd.W,W);
  W *= scale;
  return true;
}

bool SVD(const Matrix3& A,Matrix3& U,Vector3& W,Matrix3& V)
{
  Matrix mA;
  Copy(A,mA);
  Real scale=mA.maxAbsElement();
  mA /= scale;
  SVDecomposition<Real> svd;
  if(!svd.set(mA)) {
    return false;
  }
  svd.sortSVs();
  Copy(svd.U,U);
  Copy(svd.V,V);
  Copy(svd.W,W);
  W *= scale;
  return true;
}

bool SVD(const Matrix4& A,Matrix4& U,Vector4& W,Matrix4& V)
{
  Matrix mA;
  Copy(A,mA);
  Real scale=mA.maxAbsElement();
  mA /= scale;
  SVDecomposition<Real> svd;
  if(!svd.set(mA)) {
    return false;
  }
  svd.sortSVs();
  Copy(svd.U,U);
  Copy(svd.V,V);
  Copy(svd.W,W);
  W *= scale;
  return true;
}


void Eigenvalues(const Matrix2& A,Complex& lambda1,Complex& lambda2)
{
  Real trace=A.trace();
  Real det=A.determinant();
  Complex temp2 = Sqr(trace) - 4.0*det;
  Complex temp = Sqrt(temp2);
  lambda1 = 0.5*(Complex(trace) + temp);
  lambda2 = 0.5*(Complex(trace) - temp);
}

bool Eigenvalues(const Matrix2& A,Real& lambda1,Real& lambda2)
{
  Real trace=A.trace();
  Real det=A.determinant();
  Real temp2 = Sqr(trace) - 4.0*det;
  if(temp2 < 0) return false;
  Real temp=Sqrt(temp2);
  lambda1 = 0.5*(trace + temp);
  lambda2 = 0.5*(trace - temp);
  return true;
}

bool Eigendecomposition(const Matrix2& A,Vector2& lambda,Matrix2& Q)
{
  if(!IsSymmetric(A)) return false;
  Matrix2 U;
  return SVD(A,U,lambda,Q);
}

bool Eigendecomposition(const Matrix3& A,Vector3& lambda,Matrix3& Q)
{
  if(!IsSymmetric(A)) return false;
  Matrix3 U;
  return SVD(A,U,lambda,Q);
}

bool Eigendecomposition(const Matrix4& A,Vector4& lambda,Matrix4& Q)
{
  if(!IsSymmetric(A)) return false;
  Matrix4 U;
  return SVD(A,U,lambda,Q);
}

} //namespace Math3D

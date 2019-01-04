#include <KrisLibrary/Logger.h>
#include "BSpline.h"
#include <math/fastarray.h>
#include <errors.h>
#include <algorithm>
#include <iostream>
using namespace std;

namespace Spline {

//Evaluates the basis function N(i,p)(t)
Real CoxDeBoor(int i,int p,Real t,const std::vector<Real>& u)
{
  Assert(p >= 0);
  Assert(i >= 0 && i+p+1 < (int)u.size());
  if(p==0) {
    if(u[i] <= t && t < u[i+1] && u[i] < u[i+1]) return One;
    else return Zero;
  }
  Real N1=CoxDeBoor(i,p-1,t,u);
  Real N2=CoxDeBoor(i+1,p-1,t,u);
  Real sum=Zero;
  if(N1 != Zero) sum += N1*(t-u[i])/(u[i+p]-u[i]);
  if(N2 != Zero) sum += N2*(u[i+p+1]-t)/(u[i+p+1]-u[i+1]);
  return sum;
}

///assuming N is of size p+1, fills in the Cox-DeBoor basis coefficients
///N(i,p)...N(i+p,p) in N[0]...N[p].  Requires u[i+p]<=t<=u[i+p+1]
void CoxDeBoor(int i,int p,Real t,const std::vector<Real>& u,Real N[])
{
  Assert(u[i+p] <= t && t < u[i+p+1]);
  for(int k=0;k<p;k++) N[k]=0;
  N[p] = 1;
  //starting condition: N[k] contains N(k+i,0)
  //at the end of each iteration j, N[k] will contain N(k+i,j)
  for(int j=1;j<=p;j++) {
    int pj = j;
    //N[k] = 0 unless it's N(i+p-j)...N(i+p)
    for(int k=p-j;k<=p;k++) {
      int ij = i+k;
      Real sum=Zero;
      //N[k] can still be zero if u[i+k+j] = u[i+k]
      Real den=(u[ij+pj]-u[ij]);
      if(den != Zero) sum += N[k]*(t-u[ij])/den;
      else Assert(N[k]==Zero);
      if(k < p) {
	den = (u[ij+pj+1]-u[ij+1]);
	if(den != Zero) sum += N[k+1]*(u[ij+pj+1]-t)/den;
	else Assert(N[k+1]==Zero);
      }
      N[k] = sum;
    }
  }
}

//assuming N is of size p+1, fills in the Cox-DeBoor basis and derivative
//coefficients N(i,p)...N(i+p,p) in N[0]...N[p] and 
//N'(i,p)...N'(i+p,p) in dN[0]...dN[p].  Requires u[i+p]<=t<=u[i+p+1]
void CoxDeBoorDeriv(int i,int p,Real t,const std::vector<Real>& u,Real N[],Real dN[])
{
  Assert(u[i+p] <= t && t < u[i+p+1]);
  for(int k=0;k<p;k++) N[k]=0;
  for(int k=0;k<=p;k++) dN[k]=0;
  N[p] = 1;
  //starting condition: N[k] = N(k+i,0), dN[k] = N'(k+i,0)
  //at the end of each iteration j, N[k] = N(k+i,j), dN[k] = N'(k+i,j)
  for(int j=1;j<=p;j++) {
    int pj = j;
    for(int k=p-j;k<=p;k++) {
      int ij = i+k;
      Real sum=Zero,dsum=Zero;
      Real den=(u[ij+pj]-u[ij]);
      if(den != Zero) {
	sum += N[k]*(t-u[ij])/den;
	dsum += (dN[k]*(t-u[ij])+N[k])/den;
      }
      else Assert(N[k]==Zero);
      if(k < p) {
	Real den=(u[ij+pj+1]-u[ij+1]);
	if(den != Zero) {
	  sum += N[k+1]*(u[ij+pj+1]-t)/den;
	  dsum += (dN[k+1]*(u[ij+pj+1]-t)-N[k+1])/den;
	}
	else Assert(N[k+1]==Zero);
      }
      N[k] = sum;
      dN[k] = dsum;
    }
  }
}

//Same as above but includes 2nd derivative ddN=N''
void CoxDeBoorDeriv2(int i,int p,Real t,const std::vector<Real>& u,Real N[],Real dN[],Real ddN[])
{
  Assert(u[i+p] <= t && t < u[i+p+1]);
  for(int k=0;k<p;k++) N[k]=0;
  for(int k=0;k<=p;k++) dN[k]=0;
  for(int k=0;k<=p;k++) ddN[k]=0;
  N[p] = 1;
  //starting condition: N[k] = N(k+i,0), dN[k] = N'(k+i,0), ddN[k] = N''(k+1,0)
  //at the end of each iteration j, N[k] = N(k+i,j), dN[k] = N'(k+i,j), ddN[k] = N''(k+1,j)
  for(int j=1;j<=p;j++) {
    int pj = j;
    for(int k=p-j;k<=p;k++) {
      int ij = i+k;
      Real sum=Zero,dsum=Zero,ddsum=Zero;
      Real den=(u[ij+pj]-u[ij]);
      if(den != Zero) {
	sum += N[k]*(t-u[ij])/den;
	dsum += (dN[k]*(t-u[ij])+N[k])/den;
	ddsum += (ddN[k]*(t-u[ij])+Two*dN[k])/den;
      }
      else Assert(N[k] ==Zero);
      if(k < p) {
	Real den=(u[ij+pj+1]-u[ij+1]);
	if(den != Zero) {
	  sum += N[k+1]*(u[ij+pj+1]-t)/den;
	  dsum += (dN[k+1]*(u[ij+pj+1]-t)-N[k+1])/den;
	  ddsum += (ddN[k+1]*(u[ij+pj+1]-t)-Two*dN[k+1])/den;
	}
	else Assert(N[k+1]==Zero);
      }
      N[k] = sum;
      dN[k] = dsum;
      ddN[k] = ddsum;
    }
  }
}

//Same as above but includes Nth derivative
void CoxDeBoorDerivN(int i,int p,Real t,int n,const std::vector<Real>& u,Real** N)
{
  Assert(u[i+p] <= t && t < u[i+p+1]);
  Assert(n <= p);
  for(int m=0;m<=n;m++)
    for(int k=0;k<=p;k++) N[m][k]=0;
  N[0][p] = 1;
  //starting condition: N[k] = N(k+i,0), etc
  //at the end of each iteration j, N[k] = N(k+i,j), etc
  Real* sums = new Real[n+1];
  for(int j=1;j<=p;j++) {
    int pj = j;
    for(int k=p-j;k<=p;k++) {
      int ij = i+k;
      for(int m=0;m<=n;m++) sums[m]=Zero;
      Real den=(u[ij+pj]-u[ij]);
      if(den != Zero) {
	sums[0] += N[0][k]*(t-u[ij])/den;
	for(int m=1;m<=j;m++)   //only the derivatives up to j can be nonzero
	  sums[m] += (N[m][k]*(t-u[ij])+Real(m)*N[m-1][k])/den;
      }
      else Assert(N[0][k] ==Zero);
      if(k < p) {
	Real den=(u[ij+pj+1]-u[ij+1]);
	if(den != Zero) {
	  sums[0] += N[0][k+1]*(u[ij+pj+1]-t)/den;
	  for(int m=1;m<=j;m++)
	    sums[m] += (N[m][k+1]*(u[ij+pj+1]-t)-Real(m)*N[m-1][k+1])/den;
	}
      }
      else Assert(N[0][k+1]==Zero);
      for(int m=0;m<=n;m++)
	N[m][k] = sums[m];
    }
  }
}

//B must be of size p+1
//returns the spline basis for u[base] <= t < u[base+1] such that
//N(i,p)(t) = sum(k=0...p) (B[k]*t^k) 
void CoxDeBoorBasis(int base,int i,int p,const std::vector<Real>& u,Real B[])
{
  Assert(p >= 0);
  Assert(i >= 0 && i+p+1 < (int)u.size());
  if(p==0) {
    B[0]=Delta(base,i);
    return;
  }
  Real* B1 = new Real[p];
  Real* B2 = new Real[p];
  CoxDeBoorBasis(base,i,p-1,u,B1);
  CoxDeBoorBasis(base,i+1,p-1,u,B2);
  //N(i,p) = N(i,p-1)*(t-u[i])/(u[i+1]-u[i])
  //       + N(i+1,p-1)*(u[i+p+1]-t)/(u[i+p+1]-u[i+1])
  for(int k=0;k<=p;k++) B[k] = Zero;
  Real den1 = (u[i+p]-u[i]);
  Real den2 =(u[i+p+1]-u[i+1]);
  for(int k=0;k<p;k++) {
    if(den1 != Zero) {
      B[k] -= B1[k]*u[i]/den1;      //constant part
      B[k+1] += B1[k]/den1;    //linear part
    }
    if(den2 != Zero) {
      B[k] += B2[k]*u[i+p+1]/den2;      //constant part
      B[k+1] -= B2[k]/den2;    //linear part
    }
  }
  delete [] B1;
  delete [] B2;
}

//Returns the basis B[i,k] such that for u[base+p]<=t<u[base+p+1]
//N(base+i,p) = sum(k=0...p) B[i,k]*t^k
void CoxDeBoorBasis2(int base,int p,const std::vector<Real>& u,Real** B)
{
  Assert(p >= 0);
  Assert(base >= 0 && base+p+1 < (int)u.size());
  for(int i=0;i<=p;i++) {
    for(int k=0;k<=p;k++)
      B[i][k] = Zero;
  }
  if(u[base+p] < u[base+p+1]) B[p][0]=One;
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Uh... u[base] = u[base+1]?");
    return;
  }

  //start: B[i,k] are coefficients for N(base+i,0)
  //loop invariant: after step j, B[i,k] are coefficients for N(base+i,j)
  //note that B[i,k] is nonzero only for k<=j
  //          N(base+i,j) only nonzero for p-j<=i<=p.
  Real* Btemp = new Real[p+1];
  for(int j=1;j<=p;j++) {
    //N(i,p) = N(i,p-1)*(t-u[i])/(u[i+1]-u[i])
    //       + N(i+1,p-1)*(u[i+p+1]-t)/(u[i+p+1]-u[i+1])
    int pj = j;
    for(int i=p-j;i<=p;i++) {
      int ij = i+base;
      Real den1 = (u[ij+pj]-u[ij]);
      Real den2 =(u[ij+pj+1]-u[ij+1]);
      for(int k=0;k<=j;k++) Btemp[k]=Zero;
      for(int k=0;k<j;k++) {
	if(den1 != Zero) {
	  Btemp[k] -= B[i][k]*u[ij]/den1;      //constant part
	  Btemp[k+1] += B[i][k]/den1;    //linear part
	}
	if(i!=p && den2 != Zero) {
	  Btemp[k] += B[i+1][k]*u[ij+pj+1]/den2;      //constant part
	  Btemp[k+1] -= B[i+1][k]/den2;    //linear part
	}
      }
      for(int k=0;k<=j;k++)
	B[i][k] = Btemp[k];
    }
  }
  delete [] Btemp;
}

int BSplineBasis::GetKnot(Real t) const
{
  vector<Real>::const_iterator it=--upper_bound(knots.begin(),knots.end(),t);
  int knot = it-knots.begin();
  Assert(knots[knot] <= t);
  Assert(t < knots[knot+1]);
  return knot;
}

void BSplineBasis::SetUniformKnots(size_t numCps,size_t degree,Real h)
{
  numControlPoints = (int)numCps;
  knots.resize(numCps+degree+1);
  for(size_t i=0;i<knots.size();i++) 
    knots[i] = Real(int(i)-int(degree))*h;
}

void BSplineBasis::SetClosedKnots(size_t numCps,size_t degree,Real h)
{
  numControlPoints = (int)numCps;
  knots.resize(numCps+degree+1);
  for(size_t i=0;i<degree+1;i++) 
    knots[i] = 0;
  for(size_t i=degree+1;i<numCps;i++) 
    knots[i] = (int(i)-int(degree))*h;
  for(size_t i=numCps;i<numCps+degree+1;i++) 
    knots[i] = (int(numCps)-int(degree))*h;
}


void BSplineBasis::Evaluate(Real t,SparseVector& basis) const
{
  basis.resize(numControlPoints);
  basis.setZero();
  int p=Degree();
  /*
   This version is slow (CoxDeBoor version 1 is exponential)
  for(int i=0;i<numControlPoints;i++) {
    Real bi = CoxDeBoor(i,p,t,knots);
    if(bi != Zero) basis.set(i,bi);
  }
  */
  if(t < knots[p]) return;
  if(t >= knots[knots.size()-p]) return;
  //find knot point such that ui<=t<ui+1
  int knot = GetKnot(t);
  Real* N = new Real[p+1];
  CoxDeBoor(knot-p,p,t,knots,N);
  for(int i=0;i<=p;i++) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"N["<<i<<"]:"<<N[i]);
    //LOG4CXX_INFO(KrisLibrary::logger(),"basis["<<knot-p+i<<"]:"<<basis(knot-p+i));
    basis.set(knot-p+i,N[i]);
  }
  delete [] N;
}

void BSplineBasis::Evaluate(int knot,SparseMatrix& basis) const
{
  int p=Degree();
  Assert(knot >= p && knot < (int)knots.size()-p-1);
  Real** B = array2d_create<Real>(p+1,p+1);
  CoxDeBoorBasis2(knot-p,p,knots,B);
  basis.setZero();
  basis.resize(numControlPoints,p+1);
  for(int i=0;i<=p;i++) {
    for(int j=0;j<=p;j++) {
      basis.insertEntry(i+knot-p,j,B[i][j]);
    }
  }
  array2d_delete(B);
}

void BSplineBasis::Deriv(Real t,SparseVector& basis) const
{
  basis.resize(numControlPoints);
  basis.setZero();
  int p=Degree();
  if(t < knots[p]) return;
  if(t >= knots[knots.size()-p]) return;
  //find knot point i such that ui<=t<ui+1
  int knot = GetKnot(t);
  Real* N = new Real[p+1];
  Real* dN = new Real[p+1];
  CoxDeBoorDeriv(knot-p,p,t,knots,N,dN);
  for(int i=0;i<=p;i++) {
    basis.set(knot-p+i,dN[i]);
  }
  delete [] N;
  delete [] dN;
}

void BSplineBasis::Deriv2(Real t,SparseVector& basis) const
{
  basis.resize(numControlPoints);
  basis.setZero();
  int p=Degree();
  if(t < knots[p]) return;
  if(t >= knots[knots.size()-p-1]) return;
  //find knot point i such that ui<=t<ui+1
  int knot = GetKnot(t);
  Real* N = new Real[p+1];
  Real* dN = new Real[p+1];
  Real* ddN = new Real[p+1];
  CoxDeBoorDeriv2(knot-p,p,t,knots,N,dN,ddN);
  for(int i=0;i<=p;i++) {
    basis.set(knot-p+i,ddN[i]);
  }
  delete [] N;
  delete [] dN;
  delete [] ddN;
}


void BSplineBasis::EvaluateWithDerivs(Real t,SparseVector& b,SparseVector& db,SparseVector& ddb) const
{
  b.resize(numControlPoints);
  db.resize(numControlPoints);
  ddb.resize(numControlPoints);
  b.setZero();
  db.setZero();
  ddb.setZero();
  int p=Degree();
  if(t < knots[p]) return;
  if(t >= knots[knots.size()-p-1]) return;
  //find knot point i such that ui<=t<ui+1
  int knot = GetKnot(t);
  Real* N = new Real[p+1];
  Real* dN = new Real[p+1];
  Real* ddN = new Real[p+1];
  CoxDeBoorDeriv2(knot-p,p,t,knots,N,dN,ddN);
  for(int i=0;i<=p;i++) {
    b.set(knot-p+i,N[i]);
    db.set(knot-p+i,dN[i]);
    ddb.set(knot-p+i,ddN[i]);
  }
  delete [] N;
  delete [] dN;
  delete [] ddN;
}

void BSplineBasis::EvaluateWithDerivs(Real t,vector<SparseVector >& db) const
{
  int p=Degree();
  Assert((int)db.size() <= p);
  Assert(!db.empty());
  for(size_t i=0;i<db.size();i++) {
    db[i].resize(numControlPoints);
    db[i].setZero();
  }
  if(t < knots[p]) return;
  if(t >= knots[knots.size()-p-1]) return;
  //find knot point i such that ui<=t<ui+1
  int knot = GetKnot(t);
  Real** N = array2d_create<Real>(db.size(),p+1);
  CoxDeBoorDerivN(knot-p,p,t,db.size()-1,knots,N);
  for(size_t m=0;m<db.size();m++) {
    for(int i=0;i<=p;i++)
      db[m].set(knot-p+i,N[m][i]);
  }
  array2d_delete(N);
}

bool BSplineBasis::IsValid() const
{
  if((int)knots.size() < numControlPoints) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Fewer knots than control points\n");
    return false;
  }
  for(size_t i=1;i<knots.size();i++) {
    if(knots[i] < knots[i-1]) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Knot vector is not monotonic\n");
      return false;
    }
  }
  return true;
}


BSpline::BSpline()
{}

void BSpline::SetConstant(const Vector& value)
{
  cps.resize(basis.numControlPoints);
  fill(cps.begin(),cps.end(),value);
}

void BSpline::Evaluate(Real t,Vector& value) const
{
  SparseVector weights;
  basis.Evaluate(t,weights);
  if(weights.empty()) {
    value.resize(cps[0].n);
    value.setZero();
    return;
  }
  int n=0;
  for(SparseVector::const_iterator i=weights.begin();i!=weights.end();i++) {
    if(n==0) 
      value.mul(cps[i->first],i->second);
    else
      value.madd(cps[i->first],i->second);
    n++;
  }
}

void BSpline::Deriv(Real t,Vector& value) const
{
  SparseVector weights;
  basis.Deriv(t,weights);
  if(weights.empty()) {
    value.resize(cps[0].n);
    value.setZero();
    return;
  }
  int n=0;
  for(SparseVector::const_iterator i=weights.begin();i!=weights.end();i++) {
    if(n==0) 
      value.mul(cps[i->first],i->second);
    else
      value.madd(cps[i->first],i->second);
    n++;
  }
}

void BSpline::Deriv(Real t,int k,Vector& value) const
{
  if(k == 0) {
    Evaluate(t,value);
  }
  else if(k==1) {
    Deriv(t,value);
  }
  else {
    vector<SparseVector> weights(k+1);
    basis.EvaluateWithDerivs(t,weights);
    if(weights[k].empty()) {
      value.resize(cps[0].n);
      value.setZero();
      return;
    }
    int n=0;
    for(SparseVector::const_iterator i=weights[k].begin();i!=weights[k].end();i++) {
      if(n==0) 
	value.mul(cps[i->first],i->second);
      else
	value.madd(cps[i->first],i->second);
      n++;
    }
  }
}

bool BSpline::IsValid() const
{
  if(!basis.IsValid()) return false;
  if((int)cps.size() != basis.numControlPoints) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid number of control points\n");
    return false;
  }
  if(cps.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Spline is empty\n");
    return false;
  }
  for(size_t i=1;i<cps.size();i++) {
    if(cps[i].n != cps[0].n) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid control point size\n");
      return false;
    }
  }
  return true;
}

std::istream& operator >> (std::istream& in,BSpline& spline)
{
  int numcps,degree;
  in>>numcps>>degree;
  if(!in) return in;
  if(numcps < 0 || degree < 0) {
    in.clear(ios::badbit);
    return in;
  }
  spline.basis.knots.resize(numcps+degree+1);
  spline.basis.numControlPoints = numcps;
  spline.cps.resize(numcps);
  for(size_t i=0;i<spline.basis.knots.size();i++)
    in>>spline.basis.knots[i];
  for(size_t i=0;i<spline.cps.size();i++)
    in>>spline.cps[i];
  return in; 
}

std::ostream& operator << (std::ostream& out,const BSpline& spline)
{
  out<<spline.basis.numControlPoints<<" "<<spline.basis.Degree()<<endl;
  for(size_t i=0;i<spline.basis.knots.size();i++)
    out<<spline.basis.knots[i]<<" ";
  out<<endl;
  for(size_t i=0;i<spline.cps.size();i++)
    out<<spline.cps[i]<<endl;
  return out;
}

} //namespace Spline

#include <KrisLibrary/Logger.h>
#include "MonotonicSpline.h"
#include <math/misc.h>
#include <errors.h>
using namespace Spline;
using namespace std;

//t'(u) = sum tk*bk'(u) > 0
//suppose u in [ui,ui+1]
//letting bk'(u) = Bk.U where Bk is constant, U=[1,u,u^2,...]
//(sum tk Bk).U > 0
//it seems like tk+1 > tk makes this true
bool MonotonicSpline::IsValid() const
{
  if((int)t.size() != basis.numControlPoints) return false;
  for(size_t i=0;i+1<t.size();i++)
    if(t[i] >= t[i+1]) return false;
  return true;
}

Real MonotonicSpline::UtoT(Real u) const
{
  if(u < basis.knots[basis.Degree()]) return t.front();
  if(u >= basis.knots[basis.knots.size()-basis.Degree()]) return t.back();
  SparseVector v;
  basis.Evaluate(u,v);
  Assert(v.numEntries()!=0);
  Real sum=Zero;
  for(SparseVector::const_iterator i=v.begin();i!=v.end();i++) 
    sum += t[i->first]*i->second;
  return sum;
}

Real MonotonicSpline::TDeriv(Real u) const
{
  SparseVector v;
  basis.Deriv(u,v);
  Real sum=Zero;
  for(SparseVector::const_iterator i=v.begin();i!=v.end();i++) 
    sum += t[i->first]*i->second;
  return sum;
}

Real MonotonicSpline::TDeriv2(Real u) const
{
  SparseVector v;
  basis.Deriv2(u,v);
  Real sum=Zero;
  for(SparseVector::const_iterator i=v.begin();i!=v.end();i++) 
    sum += t[i->first]*i->second;
  return sum;
}

template <class T>
T Cube(T x) { return x*x*x; }

Real MonotonicSpline::TtoU(Real time) const
{
  if(time < t.front()) time = t.front();
  if(time >= t.back()) time = t.back()-1e-4;
  //lame way: bisect to get the right segment
  Real u1=basis.knots[basis.Degree()],u2=basis.knots[basis.knots.size()-basis.Degree()];
  Assert(u2 > u1);
  Real umid;
  Real tol=1e-4;
  while(u2-u1 > tol) {
    umid = (u2+u1)*Half;
    if(UtoT(umid) > time) u2=umid;
    else u1=umid;
  }
  umid = (u2+u1)*Half;
  //next, can solve the cubic equation for the segment where u is defined

  int k=basis.GetKnot(umid);
  SparseMatrix mat;
  basis.Evaluate(k,mat);
  vector<Real> coeffs(basis.Degree()+1,Zero);
  Assert(mat.rows.size() == t.size());
  //multiply t*basis
  for(size_t i=0;i<mat.rows.size();i++) {
    Assert(mat.rows[i].n == coeffs.size());
    for(SparseMatrix::RowT::iterator j=mat.rows[i].begin();j!=mat.rows[i].end();j++) {
      coeffs[j->first] += t[i]*j->second;
    }
  }
  if(coeffs.size()==2) {
    //it's linear
    //a*x+b=t
    Assert(coeffs[1] >= 1e-4);
    return (time-coeffs[0])/coeffs[1];
  }
  else if(coeffs.size()==3) {
    //solve the quadratic equation
    Real u[3];
    Real offset=basis.knots[k];
    //add offset to get better resolution
    //a*x^2+b*x+c=t
    //a*(y+offset)^2 + b*(y+offset) + c=t
    Real a=coeffs[2],b=coeffs[1],c=coeffs[0]-time;
    c+=b*offset+a*Sqr(offset);
    b+=Two*offset*a;
    int res;
    if(FuzzyZero(a)) {
      u[0] = -c/b;
      res=1;
    }
    else {
      res=quadratic(a,b,c,u[0],u[1]);
    }
    u[0] += offset;
    u[1] += offset;
    //LOG4CXX_INFO(KrisLibrary::logger(),"finding u for t(u)="<<time<<" approximated by "<<umid);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Roots to "<<coeffs[2]<<"x^2+"<<coeffs[1]<<"x+"<<coeffs[0]-time<<": "<<u0<<", "<<u1);
    if(res == 0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"No roots to equation "<<coeffs[2]<<"x^2+"<<coeffs[1]<<"x+"<<coeffs[0]<<"=time "<<time);
      LOG4CXX_INFO(KrisLibrary::logger(),"offset and normalized: x^2+"<<b/a<<"x+"<<c/a);
      LOG4CXX_INFO(KrisLibrary::logger(),"Determinant: "<<Sqr(b)-4.0*a*c);
      if(Abs(Sqr(b)-(Real)4.0*a*c) < 1e-5) {  //determinant close to zero
	u[0]=-Half*b/a+offset;
	res=1;
	LOG4CXX_INFO(KrisLibrary::logger(),"revised estimate: "<<u[0]);
      }
    }
    Assert(res >= 1);
    int best=0;
    Real bestmargin = Min(u[0]-basis.knots[k],basis.knots[k+1]-u[0]);
    for(int i=1;i<res;i++) {
      Real margin = Min(u[i]-basis.knots[k],basis.knots[k+1]-u[i]);
      if(margin > bestmargin) {
	bestmargin = margin;
	best=i;
      }
    }
    if(bestmargin <= -tol) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MonotonicSpline::TtoU: possible numerical error: u="<<u[best]<<" in ["<<basis.knots[k]<<","<<basis.knots[k+1]);
      /*
      if(!FuzzyZero(coeffs[2],tol) && !FuzzyZero(coeffs[1],tol)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Uh... did the quadratic equation solve correctly?\n");
	LOG4CXX_INFO(KrisLibrary::logger(),""<<coeffs[2]<<"u^2+"<<coeffs[1]<<"u+"<<coeffs[0]<<"="<<time);
	LOG4CXX_INFO(KrisLibrary::logger(),"u="<<u[best]<<" in ["<<basis.knots[k]<<","<<basis.knots[k+1]);
	LOG4CXX_INFO(KrisLibrary::logger(),"umid = "<<umid);
	KrisLibrary::loggerWait();
      }
      */
      return umid;
    }
    Assert(bestmargin > -tol);
    Real ttol = tol*Abs(time+One);
    if(!FuzzyEquals(UtoT(u[best]),time,ttol)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MonotonicSpline::TtoU: numerical error solving quadratic equation? "<<UtoT(u[best])<<" vs "<<time);
      LOG4CXX_INFO(KrisLibrary::logger(),""<<coeffs[2]<<"u^2+"<<coeffs[1]<<"u+"<<coeffs[0]<<"="<<time);
      LOG4CXX_INFO(KrisLibrary::logger(),"result="<<u[best]);
    }
    Assert(FuzzyEquals(UtoT(u[best]),time,ttol));
    return u[best];
  }
  else if(coeffs.size()==4) {
    //solve the cubic equation
    //solve the quadratic equation
    Real u[3];
    Real offset=basis.knots[k];
    //add offset to get better resolution
    //a*x^3+b*x^2+c*x+d=t
    //a*(y+offset)^3 + b*(y+offset)^2 + c*(y+offset) + d=t
    //a*(y^3+3offsety^2+3offset^2y+offset^3) + b*(y^2+2offset*y+offset^2)+c*(y+offset)+d
    //= a*y^3 + (b+3aoffset)*y^2+(c+2boffset+3aoffset^2)*y+(d+aoffset^3+boffset^2+coffset)
    Real a=coeffs[3],b=coeffs[2],c=coeffs[1],d=coeffs[0]-time;
    d += a*Cube(offset)+b*Sqr(offset)+c*offset;
    c += 2.0*b*offset+3.0*a*Sqr(offset);
    b += 3.0*a*offset;
    int res;
    if(FuzzyZero(a)) res=quadratic(b,c,d,u[0],u[1]);
    else res=cubic(a,b,c,d,u);
    for(int i=0;i<res;i++) u[i]+=offset;
    Assert(res >= 1);

    int best=0;
    Real bestmargin = Min(u[0]-basis.knots[k],basis.knots[k+1]-u[0]);
    for(int i=1;i<res;i++) {
      Real margin = Min(u[i]-basis.knots[k],basis.knots[k+1]-u[i]);
      if(margin > bestmargin) {
	best=i;
	bestmargin=margin;
      }
    }
    if(bestmargin <= -tol) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Best margin of cubic equation is outside of bracket ["<<u1<<","<<u2); 
      LOG4CXX_INFO(KrisLibrary::logger(),"Did the cubic solve correctly? "<<((coeffs[3]*u[best]+coeffs[2])*u[best]+coeffs[1])*u[best]+coeffs[0]<<"="<<time);
    }
    Assert(bestmargin > -tol);
    Real ttol = tol*Abs(time+One);
    if(!FuzzyEquals(UtoT(u[best]),time,ttol)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MonotonicSpline::TtoU: numerical error solving cubic equation? "<<UtoT(u[best])<<" vs "<<time);
    }
    Assert(FuzzyEquals(UtoT(u[best]),time,ttol));
    return u[best];
  }
  return umid;
}

Real MonotonicSpline::UDeriv(Real time) const
{
  Real u=TtoU(time);
  return One/TDeriv(u);
}

//t = g(u)
//u = f(t)
//d/dt g(f(t)) = g'*f' = 1
//d^2/dt^2 g(f(t)) = g''*f'^2+g'f'' = d^2/dt^2 t = 0
//=> f'' = -g''*f'^2/g' = -g''/g'^3
Real MonotonicSpline::UDeriv2(Real time) const
{
  Real u=TtoU(time);
  Real du=TDeriv(u);
  Real ddu=TDeriv2(u);
  return -ddu/(du*Sqr(du));
}

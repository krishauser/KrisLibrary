#include "PathOptimization.h"
#include <math/quadrature.h>
#include <math/LUDecomposition.h>
#include <math/brent.h>
#include <errors.h>
using namespace Optimization;
using namespace std;

template <class T,class U>
void LinearInterpolate(const T& a, const T& b, const U& u, T& x)
{
  x = (1-u)*a+u*b;
}

template <> void LinearInterpolate(const Vector& a, const Vector& b, const Real& u, Vector& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

PathOptimization::PathOptimization(PiecewiseLinearPath& _y)
  :y(_y)
{}

VectorFieldFunction* PathOptimization::Constraint() const
{
  return C;
}

void PathOptimization::Initialize()
{
  int N = (int)y.size();
  int m=y[0].n;
  cout<<"Init "<<N<<" by "<<m<<endl;
  a.resize(y.size(),Zero);
  VectorFieldFunction* c=Constraint();
  if(c) {
    mu.resize(y.size());
    int nd = c->NumDimensions();
    for(int i=0;i<N;i++) {
      mu[i].resize(nd,Zero);
    }
  }
  A.resize(N,m);
  A1.resize(N,m);
  A2.resize(N,m);
  A3.resize(N,m);
  if(c) A4.resize(N,m);
  b.resize(N-1,m);
  b1.resize(N,m);
  b2.resize(N,m);
  b3.resize(N,m);
  if(c) b4.resize(N,m);
  fy.resize(y.size());
  f0.resize(N-1);
  g1.resize(N); g2.resize(N);
  H1.resize(N); H2.resize(N); H3.resize(N);
  if(c) {
    gcmu1.resize(N); gcmu2.resize(N);
    Hcmu1.resize(N); Hcmu2.resize(N); Hcmu3.resize(N);
  }
  for(int i=0;i<N;i++) {
    g1[i].resize(m); g2[i].resize(m);
    H1[i].resize(m,m); H2[i].resize(m,m); H3[i].resize(m,m);
    if(c) {
      gcmu1[i].resize(m); gcmu2[i].resize(m);
      Hcmu1[i].resize(m,m); Hcmu2[i].resize(m,m); Hcmu3[i].resize(m,m);
    }
  }
  Ya.resize(N);
  for(int i=0;i<N;i++) {
    Ya.diagonal[i].resize(1,m);
    if(i < N-1) {
      Ya.upperDiagonal[i].resize(1,m);
      Ya.lowerDiagonal[i].resize(1,m);
    }
  }
  YaTb.resize(N);
  YaTYa.resize(N,N);
  dy.resize(N);
  y0.points.resize(N);
  for(int i=0;i<N;i++) {
    dy[i].resize(m);
    y0[i].resize(m);
  }
}

void PathOptimization::EvalY(Real t,Vector& yt) const
{
  Assert(y.BeginTime()==Zero);
  int index=(int)Floor(t/h);
  if(index >= (int)y.size()) yt=y.points.back().x;
  else if(index < 0) yt=y.points.front().x;
  else {
    Real u=t/h-Floor(t/h);
    y.Interpolate(y[index],y[index+1],u,yt);
  }
}

Real PathOptimization::EvalAlpha(Real t) const
{
  Real at;
  Assert(y.BeginTime()==Zero);
  int index=(int)Floor(t/h);
  if(index >= (int)y.size()) at=a[a.n-1];
  else if(index < 0) at=a[0];
  else {
    Real u=t/h-Floor(t/h);
    LinearInterpolate(a[index],a[index+1],u,at);
  }
  return at;
}

void PathOptimization::EvalMu(Real t,Vector& mut) const
{
  Assert(y.BeginTime()==Zero);
  int index=(int)Floor(t/h);
  if(index >= (int)y.size()) mut=mu.back();
  else if(index < 0) mut=mu.front();
  else {
    Real u=t/h-Floor(t/h);
    LinearInterpolate(mu[index],mu[index+1],u,mut);
  }
}

Real PathOptimization::CalcObjective()
{
  int N = (int)y.size();
  //update fy
  for(int i=0;i<N;i++) {
    //evaluate f at all points
    fy[i] = (*f)(y[i]);
  }

  Real sum=Zero;
  Vector temp;
  for(int i=0;i<N-1;i++) {
    //add integral of f*|y'| = |y'|*integral f
    //from yi to yi+1
    Real len = y.Distance(y[i+1],y[i]);

    //eval integral (simpsons)
    y.Interpolate(y[i],y[i+1],Half,temp);
    Real fmidpoint = (*f)(temp);
    Real integral = Quadrature::simpsons(fy[i],fmidpoint,fy[i+1])*Half;
    sum += len*integral;  //factors of h cancel out
  }
  return sum;
}

void PathOptimization::EvalIntegrals()
{
  EvalIntegralsFast();
}

void PathOptimization::EvalIntegralsFast()
{
  int N = (int)y.size();
  for(int i=0;i<N-1;i++) {
    //evaluate f at all points
    fy[i] = (*f)(y[i]);
  }

  Real fmid;
  Vector ytemp,mutemp;
  Vector grad0,grad1,gradmid;
  Vector gc0,gc1,gcmid;
  Matrix Hess0,Hess1,Hessmid;
  Matrix Hc0,Hc1,Hcmid;
  EvalGradients(y[0],mu[0],grad0,Hess0,gc0,Hc0);
  VectorFieldFunction* c=Constraint();
  const static Real simpsoncoeffs[3] = {1.0/6.0,4.0/6.0,1.0/6.0};
  for(int i=0;i<N-1;i++) {
    //midpoint
    y.Interpolate(y[i],y[i+1],Half,ytemp);
    mutemp.add(mu[i],mu[i+1]); mutemp*=Half;
    EvalGradients(y[i+1],mu[i+1],grad1,Hess1,gc1,Hc1);
    EvalGradients(ytemp,mutemp,gradmid,Hessmid,gcmid,Hcmid);
    fmid = f->Eval(ytemp);
    
    //evaluate integrals of f
    f0[i] = simpsoncoeffs[0]*fy[i]
      + simpsoncoeffs[1]*fmid
      + simpsoncoeffs[2]*fy[i+1];

    //evaluate integrals of grad f
    //g1[i+1] = int[ti->ti+1] s*grad(f(s)) ds
    //g2[i] = int[ti->ti+1] s*grad(f(1-s)) ds
    g1[i+1].mul(gradmid,simpsoncoeffs[1]*Half);
    g2[i] = g1[i+1];
    g1[i+1].madd(grad1,simpsoncoeffs[2]);
    g2[i].madd(grad0,simpsoncoeffs[2]);

    //evaluate integrals of Hessian f
    //H1[i+1] = int[ti->ti+1] s^2*H(f(s)) ds
    //H2[i] = int[ti->ti+1] s^2*H(f(1-s)) ds
    //H3[i] = int[ti->ti+1] s(1-s)*H(f(s)) ds
    H1[i+1].mul(Hessmid,simpsoncoeffs[1]*Half*Half);
    H2[i] = H1[i+1];
    H3[i] = H1[i+1];
    H1[i+1].madd(Hess1,simpsoncoeffs[2]);
    H2[i].madd(Hess0,simpsoncoeffs[2]);

    if(c) {
      //evaluate integrals of grad c^t*mu
      gcmu1[i+1].mul(gcmid,simpsoncoeffs[1]*Half);
      gcmu2[i] = gcmu1[i+1];
      gcmu1[i+1].madd(gc1,simpsoncoeffs[2]);
      gcmu2[i].madd(gc0,simpsoncoeffs[2]);

      //evaluate integrals of Hessian c^t*mu
      Hcmu1[i+1].mul(Hcmid,simpsoncoeffs[1]*Half*Half);
      Hcmu2[i] = Hcmu1[i+1];
      Hcmu3[i] = Hcmu1[i+1];
      Hcmu1[i+1].madd(Hc1,simpsoncoeffs[2]);
      Hcmu2[i].madd(Hc0,simpsoncoeffs[2]);
    }

    swap(grad0,grad1);
    swap(Hess0,Hess1);
    swap(gc0,gc1);
    swap(Hc0,Hc1);
  }
}

void PathOptimization::EvalGradients(const Vector& y,const Vector& mu,Vector& g,Matrix& H,Vector& gcmu,Matrix& Hcmu) const
{
  int n=y.n;
  g.resize(n);
  H.resize(n,n);
  f->PreEval(y);
  f->Gradient(y,g);
  f->Hessian(y,H);
  if(Constraint()) {
    VectorFieldFunction*c=Constraint();
    c->PreEval(y);
    int m=c->NumDimensions();
    int n=this->y[0].n;
    Matrix J(m,n);
    c->Jacobian(y,J);
    J.mulTranspose(mu,gcmu);

    Matrix mtemp(n,n);
    for(int i=0;i<m;i++) {
      c->Hessian_i(y,i,mtemp);
      if(i==0) Hcmu.mul(mtemp,mu(i));
      else Hcmu.madd(mtemp,mu(i));
    }
  }
}

void PathOptimization::EvalSDIntegrals()
{
  int N = (int)y.size();
  for(int i=0;i<N-1;i++) {
    //evaluate f at all points
    fy[i] = (*f)(y[i]);
  }

  Real fmid;
  Vector ytemp;
  Vector grad0,grad1,gradmid;
  int m=y[0].n;
  grad0.resize(m); grad1.resize(m); gradmid.resize(m);
  f->PreEval(y[0]);
  f->Gradient(y[0],grad0);
  const static Real simpsoncoeffs[3] = {1.0/6.0,4.0/6.0,1.0/6.0};
  for(int i=0;i<N-1;i++) {
    //midpoint
    y.Interpolate(y[i],y[i+1],Half,ytemp);
    /*
    if(D) {
      int index;
      D->PreEval(ytemp);
      Real mtemp=D->Margin(ytemp,index);
      if(mtemp < Zero) {
	if(!Push(D,ytemp,1e-5)) {
	  int i0,i1;
	  D->PreEval(y[i]); Real m0=D->Margin(y[i],i0);
	  D->PreEval(y[i+1]); Real m1=D->Margin(y[i+1],i1);
	  cout<<"Margin of midpoint is negative!"<<endl;
	  cout<<mtemp<<" at "<<D->Label(index)<<endl;
	  cout<<"Endpoints are "<<m0<<" at "<<D->Label(i0)<<" and "<<m1<<" at "<<D->Label(i1)<<endl;
	  cout<<"y0 "<<VectorPrinter(y[i])<<endl;
	  cout<<"y1 "<<VectorPrinter(y[i+1])<<endl;
	  cout<<"ymid "<<VectorPrinter(ytemp)<<endl;
	  getchar();
	}
      }
    }
    */

    f->PreEval(y[i+1]); f->Gradient(y[i+1],grad1);
    f->PreEval(ytemp); f->Gradient(ytemp,gradmid);
    fmid = f->Eval(ytemp);
    
    //evaluate integrals of f
    f0[i] = simpsoncoeffs[0]*fy[i]
      + simpsoncoeffs[1]*fmid
      + simpsoncoeffs[2]*fy[i+1];

    //evaluate integrals of grad f
    //g1[i+1] = int[ti->ti+1] s*grad(f(s)) ds
    //g2[i] = int[ti->ti+1] s*grad(f(1-s)) ds
    g1[i+1].mul(gradmid,simpsoncoeffs[1]*Half);
    if(g2[i].n != g1[i+1].n) {
      cout<<"HMM, this is weird! i="<<i<<", N="<<N<<endl;
      cout<<g2[i].n<<" vs "<<g1[i+1].n<<endl;
    }
    g2[i] = g1[i+1];
    g1[i+1].madd(grad1,simpsoncoeffs[2]);
    g2[i].madd(grad0,simpsoncoeffs[2]);
    if(IsNaN(g1[i+1].norm())) { 
      cout<<gradmid<<endl;
      cout<<grad1<<endl;
      cout<<"fy[i+1] "<<fy[i+1]<<endl;
      cout<<"ISNAN!"<<endl;
    }

    swap(grad0,grad1);
  }
}

void PathOptimization::NewtonStep()
{
  int m=y[0].n;
  Vector btemp(m);
  Matrix mtemp(m,m);
  int N = (int)y.size();
  EvalIntegrals();
  VectorFieldFunction* c=Constraint();

  //no change in y
  if(startFixed) {
    A(0,0).setIdentity();
    A(0,1).setZero();
    b[0].setZero();
  }
  else {
    cout<<"Error, can't do non-fixed start config"<<endl;
    abort();
  }
  if(endFixed) {
    A(N-1,N-1).setIdentity();
    A(N-2,N-1).setZero();
    b[N-1].setZero();
  }
  else {
    cout<<"Error, can't do non-fixed end config"<<endl;
    abort();
  }
  for(int i=1;i<N-1;i++) {
    int k=i;
    b1[k].mul(g1[i],y[i-1].distanceSquared(y[i]));
    b1[k].madd(g2[i],y[i+1].distanceSquared(y[i]));
    b1[k] /= Sqr(h);
    
    b2[k]=y[i+1];
    b2[k].madd(y[i],-Two);
    b2[k]+=y[i-1];
    b2[k].inplaceMul(fy[i]);

    b3[k].mul(y[i],a[i-1]/6-a[i+1]/6);
    b3[k].madd(y[i-1],-(a[i-1]/6+a[i]/3));
    b3[k].madd(y[i+1],-(a[i+1]/6+a[i]/3));
   
    if(c) {
      b4[k].add(gcmu1[i],gcmu2[i]);
    }

    A1(k,k).setZero();
    A1(k,k-1).setZero();
    A1(k,k+1).setZero();
    for(int p=0;p<m;p++) {
      for(int q=0;q<m;q++) {
	A1(k,k)(p,q) += Two* g1[i](p)*(y[i](q)-y[i-1](q));
	A1(k,k)(p,q) += Two* g2[i](p)*(y[i](q)-y[i+1](q));

	A1(k,k-1)(p,q) += Two* g1[i](p)*(y[i](q)-y[i-1](q));
	A1(k,k+1)(p,q) += Two* g2[i](p)*(y[i](q)-y[i+1](q));
      }
    }
    A1(k,k).madd(H1[i],y[i-1].distanceSquared(y[i]));
    A1(k,k).madd(H2[i],y[i+1].distanceSquared(y[i]));
    A1(k,k) /= Sqr(h);

    A1(k,k-1).madd(H3[i-1],y[i-1].distanceSquared(y[i]));
    A1(k,k+1).madd(H3[i],y[i+1].distanceSquared(y[i]));
    A1(k,k-1) /= Sqr(h);
    A1(k,k+1) /= Sqr(h);

    A2(k,k).setDiag(0,-Two*fy[i]);
    A2(k,k-1).setDiag(0,fy[i]);
    A2(k,k+1).setDiag(0,fy[i]);

    A3(k,k).setDiag(0,(a[i-1]-a[i+1])/6);
    A3(k,k-1).setDiag(0,-a[i-1]/6-a[i]/3);
    A3(k,k+1).setDiag(0,a[i+1]/6+a[i]/3);

    if(c) {
      A4(k,k).add(Hcmu1[i],Hcmu2[i]);
      A4(k,k-1) = Hcmu3[i-1];
      A4(k,k) = Hcmu3[i];
    }

    b.sub(b1,b2);
    b -= b3;
    if(c) b -= b4;

    A.sub(A1,A2);
    A -= A3;
    if(c) A -= A4;
  }

  if(c) {
    cout<<"Can't yet solve for constraints"<<endl;
    abort();
  }
  A.solveInverse_LU(b,dy);
  Assert(dy[0].isZero(1e-4));
  Assert(dy[N-1].isZero(1e-4));
  //TODO: solve for "good" step t
  Real t=One;
  for(int i=1;i<N-1;i++)
    y[i].madd(dy[i],t);

  SolveAlpha();
}

//assumes b1,b2,b4 are evaluated
void PathOptimization::SolveAlpha() 
{
  int N=(int)y.size();
  //solve for multipliers a
  Ya(0,0).setZero(); //?
  Ya(N-1,N-1).setZero(); //?
  Vector YaDiag,YaUpper,YaLower;
  for(int i=1;i<N-1;i++) {
    int k=i;
    Ya(k,k).getRowRef(0,YaDiag);
    Ya(k,k+1).getRowRef(0,YaUpper);
    Ya(k,k-1).getRowRef(0,YaLower);
    //Ya(k,k).sub(y[i+1],y[i-1]);
    y.Difference(y[i+1],y[i-1],YaDiag);
    YaDiag *= 1.0/3.0;
    //Ya(k,k-1).sub(y[i],y[i-1]);
    y.Difference(y[i],y[i-1],YaLower);
    YaLower *= 1.0/6.0;
    //Ya(k,k+1).sub(y[i+1],y[i]);
    y.Difference(y[i+1],y[i],YaUpper);
    YaUpper *= 1.0/6.0;
  }
  for(int i=0;i<N;i++) {
    //since Ya is tridiagonal, YaTYa only has bandwidth 2
    YaTb(i) = Zero;
    for(int k=i-1;k<=i+1;k++)
      YaTb(i) += Ya(i,k).dotRow(0,b1[k]-b2[k]-b4[k]);
    for(int j=0;j<N;j++) {
      Real sum = Zero;
      for(int k=min(i,j)-1;k<=max(i,j)+1;k++) 
	sum += Ya(k,i).row(0).dot(Ya(k,j).row(0));
      YaTYa(i,j) = sum;
    }
  }
  LUDecomposition<Real> lud;
  lud.set(YaTYa);
  lud.backSub(YaTb,a);
}

const Real scale = 1;

class ObjectiveOnDyFunction: public RealFunction
{
public:
  ObjectiveOnDyFunction(PathOptimization* _f) :f(_f) {}
  virtual Real Eval(Real t) { return f->EvalDy(t*scale); }
  
  PathOptimization* f;
};

void PathOptimization::CalcDySD()
{
  int N=(int)y.size();
  EvalSDIntegrals();
  //calc dy
  Vector temp;
  if(startFixed) dy[0].setZero();
  else {
    Real dist1;
    dist1 = y.Distance(y[0],y[1]);
    Assert(dist1 > Zero);
    //dy[i] = -(y[0]-y[1])/dist1*f0[0] - dist1*g2[0]
    y.Difference(y[0],y[1],temp);
    dy[0].mul(temp,-f0[0]/dist1);
    dy[0].madd(g2[0],-dist1);
  }
  if(endFixed) dy[N-1].setZero();
  else {
    Real dist0;
    dist0 = y.Distance(y[N-1],y[N-2]);
    Assert(dist0 > Zero);
    //dy[N-1] = - (y[N-1]-y[N-2])/dist0*f0[N-2] - dist0*g1[N-1]
    y.Difference(y[N-1],y[N-2],temp);
    dy[N-1].mul(temp,-f0[N-2]/dist0);
    dy[N-1].madd(g1[N-1],-dist0);
  }
  for(int i=1;i<N-1;i++) {
    Real dist0,dist1;
    dist0 = y.Distance(y[i],y[i-1]);
    dist1 = y.Distance(y[i],y[i+1]);
    Assert(dist0 > Zero);
    Assert(dist1 > Zero);
    //dy[i] = -(y[i]-y[i+1])/dist1*f0[i] - (y[i]-y[i-1])/dist0*f0[i-1]
    //      - dist1*g2[i] - dist0*g1[i]
    y.Difference(y[i],y[i+1],temp);
    dy[i].mul(temp,-f0[i]/dist1);
    y.Difference(y[i],y[i-1],temp);
    dy[i].madd(temp,-f0[i-1]/dist0);
    dy[i].madd(g2[i],-dist1);
    dy[i].madd(g1[i],-dist0);
    //cout<<"dy["<<i<<"]: "; dy[i].print();
  }

  /*
  if(M) {
    cout<<"Projecting dy to tangent space"<<endl;
    for(int i=0;i<N;i++) {
      //project dy on tangent space of constraint at y[i]
      M->TangentSpaceProjection(y[i],dy[i]);
    }
  }
  */
}

ConvergenceResult PathOptimization::SDStep(Real tolx,int maxMinimizationIters)
{
  CalcDySD();
  int N=(int)y.size();

  y0 = y;
  //integrate |dy| through path
  Real dyMax = Zero;
  dyMax += dy[0].norm()*Half;
  dyMax += dy[N-1].norm()*Half;
  for(int i=1;i<N-1;i++)
    dyMax += dy[i].norm();
  dyMax *= h;
  Assert(dyMax != Zero);

  //line minimization to find optimal t
  ObjectiveOnDyFunction f(this);
  Real t;
  ConvergenceResult res=ParabolicMinimization(0,f,maxMinimizationIters,tolx/(dyMax*scale),t);
  t *= scale;
  Real objective=EvalDy(t);
  cout<<"Optimal step is "<<t<<", approx distance "<<t*dyMax<<", achived in "<<maxMinimizationIters<<" iters.  New objective "<<objective<<endl;
  //if(res == ConvergenceX) { return ConvergenceX; }
  if(t*dyMax < tolx) return ConvergenceX;
  return MaxItersReached;
}

Real PathOptimization::EvalDy(Real t)
{
  for(size_t i=0;i<y.size();i++) {
    y[i] = y0[i];
    if(t != Zero) {
      y[i].madd(dy[i],t);
    }
  }
  if(C) {
    FatalError("NOT DONE YET");
    /*
    cout<<"Moving points to surface"<<endl;
    for(size_t i=0;i<y.size();i++) {
      bool res=MoveToSurface(C,y[i]);
      if(!res) {
	cout<<"Failed at "<<i<<"!"<<endl;
	getchar();
      }
    }
    */
  }

  return CalcObjective();
}


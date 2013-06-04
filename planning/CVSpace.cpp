#include "CVSpace.h"
#include "EdgePlanner.h"
#include "GeneralizedBezierCurve.h"

CVSpace::CVSpace(CSpace* _baseSpace,CSpace* _velSpace)
  :baseSpace(_baseSpace),velSpace(_velSpace)
{}

void CVSpace::Sample(Config& x)
{
  Assert(baseSpace && velSpace);
  Vector q,v;
  baseSpace->Sample(q);
  velSpace->Sample(v);
  SetState(q,v,x);
}

void CVSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  Vector cq,cv,xq,xv;
  x.resize(c.n);
  GetStateRef(x,xq,xv);
  GetStateRef(c,cq,cv);
  baseSpace->SampleNeighborhood(cq,r,xq);
  if(velSpace) velSpace->SampleNeighborhood(cv,r,xv);
  else CSpace::SampleNeighborhood(cv,r,xv);
}

bool CVSpace::IsFeasible(const Config& x)
{
  Vector q,v;
  GetStateRef(x,q,v);
  if(!baseSpace->IsFeasible(q)) return false;
  if(velSpace && !velSpace->IsFeasible(v)) return false;
  return true;
}

Real CVSpace::Distance(const Config& x, const Config& y)
{
  Vector xq,xv,yq,yv;
  GetStateRef(x,xq,xv);
  GetStateRef(y,yq,yv);
  Real d=baseSpace->Distance(xq,yq);
  if(velSpace) d += velSpace->Distance(xv,yv);
  else d += xv.distance(yv);
  return d;
}

EdgePlanner* CVSpace::LocalPlanner(const Config& a,const Config& b)
{
  //TODO: do this better
  return new BisectionEpsilonEdgePlanner(this,a,b,1e-3);
}

void CVSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Vector xq,xv,yq,yv,outq,outv;
  GetStateRef(x,xq,xv);
  GetStateRef(y,yq,yv);
  out.resize(x.n);
  GetStateRef(out,outq,outv);
  baseSpace->Interpolate(xq,yq,u,outq);
  if(velSpace) velSpace->Interpolate(xv,yv,u,outv);
  else CSpace::Interpolate(xv,yv,u,outv);
}

void CVSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

void CVSpace::GetState(const Config& x,Config& q,Vector& v)
{
  q.resize(x.n/2);
  v.resize(x.n/2);
  x.getSubVectorCopy(0,q);
  x.getSubVectorCopy(q.n,v);
}

void CVSpace::GetConfig(const Config& x,Config& q)
{
  q.resize(x.n/2);
  x.getSubVectorCopy(0,q);
}

void CVSpace::GetVelocity(const Config& x,Vector& v)
{
  v.resize(x.n/2);
  x.getSubVectorCopy(v.n,v);
}

void CVSpace::GetStateRef(const Config& x,Config& q,Vector& v)
{
  GetConfigRef(x,q);
  GetVelocityRef(x,v);
}

void CVSpace::GetConfigRef(const Config& x,Config& q)
{
  q.setRef(x,0,1,x.n/2);
}

void CVSpace::GetVelocityRef(const Config& x,Vector& v)
{
  v.setRef(x,x.n/2,1,x.n/2);
}

void CVSpace::SetState(const Config& q,const Vector& v,Config& x)
{
  Assert(q.n == v.n);
  x.resize(q.n+v.n);
  x.copySubVector(0,q);
  x.copySubVector(q.n,v);
}

void CVSpace::SetConfig(const Config& q,Config& x)
{
  Assert(x.n == q.n*2);
  x.copySubVector(0,q);
}

void CVSpace::SetVelocity(const Vector& v,Config& x)
{
  Assert(x.n == v.n*2);
  x.copySubVector(v.n,v);
}

CVGeodesicManifold::CVGeodesicManifold(GeodesicManifold* _baseManifold,GeodesicManifold* _velManifold)
  :baseManifold(_baseManifold),velManifold(_velManifold)
{}

void CVGeodesicManifold::InterpolateDeriv(const Config& x,const Config& y,Real u,Vector& dx)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  dx.resize(x.n);
  CVSpace::GetStateRef(dx,dq,dv);
  if(baseManifold) baseManifold->InterpolateDeriv(xq,yq,u,dq);
  else GeodesicManifold::InterpolateDeriv(xq,yq,u,dq);
  if(velManifold) velManifold->InterpolateDeriv(xv,yv,u,dv);
  else GeodesicManifold::InterpolateDeriv(xv,yv,u,dv);
}

void CVGeodesicManifold::Integrate(const Config& x,const Vector& dx,Config& y)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(dx,dq,dv);
  y.resize(x.n);
  CVSpace::GetStateRef(y,yq,yv);
  if(baseManifold) baseManifold->Integrate(xq,dq,yq);
  else yq = xq+dq;
  if(velManifold) velManifold->Integrate(xv,dv,yv);
  else yv = xv+dv;
}



HermiteCSpace::HermiteCSpace(CSpace* _baseSpace,CSpace* _velSpace,GeodesicManifold* _manifold)
  :CVSpace(_baseSpace,_velSpace),manifold(_manifold)
{}

Real HermiteCSpace::Distance(const Config& x, const Config& y)
{
  Vector xq,xv,yq,yv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  const static Real third=1.0/3.0;
  Vector x1,x2;
  if(manifold) {
    manifold->Integrate(xq,xv*third,x1);
    manifold->Integrate(yq,yv*(-third),x2);
  }
  else {
    x1 = xq+xv*third;
    x2 = yq-yv*third;
  }
  return baseSpace->Distance(xq,x1)+baseSpace->Distance(x1,x2)+baseSpace->Distance(x2,yq);
}

void HermiteCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Vector xq,xv,yq,yv,outq,outv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  out.resize(x.n);
  CVSpace::GetStateRef(out,outq,outv);
  GeneralizedCubicBezierCurve curve(baseSpace,manifold);
  curve.x0.setRef(xq);
  curve.x3.setRef(yq);
  curve.SetNaturalTangents(xv,yv);
  curve.Eval(u,outq);
  curve.Deriv(u,outv);
}

void HermiteCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

HermiteGeodesicManifold::HermiteGeodesicManifold(HermiteCSpace* _space)
  :space(_space)
{}

void HermiteGeodesicManifold::InterpolateDeriv(const Config& x,const Config& y,Real u,Vector& dx)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  dx.resize(x.n);
  CVSpace::GetStateRef(dx,dq,dv);
  GeneralizedCubicBezierCurve curve(space->baseSpace,space->manifold);
  curve.x0.setRef(xq);
  curve.x3.setRef(yq);
  curve.SetNaturalTangents(xv,yv);
  curve.Deriv(u,dq);
  curve.Accel(u,dv);
}

void HermiteGeodesicManifold::Integrate(const Config& x,const Vector& dx,Config& y)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(dx,dq,dv);
  y.resize(x.n);
  CVSpace::GetStateRef(y,yq,yv);
  if(space->manifold) space->manifold->Integrate(xq,dq,yq);
  else yq = xq+dq;
  yv = xv+dv;
}

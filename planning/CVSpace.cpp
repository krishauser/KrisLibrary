#include "CVSpace.h"
#include "EdgePlanner.h"
#include "GeneralizedBezierCurve.h"

CVSpace::CVSpace(const std::shared_ptr<CSpace>& _baseSpace,const std::shared_ptr<CSpace>& _velSpace)
  :MultiCSpace(_baseSpace,_velSpace)
{
  Assert(components.size()==2);
  Assert(components[0] == _baseSpace);
  Assert(components[1] == _velSpace);
  Assert(componentNames.size()==2);
  componentNames[0] = "q";
  componentNames[1] = "v";
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





HermiteCSpace::HermiteCSpace(const std::shared_ptr<CSpace>& _baseSpace,const std::shared_ptr<CSpace>& _velSpace)
  :CVSpace(_baseSpace,_velSpace)
{}

Real HermiteCSpace::Distance(const Config& x, const Config& y)
{
  Vector xq,xv,yq,yv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  const static Real third=1.0/3.0;
  Vector x1,x2;
  GeodesicCSpace* manifold = dynamic_cast<GeodesicCSpace*>(components[0].get());
  if(manifold) {
    manifold->Integrate(xq,xv*third,x1);
    manifold->Integrate(yq,yv*(-third),x2);
  }
  else {
    x1 = xq+xv*third;
    x2 = yq-yv*third;
  }
  CSpace* baseSpace = components[0].get();
  return baseSpace->Distance(xq,x1)+baseSpace->Distance(x1,x2)+baseSpace->Distance(x2,yq);
}

void HermiteCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Vector xq,xv,yq,yv,outq,outv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  out.resize(x.n);
  CVSpace::GetStateRef(out,outq,outv);
  GeodesicCSpace* manifold = dynamic_cast<GeodesicCSpace*>(components[0].get());
  GeneralizedCubicBezierCurve curve(components[0].get(),manifold);
  curve.x0.setRef(xq);
  curve.x3.setRef(yq);
  curve.SetNaturalTangents(xv,yv);
  curve.Eval(u,outq);
  curve.Deriv(u,outv);
}


void HermiteCSpace::Properties(PropertyMap& props)
{
  CVSpace::Properties(props);
  props.set("geodesic",1);
  props.set("metric","hermite upper bound");
}

void HermiteCSpace::InterpolateDeriv(const Config& x,const Config& y,Real u,Vector& dx)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(y,yq,yv);
  dx.resize(x.n);
  CVSpace::GetStateRef(dx,dq,dv);
  GeodesicCSpace* manifold = dynamic_cast<GeodesicCSpace*>(components[0].get());
  GeneralizedCubicBezierCurve curve(components[0].get(),manifold);
  curve.x0.setRef(xq);
  curve.x3.setRef(yq);
  curve.SetNaturalTangents(xv,yv);
  curve.Deriv(u,dq);
  curve.Accel(u,dv);
}

void HermiteCSpace::Integrate(const Config& x,const Vector& dx,Config& y)
{
  Vector xq,xv,yq,yv,dq,dv;
  CVSpace::GetStateRef(x,xq,xv);
  CVSpace::GetStateRef(dx,dq,dv);
  y.resize(x.n);
  CVSpace::GetStateRef(y,yq,yv);
  GeodesicCSpace* manifold = dynamic_cast<GeodesicCSpace*>(components[0].get());
  if(manifold) manifold->Integrate(xq,dq,yq);
  else yq = xq+dq;
  yv = xv+dv;
}

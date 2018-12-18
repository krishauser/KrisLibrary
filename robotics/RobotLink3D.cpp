#include <KrisLibrary/Logger.h>
#include "RobotLink3D.h"
#include <iostream>
using namespace std;

void RobotLink3D::SetRotationJoint(const Vector3& _w)
{
  w=_w;
  type = Revolute;
}

void RobotLink3D::SetTranslationJoint(const Vector3& _v)
{
  w=_v;
  type = Prismatic;
}

void RobotLink3D::GetLocalTransform(Real qi,Frame3D& T) const
{
  switch(type) {
  case Prismatic:
    T.R.setIdentity();
    T.t.mul(w,qi);
    break;
  case Revolute:
    T.t.setZero();
    if(w.x == One)
      T.R.setRotateX(qi);
    else if(w.y == One)
      T.R.setRotateY(qi);
    else if(w.z == One)
      T.R.setRotateZ(qi);
    else {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Not a standard axis: "<<w);
      MomentRotation r(qi*w);
      r.getMatrix(T.R);
    }
    break;
  default:
    T.setIdentity();
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid joint type "<<type);
    Abort();
    break;
  }
}

void RobotLink3D::GetOrientationJacobian(Vector3& Jo) const
{
  switch(type) {
  case Revolute:
    T_World.R.mul(w,Jo);
    break;
  case Prismatic:
    Jo.setZero();
    break;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid joint type");
    Abort();
    Jo.setZero();
    break;
  }
}

void RobotLink3D::GetPositionJacobian(Real qi,const Vector3& p, Vector3& Jp) const
{
  switch(type) {
  case Revolute:
    {
      Vector3 pi;
      T_World.mulPointInverse(p,pi);

      Frame3D TLoc;
      Matrix3 RLoc_World;
      GetLocalTransform(qi,TLoc);
      RLoc_World.mulTransposeB(T_World.R,TLoc.R);
  
      Jp = RLoc_World*(cross(w,TLoc.R*pi));
    }
    break;
  case Prismatic:
    Jp = T_World.R*w;
    break;
  default:
    LOG4CXX_INFO(KrisLibrary::logger(),"Invalid joint type");
    Jp.setZero();
    Abort();
    break;
  }
}


void RobotLink3D::GetJacobian(Real qi,const Vector3& p, Vector3& Jo, Vector3& Jp) const
{
  switch(type) {
  case Revolute:
    {
      Vector3 pi;
      T_World.mulPointInverse(p,pi);

      Frame3D TLoc;
      Matrix3 RLoc_World;
      GetLocalTransform(qi,TLoc);
      RLoc_World.mulTransposeB(T_World.R,TLoc.R);
  
      Jp = RLoc_World*(cross(w,TLoc.R*pi));
      T_World.R.mul(w,Jo);
    }
    break;
  case Prismatic:
    Jp = T_World.R*w;
    Jo.setZero();
    break;
  default:
    LOG4CXX_INFO(KrisLibrary::logger(),"Invalid joint type");
    Jp.setZero(); Jo.setZero();
    Abort();
    break;
  }
}

void RobotLink3D::GetJacobian(Real qi,Frame3D& J) const
{
  switch(type) {
  case Revolute:
    {
      Frame3D TWorld_i;
      TWorld_i.setInverse(T_World);

      Frame3D TLoc;
      Matrix3 RLoc_World;
      GetLocalTransform(qi,TLoc);
      RLoc_World.mulTransposeB(T_World.R,TLoc.R);
      
      Matrix3 JLocR;
      Matrix3 wcross;
      wcross.setCrossProduct(w);
      JLocR.mul(wcross,TLoc.R);
      J = (RLoc_World*JLocR)*TWorld_i;
    }
    break;
  case Prismatic:
    J.R.setZero();
    J.t = T_World.R*w;
    break;
  default:
    break;
  }
}

void RobotLink3D::GetJacobian(Real qi,const Frame3D& Tj_World,Frame3D& J) const
{
  Frame3D JWorld;
  GetJacobian(qi,JWorld);
  J = FrameMulWB1(JWorld,Tj_World);
}

void RobotLink3D::GetVelocity(Real qi,Real dqi,const Vector3& pi,Vector3& vel) const
{
  Vector3 Jp;
  GetPositionJacobian(qi,pi,Jp);
  vel.mul(Jp,dqi);
}

void RobotLink3D::GetAngularVelocity(Real dqi,Vector3& omega) const
{
  Vector3 Jo; 
  GetOrientationJacobian(Jo);
  omega.mul(Jo,dqi);
}

void RobotLink3D::GetWorldInertia(Matrix3& inertiaWorld) const
{
  //inertiaWorld = R*I*Rt
  Matrix3 temp;
  temp.mul(T_World.R,inertia);
  inertiaWorld.mulTransposeB(temp,T_World.R);
}



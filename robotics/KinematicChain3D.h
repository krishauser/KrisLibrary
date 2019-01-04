#ifndef ROBOTICS_KINEMATIC_CHAIN3D_H
#define ROBOTICS_KINEMATIC_CHAIN3D_H

#include <KrisLibrary/Logger.h>
#include "Frame.h"
#include "Rotation.h"
#include "Chain.h"
#include <string>

class KinematicLink3D
{
public:
  enum Type { Revolute, Prismatic, Spherical };

  void SetRotationJoint(const Vector3& w);
  void SetTranslationJoint(const Vector3& v);

  //screw's transformation is a function of control qi
  //say we're link i
  //T(i->i)(qi) = R(w*qi)oT(v*qi) where R rotates the given angle, T translates
  //T(i->pi)(qi) = T0(i->pi)*T(i->i)(qi)
  void GetLocalTransform(Real qi,Frame3D& T) const;

  //velocity of a point (in frame 0) with respect to qi,dqi
  void GetVelocity(Real qi,Real dqi,const Vector3& p,Vector3& vel) const;
  void GetAngularVelocity(Real dqi,Vector3& omega) const;

  //Jacobian (orientation,position) of a point (in frame 0) with respect to qi
  void GetJacobian(Real qi,const Vector3& p,Vector3& Jo,Vector3& Jp) const;
  void GetOrientationJacobian(Vector3& Jo) const;
  void GetPositionJacobian(Real qi,const Vector3& p,Vector3& Jp) const;

  //Position jacobian of points in frame 0 w.r.t. i 
  void GetJacobian(Real qi,Frame3D& J) const; 
  //Position jacobian of points in frame j w.r.t. i
  void GetJacobian(Real qi,const Frame3D& Tj_World,Frame3D& J) const; 

  Type type;
  Vector3 w;
  Frame3D T0_Parent;
  Frame3D T_World;
};

template <class Link>
class KinematicChain3DTemplate : public Chain
{
public:
  virtual ~KinematicChain3DTemplate() {}

  void Initialize(int numLinks)
  {
    links.resize(numLinks);
    parents.resize(numLinks);
    q.resize(numLinks,Zero);
    qMin.resize(numLinks,-Inf);
    qMax.resize(numLinks,Inf);
  }

  virtual std::string LinkName(int i) const
  {
    char temp[20];
    sprintf(temp,"Link[%d]",i);
    return temp;
  }
  
  inline bool InJointLimits(const Vector& q) const
  {
    assert(q.n == qMin.n);
    for(int i=0;i<qMin.n;i++) {
      if(qMin[i]>q[i] || q[i]>qMax[i]) {
	return false;
      }
    }
    return true;
  }

  //based on the values in q, update the frames T
  void UpdateFrames();

  //in the following, pi is a point in the local frame of body i

  //gets the world location of pi
  void GetWorldPosition(const Vector3& pi, int i, Vector3& p) const;
  //gets the world rotation of link i
  const Matrix3& GetWorldRotation(int i) const;
  void GetWorldRotation_Moment(int i, Vector3& m) const;

  //gets the world velocity/angular velocity of pi, given dq/dt
  void GetWorldVelocity(const Vector3& pi, int i, const Vector& dq, Vector3& dp) const;
  void GetWorldAngularVelocity(int i, const Vector& dq, Vector3& omega) const;
  //derivative of Ri w.r.t. qj
  bool GetWorldRotationDeriv(int i, int j, Matrix3& dR) const;
  bool GetWorldRotationDeriv_Moment(int i, int j, Vector3& dm) const;
  //same as above, but m s.t. Ri=e^[m] is specified
  bool GetWorldRotationDeriv_Moment(int i, int j, const Vector3& m,Vector3& dm) const;

  //gets the jacobian of pi w.r.t qj
  bool GetJacobian(const Vector3& pi, int i, int j, Vector3& dw, Vector3& dv) const;
  bool GetOrientationJacobian(int i, int j, Vector3& dw) const;
  bool GetPositionJacobian(const Vector3& pi, int i, int j, Vector3& dv) const;
  //gets the jacobian of pi w.r.t q (row 0-2 is angular, 3-5 are translational)
  void GetFullJacobian(const Vector3& pi, int i, Matrix& J) const;
  //rows 3-5 of the above
  void GetPositionJacobian(const Vector3& pi, int i, Matrix& J) const;

  std::vector<Link> links;
  Vector q;
  Vector qMin,qMax;
};

typedef KinematicChain3DTemplate<KinematicLink3D> KinematicChain3D;


template <class Link>
void KinematicChain3DTemplate<Link>::UpdateFrames()
{
  //based on the values in q, update the frames T
  //assert(HasValidOrdering());
  Frame3D Ti;
  //get local transform T(i->i)(q)
  //then do the parent transformations to get them to T(i->0)(q)
  //since the chain is top down, we can loop straight through
  //given parent j and T(j->0)(q), we have
  //T(i->0)(q) = T(j->0)(q)*T0(i->j)*T(i->i)(q)
  for(size_t i=0;i<links.size();i++) {
    Link& li = links[i];
    li.GetLocalTransform(q(i),Ti);
    int pi=parents[i];
    if(pi==-1)
      li.T_World.mul(li.T0_Parent,Ti);
    else {
      li.T_World.mul(links[pi].T_World,li.T0_Parent);
      li.T_World*=Ti;
    }
  }
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetWorldPosition(const Vector3& pi, int i, Vector3& p) const
{
  links[i].T_World.mulPoint(pi,p);
}

template <class Link>
const Matrix3& KinematicChain3DTemplate<Link>::GetWorldRotation(int i) const
{
  return links[i].T_World.R;
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetWorldRotation_Moment(int i, Vector3& theta) const
{
  MomentRotation rot;
  rot.setMatrix(links[i].T_World.R);
  theta=rot;
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetWorldVelocity(const Vector3& pi, int i, const Vector& dq, Vector3& dp) const
{
  Vector3 tempp;
  dp.setZero();
  Vector3 p;
  links[i].T_World.mulPoint(pi,p);
  int j=i;
  while(j!=-1) {
    links[j].GetVelocity(q[j],dq[j],p,tempp);
    dp+=tempp;
    j=parents[j];
  }
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetWorldAngularVelocity(int i, const Vector& dq, Vector3& omega) const
{
  Vector3 tempw;
  omega.setZero();
  int j=i;
  while(j!=-1) {
    links[j].GetAngularVelocity(dq[j],tempw);
    omega+=tempw;
    j=parents[j];
  }
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetWorldRotationDeriv(int i, int j, Matrix3& dR) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MatrixDerivative(links[i].T_World.R,dw,dR);
    return true;
  }
  dR.setZero();
  return false;
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetWorldRotationDeriv_Moment(int i, int j, Vector3& dm) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MomentDerivative(links[i].T_World.R,dw,dm);
    return true;
  }
  dm.setZero();
  return false;
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetWorldRotationDeriv_Moment(int i, int j, const Vector3& m,Vector3& dm) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MomentDerivative(m,links[i].T_World.R,dw,dm);
    return true;
  }
  dm.setZero();
  return false;
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetJacobian(const Vector3& pi, int i, int j, Vector3& dw, Vector3& dv) const
{
  if(IsAncestor(i,j)) {
    Vector3 p;
    GetWorldPosition(pi,i,p);
    links[j].GetJacobian(q[j],p,dw,dv);
    return true;
  }
  dw.setZero();
  dv.setZero();
  return false;
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetOrientationJacobian(int i, int j, Vector3& dw) const
{
  if(IsAncestor(i,j)) {
    links[j].GetOrientationJacobian(dw);
    return true;
  }
  dw.setZero();
  return false;
}

template <class Link>
bool KinematicChain3DTemplate<Link>::GetPositionJacobian(const Vector3& pi, int i, int j, Vector3& dv) const
{
  if(IsAncestor(i,j)) {
    Vector3 p;
    GetWorldPosition(pi,i,p);
    links[j].GetPositionJacobian(q[j],p,dv);
    return true;
  }
  dv.setZero();
  return false;
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetFullJacobian(const Vector3& pi, int i, Matrix& J) const
{
  J.resize(6,q.n,Zero);
  Vector3 w; Vector3 v;
  Vector3 p;
  GetWorldPosition(pi,i,p);
  int j=i;
  while(j!=-1) {
    links[j].GetJacobian(q[j],p,w,v);
    J(0,j)=w.x; J(1,j)=w.y; J(2,j)=w.z;
    J(3,j)=v.x; J(4,j)=v.y; J(5,j)=v.z;
    j=parents[j];
  }
}

template <class Link>
void KinematicChain3DTemplate<Link>::GetPositionJacobian(const Vector3& pi, int i, Matrix& J) const
{
  J.resize(3,q.n,Zero);
  Vector3 v;
  Vector3 p;
  GetWorldPosition(pi,i,p);
  int j=i;
  while(j!=-1) {
    links[j].GetPositionJacobian(q[j],p,v);
    J(0,j)=v.x; J(1,j)=v.y; J(2,j)=v.z;
    j=parents[j];
  }
}


#endif

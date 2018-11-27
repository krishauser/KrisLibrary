#ifndef ROBOTICS_KINEMATIC_CHAIN_H
#define ROBOTICS_KINEMATIC_CHAIN_H

#include <KrisLibrary/Logger.h>
#include "Frame.h"
#include "Chain.h"
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <string>

class KinematicLink2D
{
public:
	enum Type { TypeNone, TypeRotation, TypeTranslation, TypeCombination };

	Type GetType() const;
	void SetRotationJoint(Real w);
	void SetTranslationJoint(const Vector2& v);

	//screw's transformation is a function of control qi
	//say we're link i
	//T(i->i)(qi) = R(w*qi)oT(v*qi) where R rotates the given angle, T translates
	//T(i->pi)(qi) = T0(i->pi)*T(i->i)(qi)
	void GetLocalTransform(Real qi,Frame2D& T) const {
		T.set(qi*w,qi*v);
	}

	//evaluations of a point (in frame 0) with respect to qi,dqi
	void GetVelocity(Real qi,Real dqi,const Vector2& p,Vector2& vel) const; 
	void GetAngularVelocity(Real qi,Real dqi,Real& omega) const; 

	//Jacobian (orientation,position) of a point (in frame 0) w.r.t. i
	void GetJacobian(Real qi,const Vector2& p,Real& Jo,Vector2& Jp) const;
	void GetOrientationJacobian(Vector2& Jo) const;
	void GetPositionJacobian(Real qi,const Vector2& p,Vector2& Jp) const;

        //Position jacobian of points in frame 0 w.r.t. i
	void GetJacobian(Real qi,Frame2D& J) const;  
	//Position jacobian of points in frame j w.r.t. i
	void GetJacobian(Real qi,const Frame2D& Tj_World,Frame2D& J) const; 

	Real w; Vector2 v;
	Frame2D T0_Parent;
	Frame2D T_World;
};

template <class Link>
class KinematicChain2DTemplate : public Chain
{
public:
  virtual ~KinematicChain2DTemplate() {}

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
	//pi is a point in the local frame of body i
	//gets the world location of pi
	void GetWorldPosition(const Vector2& pi, int i, Vector2& p) const;
	//gets the world rotation of link i
	void GetWorldRotation(int i, Real& theta) const;
	//gets the world velocity of pi, given dq/dt
	void GetWorldVelocity(const Vector2& pi, int i, const Vector& dq, Vector2& dp) const;
	//gets the world angular velocity of frame of body i
	void GetWorldAngularVelocity(int i, const Vector& dq, Real& dtheta) const;

	//gets the jacobian of pi w.r.t qj
	bool GetJacobian(const Vector2& pi, int i, int j, Real& dw, Vector2& dv) const;
	bool GetOrientationJacobian(int i, int j, Real& dw) const;
	bool GetPositionJacobian(const Vector2& pi, int i, int j, Vector2& dv) const;
	//gets the jacobian of pi w.r.t q (row 0 is angular, 1,2 are translational)
	void GetFullJacobian(const Vector2& pi, int i, Matrix& J) const;
	//rows 1,2 of the above
	void GetPositionJacobian(const Vector2& pi, int i, Matrix& J) const;

  std::vector<Link> links;
  Vector q;
  Vector qMin,qMax;
};

typedef KinematicChain2DTemplate<KinematicLink2D> KinematicChain2D;

template <class Link>
void KinematicChain2DTemplate<Link>::UpdateFrames()
{
	//based on the values in q, update the frames T
	assert(this->HasValidOrdering());
	Frame2D Ti;
	//get local transform T(i->i)(q)
	//then do the parent transformations to get them to T(i->0)(q)
	//since the chain is top down, we can loop straight through
	//given parent j and T(j->0)(q), we have
	//T(i->0)(q) = T(j->0)(q)*T0(i->j)*T(i->i)(q)
	for(unsigned int i=0;i<this->parents.size();i++) {
		links[i].GetLocalTransform(q(i),Ti);
		if(this->parents[i]==-1)
			links[i].T_World=links[i].T0_Parent*Ti;
		else
			links[i].T_World=links[this->parents[i]].T_World*links[i].T0_Parent*Ti;
	}
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetWorldPosition(const Vector2& pi, int i, Vector2& p) const
{
	links[i].T_World.mulPoint(pi,p);
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetWorldRotation(int i, Real& theta) const
{
	Vector2 tmp;
	links[i].T_World.get(theta,tmp);
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetWorldVelocity(const Vector2& pi, int i, const Vector& dq, Vector2& dp) const
{
	Vector2 temp;
	dp.setZero();
	Vector2 p;
	links[i].T_World.mul(pi,p);
	int j=i;
	while(j!=-1) {
		links[j].GetVelocity(q[j],dq[j],p,temp);
		dp+=temp;
		j=this->parents[j];
	}
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetWorldAngularVelocity(int i, const Vector& dq, Real& dtheta) const
{
	Real temp;
	dtheta = Zero;
	int j=i;
	while(j!=-1) {
		links[j].GetAngularVelocity(q[j],dq[j],temp);
		dtheta+=temp;
		j=this->parents[j];
	}
}

template <class Link>
bool KinematicChain2DTemplate<Link>::GetJacobian(const Vector2& pi, int i, int j, Real& dw, Vector2& dv) const
{
	if(this->IsAncestor(i,j)) {
	  Vector2 p;
	  GetWorldPosition(pi,i,p);
	  links[j].GetJacobian(q[j],p,dw,dv);
	  return true;
	}
	dw=Zero;
	dv.setZero();
	return false;
}

template <class Link>
bool KinematicChain2DTemplate<Link>::GetOrientationJacobian(int i, int j, Real& dw) const
{
	if(IsAncestor(i,j)) {
	  links[j].GetOrientationJacobian(dw);
	  return true;
	}
	dw=Zero;
	return false;
}

template <class Link>
bool KinematicChain2DTemplate<Link>::GetPositionJacobian(const Vector2& pi, int i, int j, Vector2& dv) const
{
	if(IsAncestor(i,j)) {
	  Vector2 p;
	  GetWorldPosition(pi,i,p);
	  links[j].GetPositionJacobian(q[j],p,dv);
	  return true;
	}
	dv.setZero();
	return false;
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetFullJacobian(const Vector2& pi, int i, Matrix& J) const
{
	J.resize(3,q.n,Zero);
	Real w; Vector2 v;
	Vector2 p;
	GetWorldPosition(pi,i,p);
	int j=i;
	while(j!=-1) {
		links[j].GetJacobian(q[j],p,w,v);
		J(0,j)=w;
		J(1,j)=v.x;
		J(2,j)=v.y;
		j=parents[j];
	}
}

template <class Link>
void KinematicChain2DTemplate<Link>::GetPositionJacobian(const Vector2& pi, int i, Matrix& J) const
{
	J.resize(3,q.n,Zero);
	Vector2 v;
	Vector2 p;
	GetWorldPosition(pi,i,p);
	int j=i;
	while(j!=-1) {
		links[j].GetPositionJacobian(q[j],p,v);
		J(0,j)=v.x;
		J(1,j)=v.y;
		j=parents[j];
	}
}


#endif

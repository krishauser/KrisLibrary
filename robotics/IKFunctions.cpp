#include <KrisLibrary/Logger.h>
#include "IKFunctions.h"
#include "SelfTest.h"
#include "Rotation.h"
#include "Geometry.h"
#include "Kinematics.h"
#include <Timer.h>
#include <math/misc.h>
#include <math3d/misc.h>
#include <math3d/basis.h>
#include <optimization/Minimization.h>
#include <iostream>
#include <sstream>
#include <set>
using namespace std;

#define TEST_DIFFERENTIATION 0

void EvalIKError(const IKGoal& g,const RigidTransform& T,Real* poserr,Real* orierr)
{
  g.GetError(T,poserr,orierr);
}

void EvalIKError(const IKGoal& g,const RigidTransform& T,Vector& err)
{
  Real poserr[3],orierr[3];
  EvalIKError(g,T,poserr,orierr);
  int m=IKGoal::NumDims(g.posConstraint);
  int n=IKGoal::NumDims(g.rotConstraint);
  for(int i=0;i<m;i++)
    err(i) = poserr[i];
  for(int i=0;i<n;i++)
    err(i+m) = orierr[i];
}

void EvalIKGoalDeriv(const IKGoal& g,const RigidTransform& T,const Vector3& dw,const Vector3& dv,Vector& derr)
{
  Vector3 pv=dv + cross(dw,g.endPosition-T.t);
  if(g.posConstraint == IKGoal::PosFixed) {
    pv.get(derr(0),derr(1),derr(2));
  }
  else if(g.posConstraint == IKGoal::PosLinear) {
    Vector3 xb,yb;
    GetCanonicalBasis(g.direction,xb,yb);
    derr(0) = dot(pv,xb);
    derr(1) = dot(pv,yb);
  }
  else if(g.posConstraint == IKGoal::PosPlanar) {
    derr(0) = dot(pv,g.direction);
  }

  int m=IKGoal::NumDims(g.posConstraint);
  if(g.rotConstraint==IKGoal::RotFixed) {
    Matrix3 TgoalInv,Trel;
    MomentRotation endinv;
    endinv.setNegative(g.endRotation);
    endinv.getMatrix(TgoalInv);    
    Trel.mul(T.R,TgoalInv);
    Vector3 dr;
    MomentDerivative(Trel,dw,dr);
    dr.get(derr(0),derr(1),derr(2));
  }
  else if(g.rotConstraint==IKGoal::RotAxis) {
    Vector3 x,y;
    GetCanonicalBasis(g.endRotation,x,y);
    Vector3 curAxis;
    T.R.mul(g.localAxis,curAxis);
    derr(m) = Sign(dot(curAxis,x))*dot(cross(dw,curAxis),x) - dot(cross(dw,curAxis),g.endRotation);
    derr(m+1) = Sign(dot(curAxis,y))*dot(cross(dw,curAxis),y) - dot(cross(dw,curAxis),g.endRotation);
  }
  else if(g.rotConstraint==IKGoal::RotNone) { }
  else {
    FatalError("EvalIKGoalDeriv(): Invalid number of rotation terms");
  }

}

void GetPositionJacobian(RobotKinematics3D& robot,const Vector3& ploc,int link,const ArrayMapping& activeDofs,Matrix& J)
{
  if(activeDofs.IsOffset()) {
    if(activeDofs.offset==0) {
      robot.GetPositionJacobian(ploc,link,J);
    }
    else {
      Vector3 dp;
      J.setZero();
      int linkmin = activeDofs.offset;
      int linkmax = activeDofs.offset + J.n;
      int j=link;
      while(j >= linkmax && j >= 0)
        j = robot.parents[j];
      Vector3 pw;
      robot.links[link].T_World.mulPoint(ploc,pw);
      while(j >= linkmin && j >= 0) {
        int i=j-linkmin;
        robot.links[j].GetPositionJacobian(robot.q[j],pw,dp);
        J(0,i)=dp.x;
        J(1,i)=dp.y;
        J(2,i)=dp.z;
        j = robot.parents[j];
      }
    }
  }
  else {
    robot.GetPositionJacobian(ploc,link,activeDofs.mapping,J);
  }
}

void GetOrientationJacobian(RobotKinematics3D& robot,int link,const ArrayMapping& activeDofs,Matrix& J)
{
  if(activeDofs.IsOffset()) {
    if(activeDofs.offset==0) {
      robot.GetOrientationJacobian(link,J);
    }
    else {
      Vector3 dw;
      J.setZero();
      int linkmin = activeDofs.offset;
      int linkmax = activeDofs.offset + J.n;
      int j=link;
      while(j >= linkmax && j >= 0)
        j = robot.parents[j];
      while(j >= linkmin && j >= 0) {
        int i=j-linkmin;
        robot.links[j].GetOrientationJacobian(dw);
        J(0,i)=dw.x;
        J(1,i)=dw.y;
        J(2,i)=dw.z;
        j = robot.parents[j];
      }
    }
  }
  else {
    robot.GetOrientationJacobian(link,activeDofs.mapping,J);
  }
}



WorldPositionFunction::WorldPositionFunction(RobotKinematics3D& _robot,const Vector3& _pi,int _i,const ArrayMapping& _active)
  :robot(_robot),ploc(_pi),link(_i),activeDofs(_active)
{}

int WorldPositionFunction::GetDOF(int dim) const
{ return activeDofs.Map(dim); }


void WorldPositionFunction::Eval(const Vector& x, Vector& v)
{
  Assert(v.n==3);
  Vector3 p;
  robot.GetWorldPosition(ploc,link,p);
  v(0) = p.x;
  v(1) = p.y;
  v(2) = p.z;
}

Real WorldPositionFunction::Eval_i(const Vector& x, int i)
{
  Vector3 p;
  robot.GetWorldPosition(ploc,link,p);
  return p[i];
}

void WorldPositionFunction::Jacobian(const Vector& x, Matrix& J)
{
  Assert(J.hasDims(3,x.n));
  GetPositionJacobian(robot,ploc,link,activeDofs,J);
}

//void WorldPositionFunction::Jacobian_i(const Vector& x, int i, Vector& Ji)
//void WorldPositionFunction::Hessian_i(const Vector& x,int i,Matrix& Hi);



IKGoalFunction::IKGoalFunction(RobotKinematics3D& _robot,const IKGoal& _goal,const ArrayMapping& active)
  :robot(_robot),goal(_goal),activeDofs(active),
   positionScale(1),rotationScale(1)
{
  Assert(goal.posConstraint != IKGoal::PosNone || goal.rotConstraint != IKGoal::RotNone);
}

string IKGoalFunction::Label() const
{
  string str="IkGoal";
  str += "[";
  str += robot.LinkName(goal.link);
  str += "]";
  return str;
}

string IKGoalFunction::Label(int i) const
{
  string str=Label();
  if(i < IKGoal::NumDims(goal.posConstraint)) {
    str+="[position]";
  }
  else {
    str+="[rotation]";
  }
  return str;
}

int IKGoalFunction::NumDimensions() const { return IKGoal::NumDims(goal.posConstraint)+IKGoal::NumDims(goal.rotConstraint); }
int IKGoalFunction::GetDOF(int dim) const
{ return activeDofs.Map(dim); }

void IKGoalFunction::PreEval(const Vector& x)
{
  eepos.dirty=true;
  eerot.dirty=true;
  H.dirty = true;
}

void IKGoalFunction::UpdateEEPos()
{
  if(eepos.dirty) {
    robot.GetWorldPosition(goal.localPosition,goal.link,eepos);
    Vector3 dest;
    if(goal.destLink < 0) dest=goal.endPosition;
    else robot.GetWorldPosition(goal.endPosition,goal.destLink,dest);
    eepos -= dest;
    eepos.dirty=false;
  }
}

void IKGoalFunction::UpdateEERot()
{
  if(eerot.dirty) {
    if(goal.rotConstraint==IKGoal::RotFixed) {
      Assert(IsFinite(robot.links[goal.link].T_World.R));
      Assert(IsFinite(goal.endRotation));
      MomentRotation endr(goal.endRotation);
      Matrix3 Rdest;
      endr.getMatrix(Rdest);
      if(goal.destLink < 0) 
	eerot.mulTransposeB(robot.links[goal.link].T_World.R,Rdest);
      else {
	Matrix3 Rdest2;
	Rdest2.mul(robot.links[goal.destLink].T_World.R,Rdest);
	eerot.mulTransposeB(robot.links[goal.link].T_World.R,Rdest2);
      }
    }
    else if(goal.rotConstraint==IKGoal::RotAxis) {
      //robot.links[goal.link].T_World.R.mul(goal.localAxis,eerot);
    }
    eerot.dirty = false;
  }
}



void IKGoalFunction::Eval(const Vector& x, Vector& r)
{
  Assert(r.n == NumDimensions());
  UpdateEEPos();
  if(goal.posConstraint == IKGoal::PosFixed) {
    for(int j=0;j<3;j++) { 
      r(j) = positionScale*eepos[j];
    }
  }
  else if(goal.posConstraint == IKGoal::PosLinear) {
    Vector3 xb,yb;
    Vector3 d;
    if(goal.destLink < 0) d=goal.direction;
    else d=robot.links[goal.destLink].T_World.R*goal.direction;
    GetCanonicalBasis(d,xb,yb);
    r(0) = positionScale*dot(eepos,xb);
    r(1) = positionScale*dot(eepos,yb);
  }
  else if(goal.posConstraint == IKGoal::PosPlanar) {
    Vector3 d;
    if(goal.destLink < 0) d=goal.direction;
    else d=robot.links[goal.destLink].T_World.R*goal.direction;
    r(0) = positionScale*dot(eepos,d);
  }

  UpdateEERot();
  int m=goal.posConstraint;
  if(goal.rotConstraint==IKGoal::RotFixed) {
    MomentRotation em;
    Assert(IsFinite(eerot));
    if(!em.setMatrix(eerot)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"IK: Warning, end effector did not have a valid rotation matrix?\n");
      em.setZero();
    }
    r(m)=rotationScale*em.x;
    r(m+1)=rotationScale*em.y;
    r(m+2)=rotationScale*em.z;
  }
  else if(goal.rotConstraint==IKGoal::RotAxis) {
    Vector3 x,y;
    Vector3 d;
    if(goal.destLink < 0) d=goal.endRotation;
    else d=robot.links[goal.destLink].T_World.R*goal.endRotation;
    GetCanonicalBasis(d,x,y);
    Vector3 curAxis;
    robot.links[goal.link].T_World.R.mul(goal.localAxis,curAxis);
    Real neg = 1.0-curAxis.dot(d);
    r(m) = rotationScale*(Abs(dot(curAxis,x))+neg);
    r(m+1) = rotationScale*(Abs(dot(curAxis,y))+neg);
  }
  else if(goal.rotConstraint==IKGoal::RotNone) { }
  else {
    FatalError("IK(): Invalid number of rotation terms");
  }
}

Real IKGoalFunction::Eval_i(const Vector& x, int i)
{
  if(i<IKGoal::NumDims(goal.posConstraint)) {
    UpdateEEPos();
    if(goal.posConstraint == IKGoal::PosFixed) {
      return positionScale*eepos[i];
    }
    else if(goal.posConstraint == IKGoal::PosLinear) {
      Vector3 xb,yb;
      Vector3 d;
      if(goal.destLink < 0) d=goal.direction;
      else d=robot.links[goal.destLink].T_World.R*goal.direction;
      GetCanonicalBasis(d,xb,yb);
      if(i == 0) return positionScale*dot(eepos,xb);
      else return positionScale*dot(eepos,yb);
    }
    else if(goal.posConstraint == IKGoal::PosPlanar) {
      Vector3 d;
      if(goal.destLink < 0) d=goal.direction;
      else d=robot.links[goal.destLink].T_World.R*goal.direction;
      return positionScale*dot(eepos,d);
    }
  }
  else {
    i-=IKGoal::NumDims(goal.posConstraint);
    Assert(i<IKGoal::NumDims(goal.rotConstraint));
    UpdateEERot();
    if(goal.rotConstraint==IKGoal::RotFixed) {
      MomentRotation em;
      Assert(IsFinite(eerot));
      if(!em.setMatrix(eerot)) {
                LOG4CXX_ERROR(KrisLibrary::logger(),"IK: Warning, end effector did not have a valid rotation matrix?\n");
        em.setZero();
      }
      return rotationScale*em[i];
      //return eerot[i];
    }
    else if(goal.rotConstraint==IKGoal::RotAxis) {
      Vector3 x,y;
      Vector3 d;
      if(goal.destLink < 0) d=goal.endRotation;
      else d=robot.links[goal.destLink].T_World.R*goal.endRotation;
      GetCanonicalBasis(d,x,y);
      Vector3 curAxis;
      robot.links[goal.link].T_World.R.mul(goal.localAxis,curAxis);
      Real neg = 1.0-curAxis.dot(d);
      if(i==0)
	return rotationScale*(Abs(dot(curAxis,x))+neg);
      else
	return rotationScale*(Abs(dot(curAxis,y))+neg);
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"IK(): Invalid number of rotation terms\n");
      Abort(); 
    }
  }
  return Zero;
}

void IKGoalFunction::Jacobian(const Vector& x, Matrix& J)
{
  UpdateEERot();
  Jptemp.resize(3,x.n);
  GetPositionJacobian(robot,goal.localPosition,goal.link,activeDofs,Jptemp);
  if(goal.destLink >= 0) {
    Jptemp2.resize(3,x.n);
    GetPositionJacobian(robot,goal.endPosition,goal.destLink,activeDofs,Jptemp2);
  }
  if(goal.rotConstraint!=IKGoal::RotNone) {
    Jotemp.resize(3,x.n);
    GetOrientationJacobian(robot,goal.link,activeDofs,Jotemp);
    if(goal.destLink >= 0) {
      Jotemp2.resize(3,x.n);
      GetOrientationJacobian(robot,goal.destLink,activeDofs,Jotemp2);
    }
  }
  for(int k=0;k<x.n;k++) {
    int baseLink = GetDOF(k);
    Vector3 dp(Jptemp(0,k),Jptemp(1,k),Jptemp(2,k));
    //Vector3 dp
    //robot.GetPositionJacobian(goal.localPosition,goal.link,baseLink,dp);
    if(goal.posConstraint==IKGoal::PosFixed) {
      for(int j=0;j<3;j++)
	J(j,k) = positionScale*dp[j];
    }
    else if(goal.posConstraint == IKGoal::PosLinear) {
      Vector3 xb,yb;
      Vector3 d;
      if(goal.destLink < 0) d=goal.direction;
      else d=robot.links[goal.destLink].T_World.R*goal.direction;
      GetCanonicalBasis(d,xb,yb);
      J(0,k) = positionScale*dot(dp,xb);
      J(1,k) = positionScale*dot(dp,yb);
    }
    else if(goal.posConstraint == IKGoal::PosPlanar) {
      Vector3 d;
      if(goal.destLink < 0) d=goal.direction;
      else d=robot.links[goal.destLink].T_World.R*goal.direction;
      J(0,k) = positionScale*dot(dp,d);
    }

    if(goal.destLink >= 0) {
      Vector3 dpdest(Jptemp2(0,k),Jptemp2(1,k),Jptemp2(2,k));
      //Vector3 dpdest;
      //robot.GetPositionJacobian(goal.endPosition,goal.destLink,baseLink,dpdest);
      if(goal.posConstraint==IKGoal::PosFixed) {
	for(int j=0;j<3;j++)
	  J(j,k) -= positionScale*dpdest[j];
      }
      else if(goal.posConstraint == IKGoal::PosLinear) {
	FatalError("TODO: link-to-link fancy constraints");
	Vector3 xb,yb;
	Vector3 d;
	if(goal.destLink < 0) d=goal.direction;
	else d=robot.links[goal.destLink].T_World.R*goal.direction;
	GetCanonicalBasis(d,xb,yb);
	J(0,k) -= positionScale*dot(dpdest,xb);
	J(1,k) -= positionScale*dot(dpdest,yb);
      }
      else if(goal.posConstraint == IKGoal::PosPlanar) {
	FatalError("TODO: link-to-link fancy constraints");
	Vector3 d;
	if(goal.destLink < 0) d=goal.direction;
	else d=robot.links[goal.destLink].T_World.R*goal.direction;
	J(0,k) -= positionScale*dot(dpdest,d);
      }
    }

    int m=IKGoal::NumDims(goal.posConstraint);
    Vector3 dr,dw;
    if(goal.rotConstraint==IKGoal::RotFixed) {
      dw.set(Jotemp(0,k),Jotemp(1,k),Jotemp(2,k));
      //robot.GetOrientationJacobian(goal.link,baseLink,dw);
      if(goal.destLink >= 0) {
	//m = moment(Rdiff)
	//dRdiff = d/dt(R1(q)Rl^T R2^T(q))
	//       = dR1/dt(q)Rl^TR2^T(q) + R1(q)Rl^TdR2/dt^T(q)
	//       = [w1]R1 Rl^T R2^T + R1 Rl^T R2^T [-w2]
	//       = [w1]R1 Rl^T R2^T - [R1 Rl^T R2^R w2] R1 Rl^T R2^T
	//assume dRdiff = [w]Rdiff, then w = w1-eerot*w2
	Vector3 dwdest;
        dwdest.set(Jotemp2(0,k),Jotemp2(1,k),Jotemp2(2,k));
	//robot.GetOrientationJacobian(goal.destLink,baseLink,dwdest);
	dw -= eerot*dwdest;
      }
      MomentDerivative(eerot,dw,dr);
      //robot.GetWorldRotationDeriv_Moment(goal.link,baseLink,dr);
      J(m,k) = rotationScale*dr.x;
      J(m+1,k) = rotationScale*dr.y;
      J(m+2,k) = rotationScale*dr.z;
    }
    else if(goal.rotConstraint==IKGoal::RotAxis) {
      Vector3 x,y;
      Vector3 d;
      if(goal.destLink < 0) d=goal.endRotation;
      else d = robot.links[goal.destLink].T_World.R*goal.endRotation;
      if(goal.destLink >= 0) 
	FatalError("TODO: link-to-link fancy constraints");
      GetCanonicalBasis(d,x,y);
      dr.set(Jotemp(0,k),Jotemp(1,k),Jotemp(2,k));
      //robot.GetOrientationJacobian(goal.link,baseLink,dr);
      Vector3 curAxis;
      robot.links[goal.link].T_World.R.mul(goal.localAxis,curAxis);
      Vector3 axisRateOfChange = cross(dr,curAxis);
      J(m,k) = rotationScale*(Sign(dot(curAxis,x))*dot(axisRateOfChange,x)-dot(axisRateOfChange,d));
      J(m+1,k) = rotationScale*(Sign(dot(curAxis,y))*dot(axisRateOfChange,y)-dot(axisRateOfChange,d));
    }
    else if(goal.rotConstraint==IKGoal::RotNone) {
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"GetIKJacobian(): Invalid number of rotation terms\n");
      Abort(); 
    }
  }
}

void IKGoalFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Vector3 dv;
  UpdateEERot();
  for(int k=0;k<x.n;k++) {
    int baseLink = GetDOF(k);
    if(i<IKGoal::NumDims(goal.posConstraint)) {
      robot.GetPositionJacobian(goal.localPosition,goal.link,baseLink,dv);
      if(goal.posConstraint==IKGoal::PosFixed) {
	Ji(k) = positionScale*dv[i];
      }
      else if(goal.posConstraint == IKGoal::PosLinear) {
	Vector3 xb,yb;
	Vector3 d;
	if(goal.destLink < 0) d=goal.direction;
	else d=robot.links[goal.destLink].T_World.R*goal.direction;
	GetCanonicalBasis(d,xb,yb);
	if(i==0) Ji(k) = positionScale*dot(dv,xb);
	else Ji(k) = positionScale*dot(dv,yb);
      }
      else if(goal.posConstraint == IKGoal::PosPlanar) {
	Vector3 d;
	if(goal.destLink < 0) d=goal.direction;
	else d=robot.links[goal.destLink].T_World.R*goal.direction;
	Ji(k) = positionScale*dot(dv,d);
      }

      if(goal.destLink >= 0) {
	//TODO
	FatalError("TODO");
      }
    }
    else {
      robot.GetWorldRotationDeriv_Moment(goal.link,baseLink,dv);
      int m=i-IKGoal::NumDims(goal.posConstraint);
      if(goal.rotConstraint==IKGoal::RotFixed) {
	Vector3 dw;
	robot.GetOrientationJacobian(goal.link,baseLink,dw);
	MomentDerivative(eerot,dw,dv);
	Ji(k)=rotationScale*dv[m];
      }
      else if(goal.rotConstraint==IKGoal::RotAxis) {
	Vector3 x,y;
	Vector3 d;
	if(goal.destLink < 0) d=goal.endRotation;
	else d=robot.links[goal.destLink].T_World.R*goal.endRotation;
	d.getOrthogonalBasis(x,y);
	robot.GetOrientationJacobian(goal.link,baseLink,dv);
	Vector3 curAxis;
	robot.links[goal.link].T_World.R.mul(goal.localAxis,curAxis);
	Vector3 axisRateOfChange = cross(dv,curAxis);
	if(m==0)
	  Ji(k)=rotationScale*(dot(axisRateOfChange,x)-dot(axisRateOfChange,d));
	else
	  Ji(k)=rotationScale*(dot(axisRateOfChange,y)-dot(axisRateOfChange,d));
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"GetIKJacobian(): Invalid number of rotation terms\n");
	Abort(); 
      }

      if(goal.destLink >= 0) {
	//TODO
	FatalError("TODO");
      }
    }
  }
}

void IKGoalFunction::Hessian_i(const Vector& x,int component,Matrix& Hi)
{
  FatalError("Does orientation have a hessian???");
  if(H.dirty) {
    if((int)H.size() != NumDimensions()) {
      H.resize(NumDimensions());
      for(int i=0;i<NumDimensions();i++) {
	H[i].resize(x.n,x.n,Zero);
      }
    }

    Vector3 ddr,ddp;
    //NOTE: H(i,j) != 0 only if i,j are ancestors of k
    //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
    UpdateEERot();
    for(int i=0;i<x.n;i++) {
      int ilink = GetDOF(i);
      for(int j=i;j<x.n;j++) {
	int jlink = GetDOF(j);
	if(robot.GetJacobianDeriv(goal.localPosition,goal.link,ilink,jlink,ddr,ddp)) {
	  if(goal.posConstraint == IKGoal::PosFixed) {
	    for(int k=0;k<3;k++)
	      H[k](i,j) = positionScale*ddp[k];
	  }
	  else if(goal.posConstraint == IKGoal::PosLinear) {
	    Vector3 xb,yb;
	    goal.direction.getOrthogonalBasis(xb,yb);
	    H[0](i,j) = positionScale*dot(ddp,xb);
	    H[1](i,j) = positionScale*dot(ddp,yb);
	  }
	  else if(goal.posConstraint == IKGoal::PosPlanar) {
	    H[0](i,j) = positionScale*dot(ddp,goal.direction);
	  }

	  int m=goal.posConstraint;
	  //!!! Why half??? !!!
	  if(goal.rotConstraint==IKGoal::RotFixed) {
	    H[m](i,j) = Half*rotationScale*ddr.x;
	    H[m+1](i,j) = Half*rotationScale*ddr.y;
	    H[m+2](i,j) = Half*rotationScale*ddr.z;
	  }
	  else if(goal.rotConstraint==IKGoal::RotAxis) {
	    Vector3 x,y;
	    goal.endRotation.getOrthogonalBasis(x,y);
	    Vector3 curAxis;
	    robot.links[goal.link].T_World.R.mul(goal.localAxis,curAxis);
	    H[m](i,j) = rotationScale*dot(cross(ddr,curAxis),x);
	    H[m+1](i,j) = rotationScale*dot(cross(ddr,curAxis),y);
	  }
	  else if(goal.rotConstraint==IKGoal::RotNone) {
	  }
	  else {
	    LOG4CXX_INFO(KrisLibrary::logger(),"GetIKJacobian(): Invalid number of rotation terms\n");
	    Abort(); 
	  }
	}
	else {
	  for(size_t k=0;k<H.size();k++)
	    H[k](i,j)=Zero;
	}
      }
    }
    H.dirty = false;
  }

  for(int i=0;i<x.n;i++) {
    Hi(i,i) = H[component](i,i);
    for(int j=i+1;j<x.n;j++) {
      Hi(j,i) = Hi(i,j) = H[component](i,j);
    }
  }
}



RobotCOMFunction::RobotCOMFunction(RobotKinematics3D& _robot,const Vector2& com,const ArrayMapping& active)
  :robot(_robot),comGoal(com),activeDofs(active),comScale(1)
{
  mtotal = robot.GetTotalMass();
}

int RobotCOMFunction::GetDOF(int dim) const
{ return activeDofs.Map(dim); }


string RobotCOMFunction::Label() const
{
  return "COM";
}

string RobotCOMFunction::Label(int i) const
{
  if(i==0)
    return "COMx";
  else
    return "COMy";
}

void RobotCOMFunction::PreEval(const Vector& x)
{
  if(Hx.isEmpty())
    Hx.resize(x.n,x.n,Zero);
  if(Hy.isEmpty())
    Hy.resize(x.n,x.n,Zero);
  Hx.dirty = true;
  Hy.dirty = true;
}

void RobotCOMFunction::Eval(const Vector& x, Vector& r)
{
  Vector3 com = robot.GetCOM();
  r(0) = comScale*(com.x - comGoal.x);
  r(1) = comScale*(com.y - comGoal.y);
}

Real RobotCOMFunction::Eval_i(const Vector& x, int i)
{
  Vector3 com = robot.GetCOM();
  if(i==0) return comScale*(com.x-comGoal.x);
  else if(i==1) return comScale*(com.y-comGoal.y);
  else Abort();
  return Zero;
}

void RobotCOMFunction::Jacobian(const Vector& x, Matrix& J)
{  
  Vector3 dcom;
  Vector3 dp;
  for(int j=0;j<x.n;j++) {
    int baseLink = GetDOF(j);
    dcom.setZero();
    for(int k=0;k<(int)robot.links.size();k++) {
      robot.GetPositionJacobian(robot.links[k].com,k,baseLink,dp);
      dcom += robot.links[k].mass*dp;
    }
    dcom *= comScale/mtotal;
    J(0,j) = dcom.x;
    J(1,j) = dcom.y;
  }
}

void RobotCOMFunction::Jacobian_i(const Vector& x, int i, Vector& Ji)
{
  Vector3 dcom;
  Vector3 dp;
  for(int j=0;j<x.n;j++) {
    int baseLink = GetDOF(j);
    dcom.setZero();
    for(int k=0;k<(int)robot.links.size();k++) {
      robot.GetPositionJacobian(robot.links[k].com,k,baseLink,dp);
      dcom += robot.links[k].mass*dp;
    }
    if(i==0) Ji(j)=dcom.x*comScale/mtotal;
    else if(i==1) Ji(j)=dcom.y*comScale/mtotal;
    else Abort();
  }
}

void RobotCOMFunction::Hessian_i(const Vector& x,int component,Matrix& Hi)
{
  if(Hx.dirty) {
    //Copied from DynamicChain.cpp
    Vector3 ddtheta,ddp;
    //NOTE: H(i,j) != 0 only if i,j are ancestors of k
    //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
    for(int i=0;i<x.n;i++) {
      int ilink = GetDOF(i);
      for(int j=i;j<x.n;j++) {
	int jlink = GetDOF(j);
	Hx(i,j) = Zero;
	Hy(i,j) = Zero;
	for(int k=0;k<robot.q.n;k++) {
	  if(robot.GetJacobianDeriv(robot.links[k].com,k,ilink,jlink,ddtheta,ddp)) {
	    Hx(i,j) += robot.links[k].mass*ddp.x;
	    Hy(i,j) += robot.links[k].mass*ddp.y;
	  }
	}
	Hx(i,j) *= comScale/mtotal;
	Hy(i,j) *= comScale/mtotal;
      }
    }
    Hx.dirty = false;
    Hy.dirty = false;
  }
  //fill in lower triangle
  for(int i=0;i<x.n;i++) {
    for(int j=i;j<x.n;j++) {
      if(component == 0)
	Hi(j,i) = Hi(i,j) = Hx(i,j);
      else
	Hi(j,i) = Hi(i,j) = Hy(i,j);
    }
  }
}


RobotIKFunction::RobotIKFunction(RobotKinematics3D& _r)
  :robot(_r)
{
  activeDofs.imax = robot.q.n;
}

RobotIKFunction::~RobotIKFunction()
{
  Clear();
}

void RobotIKFunction::Clear()
{
  functions.clear();
}

void RobotIKFunction::UseIK(const IKGoal& g)
{
  functions.push_back(make_shared<IKGoalFunction>(robot,g,activeDofs));

#if TEST_DIFFERENTIATION
  Vector x;
  x.resize(activeDofs.Size());
  GetState(x);
  
  /*
  WorldPositionFunction f(robot,ik[0].localPosition,ik[0].link,ik.activeDofs);
  LOG4CXX_INFO(KrisLibrary::logger(),"WorldPositionFunction:");
  TestJacobian(&f,x,0.001,0.001);
  */
  LOG4CXX_INFO(KrisLibrary::logger(),"Differentiation test for RobotIKFunction:");
  TestJacobian(this,x,0.001,0.001,0.001);
#endif
}

void RobotIKFunction::UseIK(const vector<IKGoal>& ik)
{
  if(ik.empty()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, IK problem is empty");
    return;
  }
  functions.reserve(functions.size()+ik.size());
  for(size_t i=0;i<ik.size();i++)
    functions.push_back(make_shared<IKGoalFunction>(robot,ik[i],activeDofs));
  Assert(NumDimensions() > 0);

#if TEST_DIFFERENTIATION
  Vector x;
  x.resize(activeDofs.Size());
  GetState(x);
  
  /*
  WorldPositionFunction f(robot,ik[0].localPosition,ik[0].link,ik.activeDofs);
  LOG4CXX_INFO(KrisLibrary::logger(),"WorldPositionFunction:");
  TestJacobian(&f,x,0.001,0.001);
  */
  LOG4CXX_INFO(KrisLibrary::logger(),"Differentiation test for RobotIKFunction:");
  TestJacobian(this,x,0.001,0.001,0.001);
#endif
}

void RobotIKFunction::UseCOM(const Vector2& comGoal)
{
  functions.push_back(make_shared<RobotCOMFunction>(robot,comGoal,activeDofs));
}

void RobotIKFunction::UseAffineConstraint(const std::vector<int>& links,const std::vector<Real>& coeffs,Real value)
{
  functions.push_back(make_shared<RobotAffineConstraintFunction>(robot,links,coeffs,value,activeDofs));
}

void RobotIKFunction::UseAffineConstraint(const std::vector<int>& links,const std::vector<Real>& scaling,const std::vector<Real>& offset)
{
  vector<int> used;
  vector<int> unused;
  for(size_t j=0;j<links.size();j++) {
    if(activeDofs.InDomain(links[j])) 
      used.push_back(j);
    else
      unused.push_back(j);
  }
  if(used.size()==0) return; //nothing to be constrained
  vector<int> clinks(2);
  vector<Real> coeffs(2);
  Real value;
  for(size_t j=0;j+1<used.size();j++) {
    int a=used[j];
    int b=used[j+1];
    clinks[0] = links[a];
    clinks[1] = links[b];
    coeffs[0] = scaling[b];
    coeffs[1] = -scaling[a];
    value = scaling[a]*offset[b] - scaling[b]*offset[a];
    UseAffineConstraint(clinks,coeffs,value);
  }
  clinks.resize(1);
  coeffs.resize(1);
  for(size_t j=0;j<unused.size();j++) {
    int a=unused[j];
    int b=used[0];
    clinks[0] = links[b];
    //q[a] = coeffs[0]*q[clinks[0]] - value
    //v = q[b]/scaling[b]-offset[b]/scaling[b]
    //q[a] = scaling[a]*v + offset[a] =>
    //q[a] = scaling[a]/scaling[b]*q[b] + offset[a] - offset[b]*scaling[a]/scaling[b]
    coeffs[0] = scaling[a]/scaling[b];
    value = -offset[a] + offset[b]*scaling[a]/scaling[b];
    determinedDofs[links[a]]= make_shared<RobotAffineConstraintFunction>(robot,clinks,coeffs,value,activeDofs);
  }
}

void RobotIKFunction::GetState(Vector& x) const
{
  Assert(x.n == activeDofs.Size());
  activeDofs.InvMap(robot.q,x);
}

void RobotIKFunction::SetState(const Vector& x) const
{
  Assert(x.n == (int)activeDofs.Size());
  if(x.n*2 < robot.q.n) { // save some time if only a few DOF are updated
    Config qnew = robot.q;
    activeDofs.Map(x,qnew);
    for(auto i:determinedDofs)
      qnew[i.first] = i.second->Eval_i(x,0);
    robot.ChangeConfig(qnew);
  }
  else {
    activeDofs.Map(x,robot.q);
    for(auto i:determinedDofs)
      robot.q[i.first] = i.second->Eval_i(x,0);
    robot.UpdateFrames();
  }
  //TODO: figure out all the intermediate frames that might be affected
  //for(size_t i=0;i<activeDofs.mapping.size();i++) 
  //robot.UpdateSelectedFrames(activeDofs.mapping[i],activeDofs.mapping[i]);
}

void RobotIKFunction::PreEval(const Vector& x)
{
  SetState(x);
  CompositeVectorFieldFunction::PreEval(x);
}


RobotAffineConstraintFunction::RobotAffineConstraintFunction(const RobotKinematics3D& _robot,
      const std::vector<int>& _links,
      const std::vector<Real>& _coeffs,
      Real _value,
      const ArrayMapping& _activeDofs)
  :activeDofs(_activeDofs)
{
  Assert(_links.size()==_coeffs.size());
  indices.reserve(_links.size());
  coeffs.reserve(_links.size());
  value = _value;
  for(size_t i=0;i<_links.size();i++) {
    if(activeDofs.InDomain(_links[i])) {
      indices.push_back(activeDofs.InvMap(_links[i]));
      coeffs.push_back(_coeffs[i]);
    }
    else {  //not in active set, consider fixed
      value -= _robot.q[i]*_coeffs[i];
    }
  }
  if(indices.empty()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Affine IK constraint has no valid entries in active DOFs");
  }
}
int RobotAffineConstraintFunction::GetDOF(int dim) const
{
  return activeDofs.Map(dim);
}
std::string RobotAffineConstraintFunction::Label() const
{
  std::stringstream ss;
  ss<<"Affine ";
  for(size_t i=0;i<indices.size();i++)
    ss<<indices[i]<<" ";
  return ss.str();
}
std::string RobotAffineConstraintFunction::Label(int i) const
{
  return Label();
}
void RobotAffineConstraintFunction::Eval(const Vector& x, Vector& r)
{
  Assert(r.n == 1);
  r[0] = Eval_i(x,0);
}
Real RobotAffineConstraintFunction::Eval_i(const Vector& x, int i)
{
  Assert(x.n == activeDofs.Size());
  Real rhs = 0;
  for(size_t j=0;j<indices.size();j++)
    rhs += coeffs[j]*x[indices[j]];
  return rhs - value;
}

void RobotAffineConstraintFunction::Jacobian(const Vector& x, Matrix& J)
{
  Assert(J.m == 1);
  Assert(J.n == x.n);
  Vector v;
  J.getRowRef(0,v);
  Jacobian_i(x,0,v);
}

void RobotAffineConstraintFunction::Jacobian_i(const Vector& x, int i, Vector& Ji)
{
  Ji.setZero();
  for(size_t j=0;j<indices.size();j++)
    Ji[indices[j]] = coeffs[j];
}

void RobotAffineConstraintFunction::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  Hi.setZero();
}








RobotIKSolver::RobotIKSolver(RobotIKFunction& f)
  :solver(&f),function(f),robot(f.robot)
{
  solver.svd.preMultiply = false;
}

void RobotIKSolver::UseJointLimits(Real revJointThreshold) 
{
  //limits
  int n=(int)function.activeDofs.Size();
  solver.bmin.resize(n);
  solver.bmax.resize(n);
  function.activeDofs.InvMap(robot.qMin,solver.bmin);
  function.activeDofs.InvMap(robot.qMax,solver.bmax);
  if(revJointThreshold <= TwoPi) {
    //remove joint limits if revolute and angle > threshold
    for(int i=0;i<n;i++) {
      int dof = function.activeDofs.Map(i);
      if(robot.links[dof].type == RobotLink3D::Revolute) {
	if(solver.bmax[i] - solver.bmin[i] >= revJointThreshold) {
	  solver.bmax[i] = Inf;
	  solver.bmin[i] = -Inf;
	}
      }
    }
  }
}

void RobotIKSolver::UseJointLimits(const Vector& qmin,const Vector& qmax) 
{
  int n=(int)function.activeDofs.Size();
  solver.bmin.resize(n);
  solver.bmax.resize(n);
  function.activeDofs.InvMap(qmin,solver.bmin);
  function.activeDofs.InvMap(qmax,solver.bmax);
}

void RobotIKSolver::UseBiasConfiguration(const Vector& qdesired)
{
  if(qdesired.empty()) solver.bias.clear();
  else {
    int n=(int)function.activeDofs.Size();
    solver.bias.resize(n);
    function.activeDofs.InvMap(qdesired,solver.bias);
  }
}


void RobotIKSolver::ClearJointLimits()
{
  solver.bmin.clear();
  solver.bmax.clear();
}

void RobotIKSolver::RobotToState()
{
  solver.x.resize(function.activeDofs.Size());
  function.GetState(solver.x);
}

void RobotIKSolver::StateToRobot()
{
  function.SetState(solver.x);
  robot.NormalizeAngles(robot.q);
}

bool RobotIKSolver::Solve(Real tolerance,int& iters)
{
  RobotToState();
  solver.tolf = tolerance;
  solver.tolx = tolerance*0.01;
  bool res=solver.GlobalSolve(iters);
  StateToRobot();

  //Vector v(function.NumDimensions());
  //function.Eval(robot.q,v);
  return res;
}


bool RobotIKSolver::MinimizeResidual(Real tolerance,Real delta_tolerance,int& iters)
{
  RobotToState();
  NormSquaredScalarFieldFunction h;
  Compose_SF_VF_Function gnorm(&h,&function);
  Optimization::BCMinimizationProblem problem(&gnorm);
  problem.verbose = solver.verbose;
  problem.bmin = solver.bmin;
  problem.bmax = solver.bmax;
  //approximate hessian with J^T*J + 0.1*I
  function.SetState(solver.x);
  Matrix J;
  function.Jacobian(solver.x,J);
  problem.H.mulTransposeA(J,J);
  problem.H *= 2;
  for(int i=0;i<solver.x.n;i++)
    problem.H(i,i) += 0.1;
  problem.tolf = Sqr(tolerance); //squared tolerance
  problem.tolx = tolerance*0.01;
  problem.tolgrad = delta_tolerance;
  problem.x = solver.x;
  ConvergenceResult res = problem.SolveQuasiNewton(iters);
  solver.x = problem.x;
  StateToRobot();
  if(problem.fx <= Sqr(tolerance)) return true;
  return false;
}

bool RobotIKSolver::PrioritizedSolve(ScalarFieldFunction& objective,Real tolerance,Real delta_tolerance,int& iters)
{
  int maxiters = iters;
  if(!MinimizeResidual(tolerance,delta_tolerance,iters)) return false;
  if(iters==maxiters) return false;
  //iters was the # of iterations used
  int iters_minimize = iters;
  iters = maxiters - iters_minimize;
  //RobotToState();  // not needed: solver.x already contains solution
  Optimization::ConstrainedMinimizationProblem problem(&objective,&function,NULL);
  problem.verbose = solver.verbose;
  problem.bmin = solver.bmin;
  problem.bmax = solver.bmax;
  problem.tolc = tolerance;
  problem.tolx = tolerance*0.01;
  problem.tolgrad = delta_tolerance;
  problem.x = solver.x;
  //ConvergenceResult res = problem.SolveAugmentedLangrangian(iters);
  ConvergenceResult res = problem.SolveSQP(iters);
  solver.x = problem.x;
  iters += iters_minimize;
  StateToRobot();
  if(problem.fx <= Sqr(tolerance)) return true;
  return false;
}

void RobotIKSolver::PrintStats()
{
  /*
  LOG4CXX_INFO(KrisLibrary::logger(),""<<robotIK.numEvals<<" evals ("<<robotIK.evalTime<<"s), "<<robotIK.numJacobians<<" jacobian evals("<<robotIK.jacobianTime);
  LOG4CXX_INFO(KrisLibrary::logger(),""<<robotIK.setStateTime);
  */
  LOG4CXX_INFO(KrisLibrary::logger(),"TODO: record IK solver stats...");
  //Abort();
}





void GetDefaultIKDofs(const RobotKinematics3D& robot,const vector<IKGoal>& ik,ArrayMapping& m)
{
  set<int> dofs;
  for(size_t i=0;i<ik.size();i++) {
    if(ik[i].destLink >= 0) {  //take into account the branching structure
      int lca=robot.LCA(ik[i].link,ik[i].destLink);
      int d=ik[i].link;
      Assert(d >= 0 && d < (int)robot.parents.size());
      while(d!=lca) {
	if(robot.qMin(d) != robot.qMax(d))
	  dofs.insert(d);
	d = robot.parents[d];
      }
      d=ik[i].destLink;
      Assert(d >= 0 && d < (int)robot.parents.size());
      while(d!=lca) {
	if(robot.qMin(d) != robot.qMax(d))
	  dofs.insert(d);
	d = robot.parents[d];
      }
    }
    else {
      int d=ik[i].link;
      Assert(d >= 0 && d < (int)robot.parents.size());
      while(d>=0 && dofs.count(d)==0) {
	if(robot.qMin(d) != robot.qMax(d))
	  dofs.insert(d);
	d = robot.parents[d];
      }
    }
  }
  m.mapping.resize(dofs.size());
  copy(dofs.begin(),dofs.end(),m.mapping.begin());
}

void GetPassiveChainDOFs(const RobotKinematics3D& robot,const IKGoal& g,ArrayMapping& passiveDofs)
{
  GetPassiveChainDOFs(robot,g.link,g.posConstraint+g.rotConstraint,passiveDofs);
}

void GetPassiveChainDOFs(const RobotKinematics3D& robot,int link,int numTerms,ArrayMapping& passiveDofs)
{
  passiveDofs.mapping.resize(numTerms);
  int d = link;
  Assert(d >= 0 && d < (int)robot.parents.size());
  for(int i=0;i<numTerms;i++) {
    if(robot.qMin(d) != robot.qMax(d))
      passiveDofs.mapping[i]=d;
    else
      i--;
    d = robot.parents[d];
  }
}


bool SolveIK(RobotIKFunction& f,
	     Real tolerance,int& iters,int verbose)
{
  if(verbose >= 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"SolveIK(tol="<<tolerance<<",iters="<<iters<<")");
    Timer timer;
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    if(solver.Solve(tolerance,iters)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Succeeded! "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return true;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Failed... "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return false;
    }
  }
  else {
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    return solver.Solve(tolerance,iters);
  }
}

bool SolveIK(RobotKinematics3D& robot,const vector<IKGoal>& problem,
	     Real tolerance,int& iters,int verbose)
{
  RobotIKFunction function(robot);
  function.UseIK(problem);
  GetDefaultIKDofs(robot,problem,function.activeDofs);
  return SolveIK(function,tolerance,iters,verbose);
}

bool SolveIK(RobotKinematics3D& robot,const vector<IKGoal>& problem,
	     const vector<int>& activeDofs,
	     Real tolerance,int& iters,int verbose)
{
  RobotIKFunction function(robot);
  function.UseIK(problem);
  function.activeDofs.mapping = activeDofs;
  return SolveIK(function,tolerance,iters,verbose);
}

bool SolvePassiveChainIK(RobotKinematics3D& robot,const IKGoal& goal,
			 Real tolerance,int& iters,int verbose)
{
  RobotIKFunction function(robot);
  function.UseIK(goal);
  GetPassiveChainDOFs(robot,goal,function.activeDofs);
  return SolveIK(function,tolerance,iters,verbose);
}


bool MinimizeIK(RobotIKFunction& f,
	     Real tolerance,Real delta_tolerance,int& iters,int verbose)
{
  if(verbose >= 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"MinimizeIK(tol="<<tolerance<<",delta_tol="<<delta_tolerance<<"iters="<<iters<<")");
    Timer timer;
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    if(solver.MinimizeResidual(tolerance,delta_tolerance,iters)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Succeeded! "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return true;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Failed... "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return false;
    }
  }
  else {
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    return solver.MinimizeResidual(tolerance,delta_tolerance,iters);
  }
}


bool PrioritizedIK(RobotIKFunction& f,VectorFieldFunction& g,
       Real tolerance,Real delta_tolerance,int& iters,int verbose)
{
  NormSquaredScalarFieldFunction h;
  Compose_SF_VF_Function gnorm(&h,&g);
  return PrioritizedIK(f,gnorm,tolerance,delta_tolerance,iters,verbose);
}

bool PrioritizedIK(RobotIKFunction& f,ScalarFieldFunction& g,
       Real tolerance,Real delta_tolerance,int& iters,int verbose)
{
    if(verbose >= 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"MinimizeIK(tol="<<tolerance<<",delta_tol="<<delta_tolerance<<"iters="<<iters<<")");
    Timer timer;
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    if(solver.PrioritizedSolve(g,tolerance,delta_tolerance,iters)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Succeeded! "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return true;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"    Failed... "<< timer.ElapsedTime());
      if(verbose >= 2) solver.PrintStats();
      return false;
    }
  }
  else {
    RobotIKSolver solver(f);
    solver.UseJointLimits(TwoPi);
    solver.solver.verbose = verbose;
    return solver.PrioritizedSolve(g,tolerance,delta_tolerance,iters);
  }
}



bool IsReachableGoal(const RobotKinematics3D& robot,const IKGoal& a, const IKGoal& b)
{
  Real Lmax = MaxJointDistance(robot,a.link,b.link);
  return IsReachableGoal(a,b,Lmax);
}

bool IsReachableGoal(const IKGoal& a,const IKGoal& b,Real Lmax)
{
  if(a.posConstraint != IKGoal::PosFixed || b.posConstraint != IKGoal::PosFixed) {
    LOG4CXX_WARN(KrisLibrary::logger(),"IsReachableGoal(): Warning: unable to calculate reachability of sliding IK target");
    return true;
  }
  if(b.rotConstraint > a.rotConstraint) return IsReachableGoal(b,a,Lmax);
  if(a.rotConstraint == IKGoal::RotTwoAxis || b.rotConstraint == IKGoal::RotTwoAxis) {
    FatalError("Can't yet do two-axis rotations");
  }

  if(a.rotConstraint == IKGoal::RotFixed) {
    //a's limb is fixed, find the transform to find the joint center
    Matrix3 R;
    MomentRotation mr(a.endRotation);
    mr.getMatrix(R);

    Vector3 ja = a.endPosition - R*a.localPosition;

    if(b.rotConstraint == IKGoal::RotFixed) {
      //fixed/fixed constraint
      //we can find b's joint center too
      mr.set(b.endRotation);
      mr.getMatrix(R);

      Vector3 jb = b.endPosition - R*b.localPosition;
      Real d=(jb-ja).norm();
      return (d <= Lmax);
    }
    else if(b.rotConstraint == IKGoal::RotAxis) {
      //fixed/axis constraint
      Sphere3D sa; sa.center = ja; sa.radius = Lmax;
      Circle3D cb; cb.center = b.endPosition; cb.axis = b.endRotation; cb.radius = b.localPosition.norm();
      return BallCircleCollision(sa,cb);
    }
    else {
      //fixed/point constraint
      Real db = b.localPosition.norm();
      Real d=(b.endPosition-ja).norm();
      return (d <= Lmax+db);
    }
  }
  else if(a.rotConstraint == IKGoal::RotAxis) {
    Circle3D ca;
    ca.center = a.endPosition; ca.axis = a.endRotation; ca.radius = a.localPosition.norm();
    if(b.rotConstraint == IKGoal::RotAxis) {
      //axis/axis constraint
      Circle3D cb; 
      cb.center = b.endPosition; cb.axis = b.endRotation; cb.radius = b.localPosition.norm();
      Vector3 pa,pb;
      CircleCircleClosestPoints(ca,cb,pa,pb);
      return ((pa-pb).norm() <= Lmax);
    }
    else {
      //axis/point constraint
      Sphere3D sb; sb.center = b.endPosition; sb.radius = Lmax+b.localPosition.norm();
      return BallCircleCollision(sb,ca);
    }
  }
  else {
    //point/point constraint
    Real da = a.localPosition.norm();
    Real db = b.localPosition.norm();
    Real d = (a.endPosition-b.endPosition).norm();
    return (d <= Lmax+da+db);
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Shouldn't get here...");
  Abort();
  return false;
}














Real RobotIKError(const RobotKinematics3D& robot,const IKGoal& goal)
{
  int m=IKGoal::NumDims(goal.posConstraint);
  int n=IKGoal::NumDims(goal.rotConstraint);
  Real poserr[3],orierr[3];
  if(goal.destLink < 0)
     goal.GetError(robot.links[goal.link].T_World,poserr,orierr);
  else {
    RigidTransform Trel;
    Trel.mulInverseB(robot.links[goal.link].T_World,robot.links[goal.destLink].T_World);
    goal.GetError(Trel,poserr,orierr);
  }

  Real emax=0.0;
  for(int i=0;i<m;i++)
    emax = Max(emax,Abs(poserr[i]));
  for(int i=0;i<n;i++)
    emax = Max(emax,Abs(orierr[i]));
  return emax;
}




void GetConstraintPoints(const IKGoal& g,vector<Vector3>& lp,vector<Vector3>& wp)
{
  Assert(g.posConstraint == IKGoal::PosFixed);
  switch(g.rotConstraint) {
  case IKGoal::RotNone:
    lp.resize(1);
    wp.resize(1);
    lp[0] = g.localPosition;
    wp[0] = g.endPosition;
    break;
  case IKGoal::RotAxis:
    lp.resize(2);
    wp.resize(2);
    lp[0] = g.localPosition;
    wp[0] = g.endPosition;
    lp[1] = g.localPosition + g.localAxis;
    wp[1] = g.endPosition + g.endRotation;
    break;
  case IKGoal::RotFixed:
    lp.resize(3);
    wp.resize(3);
    lp[0] = g.localPosition;
    wp[0] = g.endPosition;
    {
      RigidTransform T;
      g.GetFixedGoalTransform(T);
      lp[1] = g.localPosition;
      lp[1].x += 1.0;
      wp[1] = T * lp[1];
      lp[2] = g.localPosition;
      lp[2].y += 1.0;
      wp[1] = T * lp[2];
    }
    break;
  default:
        LOG4CXX_ERROR(KrisLibrary::logger(),"Two-axis rotations not supported\n");
    break;
  }
}

bool IntersectGoals(const IKGoal& a,const IKGoal& b,IKGoal& c,Real tolerance)
{
  Assert(a.link == b.link);
  Assert(a.destLink == b.destLink);
  //fixed constraints get copied over if compatible
  if(b.posConstraint == IKGoal::PosFixed && b.rotConstraint == IKGoal::RotFixed) {
    RigidTransform T;
    b.GetFixedGoalTransform(T);
    Real poserr[3],roterr[3];
    a.GetError(T,poserr,roterr);
    for(int j=0;j<IKGoal::NumDims(a.posConstraint);j++)
      if(poserr[j] > tolerance) return false;
    for(int j=0;j<IKGoal::NumDims(a.rotConstraint);j++)
      if(roterr[j] > tolerance) return false;
    c = b;
    return true;
  }
  if(a.posConstraint == IKGoal::PosFixed && a.rotConstraint == IKGoal::RotFixed) {
    return IntersectGoals(b,a,c,tolerance);
  }
  //empty goals
  if(a.posConstraint == IKGoal::PosNone && a.rotConstraint == IKGoal::RotNone) {
    c = b;
    return true;
  }
  if(b.posConstraint == IKGoal::PosNone && b.rotConstraint == IKGoal::RotNone) {
    c = a;
    return true;
  }

  c.link = a.link;
  c.destLink = a.destLink;

  if(a.posConstraint == IKGoal::PosFixed && b.posConstraint == IKGoal::PosFixed) {
    //a fixes certain points and b fixes certain other points -- merge them together
    vector<Vector3> aplocal,apworld,bplocal,bpworld;
    GetConstraintPoints(a,aplocal,apworld);
    GetConstraintPoints(b,bplocal,bpworld);
    aplocal.insert(aplocal.end(),bplocal.begin(),bplocal.end());
    apworld.insert(apworld.end(),bpworld.begin(),bpworld.end());
    c.SetFromPoints(aplocal,apworld,tolerance);
    return true;
  }

  //handle rotation constraints first
  if(a.rotConstraint == IKGoal::RotNone) {
    c.rotConstraint = b.rotConstraint;
    c.localAxis = b.localAxis;
    c.endRotation = b.endRotation;
  }
  else if(b.rotConstraint == IKGoal::RotNone) {
    c.rotConstraint = a.rotConstraint;
    c.localAxis = a.localAxis;
    c.endRotation = a.endRotation;
  }
  else if(a.rotConstraint == IKGoal::RotFixed) {
    c.rotConstraint = a.rotConstraint;
    c.endRotation = a.endRotation;
    //now determine compatibility
    if(b.rotConstraint == IKGoal::RotFixed) {
      if(!a.endRotation.isEqual(b.endRotation,tolerance)) return false;
    }
    else if(b.rotConstraint == IKGoal::RotAxis) {
      MomentRotation m(a.endRotation);
      Matrix3 R;
      m.getMatrix(R);
      if(!b.endRotation.isEqual(R*b.localAxis)) return false;
    }
    else {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Two-axis rotations not supported\n");
      return false;
    }
  }
  else if(b.rotConstraint == IKGoal::RotFixed) {
    c.rotConstraint = b.rotConstraint;
    c.endRotation = b.endRotation;
    //now determine compatibility
    if(a.rotConstraint == IKGoal::RotFixed) {
      if(!a.endRotation.isEqual(b.endRotation,tolerance)) return false;
    }
    else if(a.rotConstraint == IKGoal::RotAxis) {
      MomentRotation m(b.endRotation);
      Matrix3 R;
      m.getMatrix(R);
      if(!a.endRotation.isEqual(R*a.localAxis,tolerance)) return false;
    }
    else {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Two-axis rotations not supported\n");
      return false;
    }
  }
  else if(a.rotConstraint == IKGoal::RotTwoAxis || b.rotConstraint == IKGoal::RotTwoAxis) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Two-axis rotations not supported\n");
    return false;
  }
  else {
    Assert(a.rotConstraint == IKGoal::RotAxis && b.rotConstraint == IKGoal::RotAxis);
    //if compatible, set a single axis constraint
    //otherwise, set the proper rotation matrix
    if((a.localAxis.isEqual(b.localAxis,tolerance) && a.endRotation.isEqual(b.endRotation,tolerance)) || 
       (a.localAxis.isEqual(-b.localAxis,tolerance) && a.endRotation.isEqual(-b.endRotation,tolerance))) {
      c.rotConstraint = IKGoal::RotAxis;
      c.localAxis = a.localAxis;
      c.endRotation = a.endRotation;
    }
    else {
      //TODO find a rotation matrix that maps both axes to their directions
            LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: intersect two axis rotations\n");
      return false;
    }
  }

  //c's rotation is now chosen.
  //now deal with the positions, at least one of which must be non-fixed
  if(a.posConstraint == IKGoal::PosNone) {
    //copy b's position constraint 
    c.posConstraint = b.posConstraint;
    c.localPosition = b.localPosition;
    c.endPosition = b.endPosition;
    c.direction = b.direction;
    return true;
  }
  else if(b.posConstraint == IKGoal::PosNone) {
    //copy a's position constraint 
    c.posConstraint = a.posConstraint;
    c.localPosition = a.localPosition;
    c.endPosition = a.endPosition;
    c.direction = a.direction;
    return true;
  }
  if(c.rotConstraint == IKGoal::RotFixed) {
    MomentRotation m(c.endRotation);
    Matrix3 R;
    m.getMatrix(R);
    //check compatibility of fixed directions
    if(a.posConstraint == IKGoal::PosFixed) {
      c.posConstraint = a.posConstraint;
      c.localPosition = a.localPosition;
      c.endPosition = a.endPosition;
      RigidTransform T;
      c.GetFixedGoalTransform(T);
      if(b.posConstraint == IKGoal::PosLinear) {
	if(!cross(b.direction,T*b.localPosition-b.endPosition).isZero(tolerance)) return false;
      }
      else if(b.posConstraint == IKGoal::PosPlanar) {
	if(!FuzzyZero(b.direction.dot(T*b.localPosition-b.endPosition),tolerance)) return false;
      }
      return true;
    }
    else if(b.posConstraint == IKGoal::PosFixed) {
      c.posConstraint = b.posConstraint;
      c.localPosition = b.localPosition;
      c.endPosition = b.endPosition;
      RigidTransform T;
      c.GetFixedGoalTransform(T);
      if(a.posConstraint == IKGoal::PosLinear) {
	if(!cross(a.direction,T*a.localPosition-a.endPosition).isZero(tolerance)) return false;
      }
      else if(a.posConstraint == IKGoal::PosPlanar) {
	if(!FuzzyZero(a.direction.dot(T*a.localPosition-a.endPosition),tolerance)) return false;
      }
      return true;
    }
    else {
      //fixed rotation, linear or planar positions
            LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: merging linear or planar position constraints\n");
      return false;
    }
  }
  //check equality
  if(a.posConstraint == b.posConstraint) {
    if(a.localPosition.isEqual(b.localPosition,tolerance) &&
       a.endPosition.isEqual(b.endPosition,tolerance) &&
       a.direction.isEqual(b.direction,tolerance)) {
      c.posConstraint = a.posConstraint;
      c.localPosition = a.localPosition;
      c.endPosition = a.endPosition;
      return true;
    }
  }
  //non-fixed rotation, linear or planar positions
    LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: merging linear or planar position constraints with non-fixed rotations\n");
  return false;
}

bool AddGoalNonredundant(const IKGoal& goal,vector<IKGoal>& goalSet,Real tolerance)
{
  for(size_t i=0;i<goalSet.size();i++) {
    if(goal.link == goalSet[i].link) {
      if(goal.destLink == goalSet[i].destLink) {
	//non-duplicate -- examine more closely
	IKGoal newgoal;
	if(!IntersectGoals(goal,goalSet[i],newgoal,tolerance)) return false;
	return true;
      }
    }
  }
  goalSet.push_back(goal);
  return true;
}

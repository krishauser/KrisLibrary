#include <KrisLibrary/Logger.h>
#include "IK.h"
#include "Rotation.h"
#include "Geometry.h"
#include <math3d/basis.h>
#include <math3d/rotationfit.h>
#include <math3d/Plane3D.h>
#include <math3d/Line3D.h>
#include <math3d/misc.h>
#include <iostream>
using namespace std;

IKGoal::IKGoal()
  : link(0),destLink(-1),posConstraint(PosNone),localPosition(Zero),rotConstraint(RotNone)
{
}

void IKGoal::SetFixedTransform(const RigidTransform& T)
{
  localPosition.setZero(); 
  SetFixedPosition(T.t);
  SetFixedRotation(T.R); 
}

void IKGoal::SetPlanarPosition(const Vector3& point,const Vector3& normal)
{
  posConstraint = PosPlanar;
  endPosition = point;
  direction = normal;
}

void IKGoal::SetLinearPosition(const Vector3& point,const Vector3& d)
{
  posConstraint = PosLinear;
  endPosition = point;
  direction = d;
}


void IKGoal::SetFixedRotation(const Matrix3& R)
{
  rotConstraint = RotFixed;
  MomentRotation mR;
  if(!mR.setMatrix(R)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal::SetFixedRotation: matrix does not appear to be a rotation?\n");
    endRotation.setZero();
    rotConstraint = RotNone;
  }
  else
    endRotation = mR;
}

void IKGoal::SetAxisRotation(const Vector3& locAxis,const Vector3& worldAxis) 
{
  rotConstraint = RotAxis;
  localAxis = locAxis;
  Assert(FuzzyEquals(localAxis.normSquared(),One));
  endRotation = worldAxis;
}

void IKGoal::SetFromPoints(const vector<Vector3>& loc,const vector<Vector3>& wor,Real tol)
{
  Assert(loc.size()==wor.size());
  if(loc.size()==0) {
    SetFreePosition();
    SetFreeRotation();
  }
  else if(loc.size()==1) {
    localPosition=loc[0];
    SetFixedPosition(wor[0]);
    SetFreeRotation();
  }
  else if(loc.size()==2) {
    localPosition = 0.5*(loc[0]+loc[1]);
    SetFixedPosition(0.5*(wor[0]+wor[1]));
    if(loc[0].isEqual(loc[1],tol) || wor[0].isEqual(wor[1],tol)) {
      SetFreeRotation();
    }
    else {
      Vector3 laxis,waxis;
      laxis.sub(loc[1],loc[0]);
      waxis.sub(wor[1],wor[0]);
      laxis.inplaceNormalize();
      waxis.inplaceNormalize();
      SetAxisRotation(laxis,waxis);
    }
  }
  else {
    RigidTransform Tloc,Twor;
    Vector3 cov;
    Real res=FitFrames(loc,wor,Tloc,Twor,cov);
    if(IsInf(res)) {
      SetFreePosition();
      SetFreeRotation();
    }
    else {
		//TLoc * lp = Twor * wp (verified)
		//We want R, loc, and wor such that R*(lp-loc) = wp - wor for all pairs (lp,wp)
		//Rloc*lp + tloc = Rwor*wp + twor
		//Rwor^T*Rloc*lp + Rwor^T * tloc = wp + RWor^T*twor
		//Rwor^T*Rloc*(lp - (-Rloc^T * tloc)) = wp + RWor^T*twor (confirmed)
      localPosition = -(transpose(Tloc.R)*Tloc.t);
      Matrix3 R = transpose(Twor.R)*Tloc.R;
      SetFixedPosition(-(transpose(Twor.R)*Twor.t));
      SetFixedRotation(R);
	  /*
	  LOG4CXX_ERROR(KrisLibrary::logger(), "Fitting error " << res << "\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"Local point rot to common coords: "<<Tloc.R);
	  LOG4CXX_INFO(KrisLibrary::logger(), "Local point translation to common coords: " << Tloc.t << "\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"World point rot to common coords: "<<Twor.R);
	  LOG4CXX_INFO(KrisLibrary::logger(), "World point translation to common coords: " << Twor.t << "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "Points in common coordinates:" << "\n");
	  for (size_t i = 0; i<loc.size(); i++) 
		  LOG4CXX_ERROR(KrisLibrary::logger(), Tloc*loc[i] << " vs " << Twor*wor[i] << " error "<< (Tloc*loc[i]).distance(Twor*wor[i]) << "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "Rotation: " << R << "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "local position: " << localPosition << "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "world position: " << endPosition << "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "\n");
	  LOG4CXX_INFO(KrisLibrary::logger(), "Mapping: " << "\n");
      for(size_t i=0;i<loc.size();i++) {
	LOG4CXX_INFO(KrisLibrary::logger(),loc[i]<<" -> "<< R*(loc[i]-localPosition) + endPosition <<" = "<<wor[i]);
      }
	  */
      if(Abs(cov.z) < tol) {
	if(Abs(cov.y) < tol) {
	  if(Abs(cov.x) < tol) {
	    //just a point
	    SetFreeRotation();
	  }
	  else { 
	    //just an axis, look at x column for axis
	    Vector3 wx,lx;
	    Tloc.R.getRow1(lx);
	    Twor.R.getRow1(wx);
	    SetAxisRotation(lx,wx);
	  }
	}
      }
    }
  }
}

void IKGoal::GetFixedGoalRotation(Matrix3& R) const 
{
  Assert(rotConstraint == RotFixed);
  MomentRotation m(endRotation);
  m.getMatrix(R);
}


void IKGoal::GetFixedGoalTransform(RigidTransform& T) const 
{
  Assert(posConstraint == PosFixed);
  Assert(rotConstraint == RotFixed);
  MomentRotation m(endRotation);
  GetRotationAboutLocalPoint(localPosition,endPosition,m,T);
}

void IKGoal::GetBaseEdgeRotation(Matrix3& R0) const
{
  GetMinimalRotation(localAxis,endRotation,R0);
}

void IKGoal::GetEdgeGoalTransform(Real theta,RigidTransform& T) const
{
  Matrix3 R0,RTheta;
  GetBaseEdgeRotation(R0);
  AngleAxisRotation aa;
  aa.axis = localAxis; 
  aa.angle = theta;
  aa.getMatrix(RTheta);
  T.R.mul(R0,RTheta);
  T.t = endPosition - T.R*localPosition;
}

void IKGoal::GetClosestGoalTransform(const RigidTransform& T0,RigidTransform& T) const
{
  //fill out rotation first
  if(rotConstraint == RotFixed) {
    GetFixedGoalRotation(T.R);
  }
  else if(rotConstraint == RotAxis) {
    //T.R*localAxis = endRotation
    GetMinimalRotation(localAxis,T0.R*endRotation,T.R);
    //make it so orthogonal directions perform a rotation similar to T0.R
    Vector3 lx,ly,rx,ry,refx;
    GetCanonicalBasis(localAxis,lx,ly);
    rx = T.R*rx;
    ry = T.R*ry;
    refx = T0.R*lx;
    Real x = dot(refx,rx);
    Real y = dot(refx,ry);
    //find the rotation about endRotation that gets closer to this
    Real theta = Atan2(y,x);
    AngleAxisRotation aa;
    aa.angle = theta;
    aa.axis = T0.R*endRotation;
    
    Matrix3 Rrot;
    aa.getMatrix(Rrot);
    T.R = Rrot*T.R;
  }
  else 
    T.R = T0.R;

  T.t = endPosition - T.R*localPosition;
  if(posConstraint == PosPlanar) {
    //find closest transform on plane to T0.t
    Plane3D p;
    p.setPointNormal(T.t,direction);
    p.project(T0.t,T.t);
  }
  else if(posConstraint == PosLinear) {
    //find closest transform on line to T0.t
    Line3D line;
    line.source = T.t;
    line.direction = direction;
    line.closestPoint(T0.t,T.t);
  }
  else if(posConstraint == PosNone)
    T.t = T0.t;
}

void IKGoal::MatchGoalTransform(const RigidTransform& Trel)
{
  Trel.mul(localPosition,endPosition);
  if(rotConstraint == RotFixed) {
    MomentRotation m;
    m.setMatrix(Trel.R);
    endRotation = m;
  }
  else if(rotConstraint == RotAxis) {
    Trel.R.mul(localAxis,endRotation);
  }
}





void IKGoal::Transform(const RigidTransform& T)
{
  endPosition = T*endPosition;
  direction = T.R*direction;
  if(rotConstraint == IKGoal::RotFixed) {
    MomentRotation m; m.set(endRotation);
    Matrix3 R;
    m.getMatrix(R);
    R = T.R*R;
    Assert(IsFinite(R));
    if(!m.setMatrix(R)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal::Transform: matrix does not appear to be a rotation?\n");
      endRotation.setZero();
    }
    else
      endRotation = m;
  }
  else if(rotConstraint == IKGoal::RotAxis) {
    endRotation = T.R*endRotation;
  }
}

void IKGoal::TransformLocal(const RigidTransform& T)
{
  localPosition = T*localPosition;
  if(rotConstraint == IKGoal::RotFixed) {
    MomentRotation m; m.set(endRotation);
    Matrix3 R;
    m.getMatrix(R);
    R.mulTransposeA(T.R,R);
    Assert(IsFinite(R));
    if(!m.setMatrix(R)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal::TransformLocal: matrix does not appear to be a rotation?\n");
      endRotation.setZero();
    }
    else
      endRotation = m;
  }
  else if(rotConstraint == IKGoal::RotAxis) {
    localAxis = T.R*localAxis;
  }
}

void IKGoal::RemoveRotationAxis(const Vector3& p,const Vector3& axis)
{
  switch(rotConstraint) {
  case 0: //done
    break;
  case 1: //single-axis rotation
    FatalError("TODO - set dual-axis rotation");
    break;
  case 2: //dual-axis rotation
    //if one of the axes are parallel to the normal, all is well
    FatalError("TODO - dual-axis rotation isn't implemented");
    break;
  case 3: //fixed rotation
    //allow it to rotate about the cp normal
    {
      Vector3 locAxis;
      Vector3 locPos;
      RigidTransform T;
      GetFixedGoalTransform(T);
      T.R.mulTranspose(axis,locAxis);
      T.mulInverse(p,locPos);
      SetAxisRotation(locAxis,axis);
      localPosition = locPos;
      endPosition = p;
    }
    break;
  }
}

void IKGoal::RemoveRotationAxis(const Vector3& axis)
{
  switch(rotConstraint) {
  case IKGoal::PosNone: //done
    break;
  case IKGoal::RotTwoAxis: //dual-axis rotation
    //if one of the axes are parallel to the normal, all is well
    FatalError("TODO - dual-axis rotation isn't implemented");
    break;
  case IKGoal::RotAxis: //single-axis rotation
    FatalError("TODO - set dual-axis rotation");
    break;
  case IKGoal::RotFixed: //fixed rotation
    //allow it to rotate about the cp normal
    {
      Vector3 locAxis;
      RigidTransform T;
      GetFixedGoalTransform(T);
      T.R.mulTranspose(axis,locAxis);
      SetAxisRotation(locAxis,axis);
    }
    break;
  }
}

void IKGoal::RemovePositionAxis(const Vector3& dir)
{
  switch(posConstraint) {
  case IKGoal::PosNone: //done
    break;
  case IKGoal::PosPlanar: //fixed plane
    if(!FuzzyZero(direction.dot(dir)))
      posConstraint = IKGoal::PosNone;
    break;
  case IKGoal::PosLinear: //fixed line
    if(!FuzzyZero(direction.dot(dir))) {
      Vector3 v;
      v.setCross(direction,dir);
      direction.setNormalized(v);
      posConstraint = IKGoal::PosPlanar;
    }
    break;
  case IKGoal::PosFixed: //fixed position
    direction.setNormalized(dir);
    posConstraint = IKGoal::PosLinear;
    break;
  }
}

void IKGoal::GetError(const RigidTransform& Trel,Real poserr[3],Real orierr[3]) const
{
  Vector3 perr=Trel*localPosition-endPosition;
  if(posConstraint == IKGoal::PosFixed) {
    perr.get(poserr[0],poserr[1],poserr[2]);
  }
  else if(posConstraint == IKGoal::PosLinear) {
    Vector3 xb,yb;
    direction.getOrthogonalBasis(xb,yb);
    poserr[0] = dot(perr,xb);
    poserr[1] = dot(perr,yb);
  }
  else if(posConstraint == IKGoal::PosPlanar) {
    poserr[0] = dot(perr,direction);
  }

  if(rotConstraint==IKGoal::RotFixed) {
    MomentRotation em(endRotation);
    Matrix3 Rgoal,Rdiff;
    em.getMatrix(Rgoal);
    Rdiff.mulTransposeB(Trel.R,Rgoal);
    Assert(IsFinite(Rdiff));
    if(em.setMatrix(Rdiff)) {
      orierr[0]=em.x;
      orierr[1]=em.y;
      orierr[2]=em.z;
    }
    else {
            LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal::GetError: matrix does not appear to be a rotation?\n");
      orierr[0]=orierr[1]=orierr[2] = Inf;
    }
  }
  else if(rotConstraint==IKGoal::RotAxis) {
    Vector3 x,y;
    endRotation.getOrthogonalBasis(x,y);
    Vector3 curAxis;
    Trel.R.mul(localAxis,curAxis);
    Real neg = 1.0-curAxis.dot(endRotation);
    orierr[0] = Abs(dot(curAxis,x))+neg;
    orierr[1] = Abs(dot(curAxis,y))+neg;
  }
  else if(rotConstraint==IKGoal::RotNone) { }
  else {
    FatalError("EvalIKGoal(): Invalid number of rotation terms");
  }
}

istream& operator >> (istream& in,IKGoal& data)
{
  in >> data.link >> data.destLink;
  char ptype,rtype;
  in>>ptype;
  if(ptype == 'N') { //no position
    data.posConstraint = IKGoal::PosNone;
  }
  else if(ptype == 'P') {  //planar
    data.posConstraint = IKGoal::PosPlanar;
    in >> data.localPosition >> data.endPosition >> data.direction;
  }
  else if(ptype == 'L') { //linear
    data.posConstraint = IKGoal::PosLinear;
    in >> data.localPosition >> data.endPosition >> data.direction;
  }
  else if(ptype == 'F' ) { //fixed
    data.posConstraint = IKGoal::PosFixed;
    in >> data.localPosition >> data.endPosition;
  }
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal: invalid position type character "<<ptype);
    in.setstate(ios::badbit);
    return in;
  }
  in>>rtype;
  if(rtype == 'N') { //no rotation
    data.rotConstraint = IKGoal::RotNone;
  }
  else if(rtype == 'T') {  //twoaxis
    data.rotConstraint = IKGoal::RotTwoAxis;
    in >> data.localAxis >> data.endRotation;
  }
  else if(rtype == 'A') { //axis
    data.rotConstraint = IKGoal::RotAxis;
    in >> data.localAxis >> data.endRotation;
  }
  else if(rtype == 'F' ) { //fixed
    data.rotConstraint = IKGoal::RotFixed;
    in >> data.endRotation;
  }
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"IKGoal: invalid rotation type character "<<rtype);
    in.setstate(ios::badbit);
    return in;
  }
  return in;
}

std::ostream& operator << (std::ostream& out,const IKGoal& data)
{
  out<<data.link<<" "<<data.destLink<<endl;
  switch(data.posConstraint) {
  case IKGoal::PosNone:
    out<<"N"<<endl;
    break;
  case IKGoal::PosPlanar:
    out<<"P "<<data.localPosition<<"   "<<data.endPosition<<"   "<<data.direction<<endl;
    break;
  case IKGoal::PosLinear:
    out<<"L "<<data.localPosition<<"   "<<data.endPosition<<"   "<<data.direction<<endl;
    break;
  case IKGoal::PosFixed:
    out<<"F "<<data.localPosition<<"   "<<data.endPosition<<endl;
    break;
  }
  switch(data.rotConstraint) {
  case IKGoal::RotNone:
    out<<"N"<<endl;
    break;
  case IKGoal::RotTwoAxis:
    out<<"T "<<data.localAxis<<"   "<<data.endRotation<<endl;
    break;
  case IKGoal::RotAxis:
    out<<"A "<<data.localAxis<<"   "<<data.endRotation<<endl;
    break;
  case IKGoal::RotFixed:
    out<<"F "<<data.endRotation<<endl;
    break;
  }
  return out;
}

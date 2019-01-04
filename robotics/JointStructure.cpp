#include <KrisLibrary/Logger.h>
#include "JointStructure.h"
#include "WorkspaceBound.h"
#include <math3d/Circle3D.h>
#include <math3d/Sphere3D.h>
#include <iostream>
#include <errors.h>

/*
#include <math3d/rotation.h>

JointWorkspaceBound::JointWorkspaceBound()
{
  center.setZero();
  radius=Inf;
  maxAngle=TwoPi;
}

void JointWorkspaceBound::SetPoint(const Vector3& p)
{
  center = p;
  radius = Zero;
  maxAngle = Zero;
}

bool JointWorkspaceBound::IsEmpty() const
{
  return radius < 0;
}

//intersect this bound with the given one, returns true if this changes
bool JointWorkspaceBound::SetIntersection(const JointWorkspaceBound& w)
{
  Circle3D c;
  int res = SphereSphereIntersection(*this,w,c);
  Real solidAngle = Min(maxAngle*radius,w.maxAngle*w.radius);
  if(res==2) {
    Assert(c.radius <= radius+Epsilon && c.radius <= w.radius+Epsilon);
    Assert(contains(c.center));
    Assert(w.contains(c.center));

    //If a and b lie on opposite sides of c's plane,
    //use the sphere that contains c.
    //Otherwise, the smallest circle that contains the intersection
    //is the smaller of a and b
    Real offset=c.axis.dot(c.center);
    if(Sign(c.axis.dot(center)-offset) == Sign(c.axis.dot(w.center)-offset)) {
      //on same side of plane
      if(w.radius < radius) res=4;
      else res=3;
    }
  }
  switch(res) {
  case 0: 
    center = Half*(center + w.center);
    radius=-(center-w.center).norm();
    maxAngle=0;
    break;
  case 1:
  case 2:
    center=c.center;
    radius=c.radius;
    //get the maxAngle that gives the right solid angle
    if(solidAngle >= radius*TwoPi) maxAngle = TwoPi;
    else maxAngle = solidAngle/radius;
    break;
  case 3:  //completely contained within w
    Assert(radius <= w.radius);
    if(solidAngle < maxAngle*radius) {
      maxAngle = solidAngle / radius;
      return true;
    }
    return false;
  case 4:
    //w contained completely in this
    Assert(radius >= w.radius);
    center = w.center;
    radius = w.radius;
    if(solidAngle >= radius*TwoPi) maxAngle = TwoPi;
    else maxAngle = solidAngle/radius;
    break;
  }
  return true;
}

Real JointWorkspaceBound::MaxDistance() const
{
  if(maxAngle >= Pi) return Two*radius;
  else return Sqrt(Two-Two*Cos(maxAngle))*radius;
}

void JointWorkspaceBound::Expand(const JointWorkspaceBound& b, Real rDelta, Real qDelta)
{
  center = b.center;
  radius = b.radius + rDelta;
  maxAngle = b.maxAngle + qDelta;
}

*/



JointStructure::JointStructure(const RobotKinematics3D& _robot)
  :robot(_robot)
{
}

void JointStructure::Init()
{
  robot.GetChildList(children);
  linkPoints.resize(robot.q.n);
}

bool JointStructure::IsFeasible() const
{
  for(size_t k=0;k<bounds.size();k++)
    if(bounds[k].IsEmpty()) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Bound for link "<<k<<" collapsed to nothing");
      return false;
    }

  for(size_t k=0;k<pointBounds.size();k++) 
    for(size_t m=0;m<pointBounds[k].size();m++) 
      if(pointBounds[k][m].IsEmpty()) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Bound for point "<<m<<" on link "<<k<<" collapsed to nothing");
	return false;
      }
  return true;
}

void JointStructure::GetLinkWorkspaceBounds(vector<Real>& dbounds) const
{
  int n=robot.q.n;
  dbounds.resize(n);
  fill(dbounds.begin(),dbounds.end(),Zero);
  for(int i=0;i<n;i++) {
    //take the max dist of all joints attached to this link

    int p=robot.parents[i];
    if(p >= 0) {
      dbounds[i] = Max(dbounds[i],bounds[i].GetWidth());
    }
    for(size_t k=0;k<children[i].size();k++) {
      int c = children[i][k];
      dbounds[i] = Max(dbounds[i],bounds[c].GetWidth());
    }
    for(size_t k=0;k<linkPoints[i].size();k++) {
      dbounds[i] = Max(dbounds[i],pointBounds[i][k].GetWidth());
    }
  }
}

void JointStructure::SetInitialBounds(WorkspaceBound& b,const IKGoal& c,const Vector3& plocal)
{
  Assert(c.posConstraint==IKGoal::PosFixed);
  switch(c.rotConstraint) {
  case IKGoal::RotFixed:
    {
      Vector3 p;
      RigidTransform T;
      c.GetFixedGoalTransform(T);
      T.mulPoint(plocal,p);
      b.SetPoint(p);
    }
    break;
  case IKGoal::RotTwoAxis:
    LOG4CXX_INFO(KrisLibrary::logger(),"Can't have 2 rotation constraints");
    Abort();
  case IKGoal::RotAxis:
    {
      Circle3D circle;
      circle.center = c.endPosition;
      circle.radius = plocal.distance(c.localPosition);
      b.SetCircle(circle);
    }
    break;
  case IKGoal::RotNone:
    {
      Sphere3D s;
      s.center=c.endPosition;
      s.radius = plocal.distance(c.localPosition);
      b.SetSphere(s);
    }
    break;
  }
}

void JointStructure::SolveWorkspaceBounds(const IKGoal& constraint)
{
  int n=robot.q.n;
  int link0 = constraint.link;
  bounds.resize(n);
  pointBounds.resize(linkPoints.size());
  for(size_t i=0;i<linkPoints.size();i++)
    pointBounds[i].resize(linkPoints[i].size());
  reached.resize(n);
  fill(reached.begin(),reached.end(),false);

  //fixed the bounds from link0, start iteration from there
  SetInitialBounds(bounds[link0],constraint,Vector3(Zero));
  reached[link0] = true;
  SolveBoundsIter(robot.parents[link0],link0);
  for(size_t k=0;k<children[link0].size();k++) {
    int child = children[link0][k];
    SetInitialBounds(bounds[child],constraint,robot.links[child].T0_Parent.t);
    reached[child] = true;
    SolveBoundsIter(child,child);
  }

  for(size_t k=0;k<pointBounds[link0].size();k++)
    SetInitialBounds(pointBounds[link0][k],constraint,linkPoints[link0][k]);

  if(constraint.rotConstraint == IKGoal::RotAxis || constraint.rotConstraint == IKGoal::RotNone) {
    if(constraint.localPosition.normSquared() != 0) {
      //add the bound for the rotation point
      linkPoints[link0].push_back(constraint.localPosition);
      WorkspaceBound bpt;
      bpt.SetPoint(constraint.endPosition);
      pointBounds[link0].push_back(bpt);
    }
  }
}


void JointStructure::IntersectWorkspaceBounds(const IKGoal& goal)
{
  vector<WorkspaceBound> tempBounds;
  vector<vector<WorkspaceBound> > tempPointBounds;
  swap(bounds,tempBounds);
  swap(pointBounds,tempPointBounds);

  SolveWorkspaceBounds(goal);
  for(size_t k=0;k<bounds.size();k++) {
    bounds[k].InplaceIntersection(tempBounds[k]);
  }
  for(size_t k=0;k<pointBounds.size();k++) {
    size_t n=tempPointBounds[k].size();
    //could have added a new bound for the goal position
    Assert(n <= pointBounds[k].size() && n+1 >= pointBounds[k].size());
    for(size_t m=0;m<n;m++) {
      pointBounds[k][m].InplaceIntersection(tempPointBounds[k][m]);
    }
  }
}

void JointStructure::SolveWorkspaceBounds(const vector<IKGoal>& constraints)
{
  if(constraints.empty()) {
    bounds.resize(robot.q.n);
    pointBounds.resize(linkPoints.size());
    for(size_t i=0;i<bounds.size();i++)
      bounds[i].SetFull();
    for(size_t i=0;i<pointBounds.size();i++) {
      pointBounds[i].resize(linkPoints[i].size());
      for(size_t j=0;j<pointBounds[i].size();j++)
	pointBounds[i][j].SetFull();
    }
    return;
  }

  vector<WorkspaceBound> tempBounds;
  vector<vector<WorkspaceBound> > tempPointBounds;

  Assert(constraints.size() >= 1);
  for(size_t i=0;i<constraints.size();i++) {
    SolveWorkspaceBounds(constraints[i]);
    if(i==0) {
      swap(tempBounds,bounds);
      swap(tempPointBounds,pointBounds);
    }
    else {
      for(size_t k=0;k<tempBounds.size();k++)
	tempBounds[k].InplaceIntersection(bounds[k]);
      for(size_t k=0;k<pointBounds.size();k++) {
	size_t n=tempPointBounds[k].size();
	//could have added a new bound for the goal position
	Assert(n <= pointBounds[k].size() && n+1 >= pointBounds[k].size());
	for(size_t m=0;m<n;m++) {
	  tempPointBounds[k][m].InplaceIntersection(pointBounds[k][m]);
	}
	if(n != pointBounds[k].size()) {
	  tempPointBounds[k].push_back(pointBounds[k].back());
	  Assert(pointBounds[k].size() == tempPointBounds[k].size());
	}
      }
    }
  }
  swap(tempBounds,bounds);
  swap(tempPointBounds,pointBounds);
}

void JointStructure::SolveWorkspaceBounds(int link0)
{
  //this hack should take care of things
  IKGoal tempGoal;
  tempGoal.link = link0;
  tempGoal.posConstraint = IKGoal::PosFixed;
  tempGoal.rotConstraint = IKGoal::RotFixed;
  tempGoal.localPosition.setZero();
  tempGoal.endPosition = robot.links[tempGoal.link].T_World.t;
  MomentRotation r;
  r.setMatrix(robot.links[tempGoal.link].T_World.R);
  tempGoal.endRotation = r;
  SolveWorkspaceBounds(tempGoal);
}

void JointStructure::IntersectWorkspaceBounds(int link0)
{
  //this hack should take care of things
  IKGoal tempGoal;
  tempGoal.link = link0;
  tempGoal.posConstraint = IKGoal::PosFixed;
  tempGoal.rotConstraint = IKGoal::RotFixed;
  tempGoal.localPosition.setZero();
  tempGoal.endPosition = robot.links[tempGoal.link].T_World.t;
  MomentRotation r;
  r.setMatrix(robot.links[tempGoal.link].T_World.R);
  tempGoal.endRotation = r;
  IntersectWorkspaceBounds(tempGoal);
}

void JointStructure::SolveRootBounds()
{
  //this hack should take care of things
  IKGoal tempGoal;
  tempGoal.link = 0;
  tempGoal.posConstraint = IKGoal::PosFixed;
  tempGoal.rotConstraint = IKGoal::RotNone;
  tempGoal.localPosition.setZero();
  tempGoal.endPosition = robot.links[0].T_World.t;
  SolveWorkspaceBounds(tempGoal);
}

void JointStructure::IntersectRootBounds()
{
  //this hack should take care of things
  IKGoal tempGoal;
  tempGoal.link = 0;
  tempGoal.posConstraint = IKGoal::PosFixed;
  tempGoal.rotConstraint = IKGoal::RotNone;
  tempGoal.localPosition.setZero();
  tempGoal.endPosition = robot.links[0].T_World.t;
  IntersectWorkspaceBounds(tempGoal);
}

//return t[link1]-t[link2]
Vector3 GetJointTranslation(const RobotKinematics3D& robot,int link1,int link2)
{
  //either they have the same parent, or one is a parent of the other
  int p1 = robot.parents[link1];
  int p2 = robot.parents[link2];
  if(p1 == p2) {  //on the same parent link
    return robot.links[link1].T0_Parent.t - robot.links[link2].T0_Parent.t;
  }
  else if(p1 == link2) {
    return robot.links[link1].T0_Parent.t;
  }
  else if(p2 == link1) {
    return -robot.links[link2].T0_Parent.t;
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"GetJointTranslation(): Error! the joints are not on adjacent bodies");
    Abort();
  }
  return Zero;
}

//get the initial bound of k relative to p
WorkspaceBound InitialBound(const RobotKinematics3D& robot,int k,int p)
{
  WorkspaceBound b;
  if(robot.links[p].type == RobotLink3D::Revolute) {
    Arc3D arc;
    arc.axis.source.setZero();
    arc.axis.direction = robot.links[p].w;
    arc.p = GetJointTranslation(robot,k,p);
    if(robot.parents[k] != p)  //flip the range!
      arc.interval.setRange(-robot.qMax[p],-robot.qMin[p]);
    else
      arc.interval.setRange(robot.qMin[p],robot.qMax[p]);
    b.SetArc(arc);
  }
  else {
    if(IsInf(robot.qMin(p)) || IsInf(robot.qMax(p))) {
      b.SetFull();
    }
    else {
      Sphere3D s;
      const Vector3& axis=robot.links[p].w;
      s.radius=(robot.qMax(p)-robot.qMin(p))*Half;
      s.center.mul(axis,(robot.qMax(p)+robot.qMin(p))*Half);
      b.SetSphere(s);
    }
  }
  return b;
}

void JointStructure::Expand(int n,int p)
{
  WorkspaceBound b=InitialBound(robot,n,p);
  bounds[n].SetMinkowskiSum(bounds[p],b);
  //LOG4CXX_INFO(KrisLibrary::logger(),"From "<<p<<" to "<<n<<" expanded from "<<bounds[p].outerRadius<<" to "<<bounds[n].outerRadius);
}

void JointStructure::SolveBoundsIter(int n,int p)
{
  if(n < 0) return;  //disregard root link
  //calculate bounds for all joints attached to n
  //bounds for p are calculated  
  Assert(reached[p]);

  //for all untraversed edges, calculate the joint bounds
  if(n != p) {  //joint of n not calculated yet
    Assert(!reached[n]);
    //traverse joint n<-up
    Expand(n,p);
    reached[n] = true;
    int up = robot.parents[n];
    SolveBoundsIter(up,n);
  }
  else {
    //HACK: for prismatic joints, expand
    if(robot.links[n].type == RobotLink3D::Prismatic) {
      Expand(n,n);
    }
  }

  for(size_t k=0;k<children[n].size();k++) {
    int c = children[n][k];
    if(c != p) {
      Assert(!reached[c]);
      //traverse joint n->c
      Expand(c,p);
      reached[c] = true;
      SolveBoundsIter(c,c);
    }
  }

  Real range=robot.qMax(p)-robot.qMin(p);
  WorkspaceBound b;
  for(size_t k=0;k<linkPoints[n].size();k++) {
    //Get the true local dist -- either p is the joint of 
    //link n, or it's a child
    if(robot.links[n].type == RobotLink3D::Prismatic) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Can't yet do link points on prismatic joints!");
      Abort();
    }
    Real ptDist;
    if(p == n) {
      ptDist = linkPoints[n][k].norm();
    }
    else {
      Assert(robot.parents[p] == n);
      ptDist = (linkPoints[n][k]-robot.links[p].T0_Parent.t).norm();
    }
    Sphere3D s;
    s.center.setZero();
    s.radius = ptDist;
    b.SetSphere(s);
    b.maxAngle = range;
    pointBounds[n][k].SetMinkowskiSum(bounds[p],b);
  }
}


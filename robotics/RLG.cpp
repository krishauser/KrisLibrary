#include <KrisLibrary/Logger.h>
#include "RLG.h"
#include "WorkspaceBound.h"
#include <math/angle.h>
#include <math/AngleSet.h>
#include <math/random.h>
#include <math3d/random.h>
#include <math3d/Segment2D.h>
#include <geometry/MonotoneChain.h>
#include <iostream>
#include <errors.h>
using Geometry::XMonotoneChain;



//given angle interval [a,b] (CCW) clamps x to [a,b] using the closest angle
Real AngleClamp(Real x,Real a,Real b)
{
  Real da=AngleCCWDiff(a,x);
  Real db=AngleCCWDiff(x,b);
  if(da > AngleCCWDiff(a,b)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Hmmm... it seems like q is between [a,b]");
    LOG4CXX_INFO(KrisLibrary::logger(),"ccw difference a-b "<<AngleCCWDiff(a,b));
    LOG4CXX_INFO(KrisLibrary::logger(),"ccw difference a-x "<<AngleCCWDiff(a,x));
    LOG4CXX_INFO(KrisLibrary::logger(),""<<x<<" in ["<<a<<","<<b);
  }
  Assert(da <= AngleCCWDiff(a,b));
  if(da > db) return b;
  else return a;
}

inline Real FixAngle(Real q,Real qmin,Real qmax)
{
  q=AngleNormalize(q);
  if(q > qmax+0.001) q -= TwoPi;
  if(q < qmin-0.001) q += TwoPi;
  if(!(qmin <= q && q <= qmax)) {
    //which one's closer?
    q = AngleClamp(AngleNormalize(q),AngleNormalize(qmin),AngleNormalize(qmax));
  }
  return q;
}

inline void FixAngleWarn(Real& q,Real qmin,Real qmax)
{
  q=AngleNormalize(q);
  if(q > qmax+0.001) q -= TwoPi;
  if(q < qmin-0.001) q += TwoPi;
  if(!(qmin <= q && q <= qmax)) {
    if(FuzzyEquals(q,qmin)) q=qmin;
    else if(FuzzyEquals(q,qmax)) q=qmax;
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"RLG: violating "<<qmin<<" <= "<<q<<" <= "<<qmax);
      q = Clamp(q,qmin,qmax);
      KrisLibrary::loggerWait();
    }
    //Assert(qmin <= q && q <= qmax);
  }
}

//returns either the set of angles that allow a to intersect with b,
//or if no intersection, returns in i[0].c = Inf, i[0].d = angle of
//the closest point
void IntersectOrClosestAngle(const WorkspaceBound& b,const AxisSweptPoint& a,AngleSet& i)
{
  if(!b.Intersects(a,i)) {
    i.resize(1);
    i[0].c = Inf;
    i[0].d = b.ClosestPoint(a);
  }
}

struct SawtoothAngleEnvelope : public XMonotoneChain
{
  void setInterval(const AngleInterval& i);
  void setAngles(const AngleSet& angles);
  Real minimumInterval(Real a,Real b,Real* x);
  void flip_y();
};

void SawtoothAngleEnvelope::flip_y()
{
  for(size_t i=0;i<v.size();i++)
    v[i].y = -v[i].y;
}

void SawtoothAngleEnvelope::setAngles(const AngleSet& angles)
{
  Assert(!angles.empty());
  setInterval(angles[0]);
  flip_y();
  SawtoothAngleEnvelope temp;
  for(size_t i=1;i<angles.size();i++) {
    temp.setInterval(angles[i]);
    temp.flip_y();
    upperEnvelope(temp);
  }
  flip_y();

  /*
  LOG4CXX_INFO(KrisLibrary::logger(),"Sawtooth ");
  for(size_t i=0;i<v.size();i++) {
    LOG4CXX_INFO(KrisLibrary::logger(),v[i]<<", ");
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  */
}

void SawtoothAngleEnvelope::setInterval(const AngleInterval& i)
{
  Assert(!i.isEmpty());
  Vector2 amax,amin;
  amin.set(i.c+Half*i.d,-Half*i.d);
  amax.set(amin.x+Pi,amin.y+Pi);
  amin.x = AngleNormalize(amin.x);
  amax.x = AngleNormalize(amax.x);
  v.resize(4);
  if(amin.x < amax.x) {
    //hi lo hi lo
    v[0].x=0; v[0].y = amin.y+amin.x;
    v[1] = amin;
    v[2] = amax;
    v[3].x=TwoPi; v[3].y = amax.y+amax.x-TwoPi;
    Assert(FuzzyEquals(v[3].y,v[0].y));
  }
  else {
    //lo hi lo hi
    v[0].x=0; v[0].y = amax.y-amax.x;
    v[1] = amax;
    v[2] = amin;
    v[3].x=TwoPi; v[3].y = amin.y+TwoPi-amin.x;
    Assert(FuzzyEquals(v[3].y,v[0].y));
  }
  if(v[2].x == v[3].x) v.erase(v.begin()+3);
  if(v[0].x == v[1].x) v.erase(v.begin());
  Assert(isValid());
}

Real SawtoothAngleEnvelope::minimumInterval(Real a,Real b,Real* x)
{
  Assert(!v.empty());
  //delete least endpoints at 0 and 2pi
  Real maxZero=v[0].y;
  int maxZeroInd=0;
  int lastZero=0;
  Real max2Pi=v.back().y;
  int max2PiInd=(int)v.size()-1;
  int first2Pi=-1;
  if(!FuzzyZero(v[0].x) || !FuzzyEquals(v.back().x,TwoPi)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SawtoothAngleEnvelope: Error in the upper envelope function?");
    for(size_t i=0;i<v.size();i++)
      LOG4CXX_ERROR(KrisLibrary::logger(),v[i]<<", ");
    LOG4CXX_ERROR(KrisLibrary::logger(),"\n");
    KrisLibrary::loggerWait();
  }
  Assert(FuzzyZero(v[0].x) && FuzzyEquals(v.back().x,TwoPi));
  for(size_t i=0;i<v.size();i++) {
    if(FuzzyEquals(v[i].x,0)) {
      if(v[i].y > maxZero) {
	maxZero=v[i].y;
	maxZeroInd=(int)i;
      }
      lastZero = (int)i;
    }
    if(FuzzyEquals(v[i].x,TwoPi)) {
      if(first2Pi == -1) first2Pi=(int)i;
      if(v[i].y > max2Pi) {
	max2Pi=v[i].y;
	max2PiInd=(int)i;
      }
    }
  }
  if(first2Pi <= 0) {
    LOG4CXX_INFO(KrisLibrary::logger(),"What?? strange set of sawtooth intervals, should be in [0,2pi]");
    for(size_t i=0;i<v.size();i++) 
      LOG4CXX_INFO(KrisLibrary::logger(),v[i]<<", ");
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    KrisLibrary::loggerWait();
  }
  Assert(first2Pi > 0);
  if(first2Pi != (int)v.size()-1)
    v.erase(v.begin()+first2Pi+1,v.end());
  if(lastZero!=0)
    v.erase(v.begin(),v.begin()+lastZero);
  Assert(!v.empty());
  Assert(FuzzyZero(v.front().x));
  v.front().x=0;
  v.front().y=maxZero;
  Assert(FuzzyEquals(v.back().x,TwoPi));
  v.back().x=TwoPi;
  v.back().y=max2Pi;

  Assert(a <= b);
  Real xmin=a;
  Real ymin=Inf;
  if(a < 0) {
    if(b < 0) // get the angles from 2Pi+a to 2Pi+b
      return minimum(a+TwoPi,b+TwoPi,x);
    else {
      Real xtemp,ytemp;
      //get the angles from 2Pi+a to 2Pi, then 0 to b
      ymin=minimum(TwoPi+a,TwoPi,&xmin);
      ytemp=minimum(0,b,&xtemp);
      if(ytemp < ymin) { ymin=ytemp; xmin=xtemp; }
      if(x) *x=xmin;
      return ymin;
    }
  }
  else {
    return minimum(a,b,x);
  }
}



void SampleTransform(const IKGoal& goal,RigidTransform& T)
{
  Assert(goal.posConstraint==IKGoal::PosFixed);
  switch(goal.rotConstraint) {
  case IKGoal::RotFixed:
    goal.GetFixedGoalTransform(T);
    break;
  case IKGoal::RotTwoAxis:
    FatalError("Can't have 2 axis terms rotation!");
    break;
  case IKGoal::RotAxis:
    {
      Real theta = Rand()*TwoPi;
      goal.GetEdgeGoalTransform(theta,T);
    }
    break;
  case IKGoal::RotNone:
    {
      QuaternionRotation q;
      RandRotation(q);
      q.getMatrix(T.R);
      T.t = goal.endPosition - T.R*goal.localPosition;
    }
    break;
  }
}

RLG::RLG(RobotKinematics3D& _robot, const JointStructure& _js)
  :robot(_robot),js(_js)
{}

void RLG::SampleFrom(int k)
{
  if(k==0) {
    IterateChild(k);
    return;
  }
  //iterate upward in the chain
  IterateParent(k);
  //iterate downward in the chain
  for(size_t j=0;j<js.children[k].size();j++) {
    int c=js.children[k][j];
    IterateChild(c);
  }
}

void RLG::Sample(const vector<IKGoal>& ik)
{
  int r=RandInt(ik.size());
  Sample(ik[r]);
}

void RLG::Sample(const IKGoal& start)
{
  //this fixes all of the joint positions attached to start.link
  SampleTransform(start,robot.links[start.link].T_World);
  SampleFrom(start.link);
}

void RLG::IterateParent(int k) 
{
  int p=robot.parents[k];
  if(p==-1) return;

  if(SolveAnalyticIKParent(k)) return;
  SampleParent(k);
  
  //process parent
  IterateParent(p);
  //process children
  for(size_t j=0;j<js.children[p].size();j++) {
    int c=js.children[p][j];
    if(c != k) IterateChild(c);
  }
}

void RLG::IterateChild(int k) 
{
  if(SolveAnalyticIKChild(k)) return;
  SampleChild(k);
  
  //process children
  for(size_t j=0;j<js.children[k].size();j++) {
    int c=js.children[k][j];
    IterateChild(c);
  }
}

void RLG::GetAnglesParent(int k,vector<AngleSet>& intervals) const
{
  int p=robot.parents[k];
  Assert(p!=-1);

  //Ti=Tp*T0_pi*Tloc(qk)  => Tp=Ti*Tloc(qk)^-1*T0_pk^-1
  //want to make sure for each joint pos x on p, Tp*x is in S
  //=>Tloc(qk)^-1*T0_pk^-1*x is in Tk^-1*S 
  //in this frame we can use the regular algorithm, and then flip the
  //joint range at the end because Tloc(qk)^-1 = Tloc(-qk)
  const RigidTransform& Tk=robot.links[k].T_World;
  const RigidTransform& T0_pk=robot.links[k].T0_Parent;
  RigidTransform T0_p;
  T0_p.mulInverseB(Tk,T0_pk);

  AngleSet itemp;
  intervals.reserve(1+js.children[p].size());
  intervals.resize(0);

  AxisSweptPoint a;
  a.axis.source = robot.links[k].T_World.t;
  robot.links[k].T_World.mulVector(robot.links[k].w,a.axis.direction);
  a.axis.direction.inplaceNegative();

  a.p = T0_p.t;
  IntersectOrClosestAngle(js.bounds[p],a,itemp);
  intervals.push_back(itemp);

  /*
  Vector3 blocal,ptLocal;
  //first the parent's joint
  const WorkspaceBound& bp=js.bounds[p];
  //get local position of bound blocal
  Tk.mulPointInverse(bp.center,blocal);
  T0_pk.mulPointInverse(Vector3(Zero),ptLocal);
  i = AngleBracket_3D_Ball(ptLocal,robot.links[k].w,blocal,bp.outerRadius+Epsilon);

  //LOG4CXX_INFO(KrisLibrary::logger(),"Interval "<<k<<": "<<i.c<<" -> "<<i.d);
  intervals.push_back(i);
  */

  //then children (!= k)
  for(size_t j=0;j<js.children[p].size();j++) {
    int c=js.children[p][j];
    if(c==k) continue;
    T0_p.mulPoint(robot.links[c].T0_Parent.t,a.p);
    IntersectOrClosestAngle(js.bounds[c],a,itemp);
    intervals.push_back(itemp);
    /*
    const WorkspaceBound& bc=js.bounds[c];
    Tk.mulPointInverse(bc.center,blocal);
    T0_pk.mulPointInverse(robot.links[c].T0_Parent.t,ptLocal);
    i=AngleBracket_3D_Ball(ptLocal,robot.links[k].w,blocal,bc.outerRadius+Epsilon);
    intervals.push_back(i);
    */
  }

  //then point bounds
  for(size_t j=0;j<js.linkPoints[p].size();j++) {
    T0_p.mulPoint(js.linkPoints[p][j],a.p);
    IntersectOrClosestAngle(js.pointBounds[p][j],a,itemp);
    intervals.push_back(itemp);
    /*
    const WorkspaceBound& bp=js.pointBounds[p][j];
    Tk.mulPointInverse(bp.center,blocal);
    T0_pk.mulPointInverse(js.linkPoints[p][j],ptLocal);
    i=AngleBracket_3D_Ball(ptLocal,robot.links[k].w,blocal,bp.outerRadius+Epsilon);
    intervals.push_back(i);
    */
  }

  /*
  for(size_t j=0;j<intervals.size();j++)
    Assert(!intervals[j].empty());
  */
}

void RLG::GetAnglesChild(int k,vector<AngleSet>& intervals) const
{
  //k's initial transform is T0k=Tp*T0_kp
  //the final transform is Tc=T(qk)*T0k

  RigidTransform T0;
  int p=robot.parents[k];
  if(p<0) T0=robot.links[k].T0_Parent;
  else T0.mul(robot.links[p].T_World,robot.links[k].T0_Parent);

  AngleSet itemp;
  intervals.reserve(js.children[k].size());
  intervals.resize(0);

  AxisSweptPoint a;
  a.axis.source = T0.t;
  T0.mulVector(robot.links[k].w,a.axis.direction);

  //Vector3 blocal;
  //first children
  for(size_t j=0;j<js.children[k].size();j++) {
    int c=js.children[k][j];
    T0.mulPoint(robot.links[c].T0_Parent.t,a.p);
    IntersectOrClosestAngle(js.bounds[c],a,itemp);
    intervals.push_back(itemp);
  }

  //then point bounds
  for(size_t j=0;j<js.linkPoints[k].size();j++) {
    T0.mulPoint(js.linkPoints[k][j],a.p);
    IntersectOrClosestAngle(js.pointBounds[k][j],a,itemp);
    intervals.push_back(itemp);
  }

  /*
  for(size_t j=0;j<intervals.size();j++)
    Assert(!intervals[j].empty());
  */
}

Real Sample(const AngleSet& angles)
{
  Real sum=0;
  for(size_t i=0;i<angles.size();i++)
    sum += angles[i].d;
  Real u=Rand()*sum;
  for(size_t i=0;i<angles.size();i++) {
    if(u <= angles[i].d) 
      return AngleNormalize(angles[i].c+u);
    u -= angles[i].d;
  }
  AssertNotReached();
  return 0;
}

//NOTE: requires empty angle sets to have one AngleInterval,
//whose c = Inf and d = closest angle
Real Sample(const vector<AngleSet>& angles,Real qmin,Real qmax)
{
  for(size_t j=0;j<angles.size();j++) {
    Assert(!angles[j].empty());
    if(angles[j].size() > 1) {
      for(size_t k=0;k<angles[j].size();k++)
	Assert(!angles[j][k].isEmpty());
    }
  }
  //try to satisfy all angles simultaneously
  AngleSet s; s.resize(1);
  s[0].setRange(AngleNormalize(qmin),AngleNormalize(qmax));
  for(size_t j=0;j<angles.size();j++) {
    /*
    if(angles[j].size() > 1) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Intersect "<<s<<" with "<<angles[j]); 
    }
    */
    s.Intersect(angles[j]);
    /*
    if(angles[j].size() > 1) {
      LOG4CXX_INFO(KrisLibrary::logger(),"= "<<s); 
    }
    */
    if(s.empty()) break;
  }
  if(s.empty()) {
    if(angles.size() == 1 && angles[0][0].isEmpty()) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Setting automatic point interval "<<angles[0][0]);
      return FixAngle(angles[0][0].d,qmin,qmax);
    }
    else {
      //we've gotta do some kind of biasing toward valid values
      
      //minimax distance from angles
      //upper envelope of sawtooth function in range 0,2pi
      SawtoothAngleEnvelope temp,env;
      for(size_t j=0;j<angles.size();j++) {
	//in the case of empty angles, the closest valid value is stored in i.d
	if(angles[j][0].isEmpty()) {
	  AngleInterval itemp; itemp.setPoint(angles[j][0].d);
	  temp.setInterval(itemp);
	}
	else {
	  temp.setAngles(angles[j]);
	}

	env.upperEnvelope(temp);
      }

      Real q;
      Real v = env.minimumInterval(qmin,qmax,&q);
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"From desired angles in "<<qmin<<" "<<qmax);
      for(size_t i=0;i<angles.size();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),angles[i]);
      LOG4CXX_INFO(KrisLibrary::logger(),"Sampling "<<q);
      LOG4CXX_INFO(KrisLibrary::logger(),"Not satisfying angles by "<<v);
      */
      return q;
    }
  }
  else {
    return Sample(s);
  }
}

//Given a fixed seed link k, generates a joint value q(k)
//and sets the parent's transform.
//This transform is such that Ti=Tp*T(qk) (i.e. the correct relative
//transform given by q(k).
//The generated q(k) is such that all the joints of the parent should
//be contained within their respective workspace bounds.
void RLG::SampleParent(int k)
{
  int p=robot.parents[k];
  Assert(p!=-1);

  vector<AngleSet> a;
  GetAnglesParent(k,a);

  robot.q(k) = ::Sample(a,robot.qMin(k),robot.qMax(k));
  FixAngleWarn(robot.q(k),robot.qMin(k),robot.qMax(k));
  Frame3D Tloc,Ttemp;
  robot.links[k].GetLocalTransform(robot.q(k),Tloc); Tloc.inplaceInverse();
  Ttemp.setInverse(robot.links[k].T0_Parent);
  robot.links[p].T_World = robot.links[k].T_World*Tloc*Ttemp;
}



//Given a fixed seed link p=parent(k), generates joint values q(c)
//and sets the transform for all children c
//The generated q(c) is such that all the joints of qc should
//be contained within their respective bounds
void RLG::SampleChild(int k)
{
  vector<AngleSet> a;
  GetAnglesChild(k,a);

  /*
    LOG4CXX_INFO(KrisLibrary::logger(),"Angle sets for "<<k);
    for(size_t i=0;i<a.size();i++) {
      LOG4CXX_INFO(KrisLibrary::logger(),a[i]);
    }
  */

  robot.q(k) = ::Sample(a,robot.qMin(k),robot.qMax(k));
  FixAngleWarn(robot.q(k),robot.qMin(k),robot.qMax(k));

  Frame3D Tloc;
  robot.links[k].GetLocalTransform(robot.q(k),Tloc);
  int p=robot.parents[k];
  if(p<0) robot.links[k].T_World = robot.links[k].T0_Parent;
  else robot.links[k].T_World.mul(robot.links[p].T_World,robot.links[k].T0_Parent);
  robot.links[k].T_World *= Tloc;
}



FreeRLG::FreeRLG(RobotKinematics3D& robot, const JointStructure& js)
  :RLG(robot,js)
{}

void FreeRLG::SampleFromRoot()
{
  for(size_t j=0;j<js.children[5].size();j++) {
    int c=js.children[5][j];
    IterateChild(c);
  }
}

bool FreeRLG::SolveAnalyticIKParent(int k) const
{
  if(k!=5) return false;
  //given the transform at link[5], solve for joint angles 0...5
  const RigidTransform& T=robot.links[5].T_World;
  const RigidTransform& T0=robot.links[0].T0_Parent;
  for(int i=0;i<3;i++) {
    Assert(robot.links[i].type == RobotLink3D::Prismatic);
    Assert(FuzzyEquals(robot.links[i].w.normSquared(),One));
    robot.q(i) = dot(robot.links[i].w,T.t-T0.t);
  }
  for(int i=3;i<6;i++) {
    Assert(robot.links[i].type == RobotLink3D::Revolute);
    Assert(FuzzyEquals(robot.links[i].w.normSquared(),One));
  }
  int u=-1,v=-1,w=-1;
  for(int i=0;i<3;i++) {
    if(robot.links[3].w[i]==1) u=i;
    if(robot.links[4].w[i]==1) v=i;
    if(robot.links[5].w[i]==1) w=i;
  }
  EulerAngleRotation e;
  e.setMatrix(u,v,w,T.R);
  robot.q(3)=e.x;
  robot.q(4)=e.y;
  robot.q(5)=e.z;
  return true;
}

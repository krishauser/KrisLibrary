#include "DenavitHartenberg.h"
using namespace Math3D;


///Alpha and theta given in radians
RigidTransform DenavitHartenbergTransform(Real alpha,Real a,Real d,Real theta)
{
  Real c=Cos(theta),s=Sin(theta);
  Real ca=Cos(alpha),sa=Sin(alpha);
  RigidTransform T;
  T.R(0,0) = c;    T.R(0,1) = -s;     T.R(0,2) = 0;
  T.R(1,0) = s*ca; T.R(1,1) = c*ca;   T.R(1,2) = -sa;
  T.R(2,0) = s*sa; T.R(2,1) = c*sa;   T.R(2,2) = ca;
  T.t.set(a,-sa*d,ca*d);
  /*
  RigidTransform T;
  T.R(0,0) = c;    T.R(0,1) = -s*ca;  T.R(0,2) = s*sa;
  T.R(1,0) = s;    T.R(1,1) = c*ca;   T.R(1,2) = -c*sa;
  T.R(2,0) = 0;    T.R(2,1) = sa;     T.R(2,2) = ca;
  T.t.set(a*c,a*s,d);
  */
  return T;
}

///Set up the frames and revolute axes for the robot, given the DH parameters
void DenavitHartenbergRobotSetup(const std::vector<Real>& alpha,const std::vector<Real>& a,const std::vector<Real>& d,const std::vector<Real>& theta,RobotKinematics3D& robot)
{
  Assert(alpha.size() == a.size());
  Assert(alpha.size() == d.size());
  if(robot.links.empty()) { //setup links
    robot.Initialize(alpha.size());
    for(size_t i=0;i<robot.parents.size();i++) 
      robot.parents[i] = ((int)i)-1;
    for(size_t i=0;i<robot.links.size();i++) {
      robot.links[i].mass = 1.0;
      robot.links[i].inertia.setIdentity();
      robot.links[i].com.setZero();
    }
  }
  else {
    Assert(robot.links.size() == alpha.size());
  }
  for(size_t i=0;i<robot.links.size();i++) {
    robot.links[i].type = RobotLink3D::Revolute;
    robot.links[i].w.set(0,0,1);
    robot.links[i].T0_Parent = DenavitHartenbergTransform(alpha[i],a[i],d[i],theta[i]);
  }
  robot.UpdateFrames();
}

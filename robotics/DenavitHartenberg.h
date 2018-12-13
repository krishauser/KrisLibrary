#ifndef DENAVIT_HARTENBERG_H
#define DENAVIT_HARTENBERG_H

#include "RobotKinematics3D.h"

///Alpha and theta given in radians
Math3D::RigidTransform DenavitHartenbergTransform(Real alpha,Real a,Real d,Real theta=0);

///Set up the frames and revolute axes for the robot, given the DH parameters
void DenavitHartenbergRobotSetup(const std::vector<Real>& alpha,const std::vector<Real>& a,const std::vector<Real>& d,const std::vector<Real>& theta,RobotKinematics3D& robot);


#endif

#ifndef ROBOTICS_KINEMATICS_H
#define ROBOTICS_KINEMATICS_H

#include "RobotKinematics3D.h"

/// Returns the maximum length between the joints of link1 and link2.
Real MaxJointDistance(const RobotKinematics3D&,int link1,int link2);

/// Computes the matrix of maximum joint-joint distances (of the frame origins)
void ComputeJointDistances(const RobotKinematics3D& robot,std::vector<std::vector<Real> >& dist);

/// Computes the maximal link-link distance on the robot
Real MaxLimbSpan(const RobotKinematics3D& robot);

#endif

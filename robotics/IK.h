#ifndef ROBOTICS_IK_H
#define ROBOTICS_IK_H

#include <KrisLibrary/math3d/primitives.h>
#include <vector>
using namespace Math3D;

/** @ingroup Kinematics
 * @brief A structure defining a link's desired configuration for IK.
 *
 * Position and/or rotation can be constrained, as well as
 * individual components of each.
 */
struct IKGoal
{
  enum PosConstraint { PosNone=0, PosPlanar=1, PosLinear=2, PosFixed=3 };
  enum RotConstraint { RotNone=0, RotTwoAxis=1, RotAxis=2, RotFixed=3 };

  /// Returns the number of operational space dof's of the given constraint
  inline static int NumDims(PosConstraint c) { return (int)c; }
  inline static int NumDims(RotConstraint c) { return (int)c; }

  IKGoal();
  ///Contrains localPosition to be fixed at pos.  NOTE: localPosition is not
  ///set here!
  void SetFixedPosition(const Vector3& pos) { posConstraint=PosFixed; endPosition=pos; }
  ///Contrains localPosition to lie along a plane passing through point with
  ///normal normal.  NOTE: localPosition is not set here!
  void SetPlanarPosition(const Vector3& point,const Vector3& normal);
  ///Contrains localPosition to lie along a line passing through point with
  ///direction normal.  NOTE: localPosition is not set here!
  void SetLinearPosition(const Vector3& point,const Vector3& direction);
  ///Removes the position constraint
  void SetFreePosition() { posConstraint=PosNone; }

  ///Contrains the link to have a fixed rotation matrix
  void SetFixedRotation(const Matrix3& R);
  ///Contrains the link to have a fixed rotation moment
  void SetFixedRotation(const Vector3& mR) { rotConstraint=RotFixed; endRotation=mR; }
  ///Contrains the link's orientation so that localAxis is mapped onto
  ///worldAxis
  void SetAxisRotation(const Vector3& localAxis,const Vector3& worldAxis);
  ///Removes the rotation constraint
  void SetFreeRotation() { rotConstraint=RotNone; }

  ///Sets a fixed goal transformation
  void SetFixedTransform(const RigidTransform& T);

  ///Fits a constraint that maps the local positions onto the world positions.
  ///If 0 points are specified, the constraint is free.  If the points are
  ///all equal, then the constraint is a point constraint.  If the points
  ///lie on a line, then the constraint is a hinge constraint.
  void SetFromPoints(const std::vector<Vector3>& localpos,const std::vector<Vector3>& worldpos,Real degeneracyTol=Epsilon);
  /// Removes the position constraint through this axis
  void RemovePositionAxis(const Vector3& axis);
  /// Removes the rotation constraint about the axis.
  void RemoveRotationAxis(const Vector3& axis);
  /// Removes the rotation constraint about the axis through the point p.
  void RemoveRotationAxis(const Vector3& p,const Vector3& axis);
  /// Inplace transformation of the goal position/orientation
  void Transform(const RigidTransform& T);
    /// Inplace transformation of the local position/orientation
  void TransformLocal(const RigidTransform& T);

  /// Returns the error in the current configuration given the current
  /// relative transform between link -> destlink
  void GetError(const RigidTransform& Trel,Real posErr[3],Real rotErr[3]) const;
  void GetFixedGoalRotation(Matrix3& R) const;
  void GetFixedGoalTransform(RigidTransform& T) const;
  void GetBaseEdgeRotation(Matrix3& R0) const;
  void GetEdgeGoalTransform(Real theta,RigidTransform& T) const;

  ///Gets a transformation T that satisfies the IK goal's constraints, and
  ///is closest to the transform T0.  Useful for non-fixed IKGoals.
  void GetClosestGoalTransform(const RigidTransform& T0,RigidTransform& T) const;

  ///sets the endPosition / endRotation values so that if the current relative transform
  ///between the link and its target is Trel, the objective's error is 0
  void MatchGoalTransform(const RigidTransform& Trel);

  /// Robot link index
  int link;
  /// Desintation link index (-1 means world frame)
  int destLink;

  /// Position terms:
  PosConstraint posConstraint;
  /// End effector position in the local frame
  Vector3 localPosition;
  /// The desired end effector position
  Vector3 endPosition;
  /// For plane position constraints, the normal of the plane. <br>
  /// For line position constraints, the direction of the line.
  Vector3 direction;

  /// Rotation terms:
  RotConstraint rotConstraint;
  /// If in axis-constrained rotation, the local rot axis.
  Vector3 localAxis;
  /// The desired end effector rotation, that is, for:
  /// Axis-constrained rotation: the desired axis.
  /// Fixed rotation: the moment of the desired rotation R=e^[endRotation]
  Vector3 endRotation;
};

std::istream& operator >> (std::istream& in,IKGoal& data);
std::ostream& operator << (std::ostream& out,const IKGoal& data);

#endif

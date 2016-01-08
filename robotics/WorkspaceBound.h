#ifndef ROBOTICS_WORKSPACE_BOUND_H
#define ROBOTICS_WORKSPACE_BOUND_H

#include <KrisLibrary/math3d/Line3D.h>
#include <KrisLibrary/math3d/Circle3D.h>
#include <KrisLibrary/math3d/Sphere3D.h>
#include <KrisLibrary/math/angle.h>
#include <vector>
using namespace Math3D;
using namespace std;

namespace Math {
  struct AngleSet;
  struct ClosedIntervalSet;
} //namespace Math


/** @ingroup Kinematics
 * @brief Parameterizes a circle in space by sweeping a point around a line.
 * 
 * If R(theta) is a rotation about the line 'axis' by angle theta, then
 * this parameterizes a circle by the set of points 
 * { R(theta)*p | theta in [0,2pi) }.
 */
struct AxisSweptPoint
{
  Vector3 center() const;
  Real radius() const;
  void getCircle(Circle3D&) const;
  Vector3 eval(Real theta) const;

  Line3D axis;
  Vector3 p;
};

/** @ingroup Kinematics
 * @brief An arc segment of a circle in space.
 */
struct Arc3D : public AxisSweptPoint
{
  AngleInterval interval;
};

/** @ingroup Kinematics
 * @brief A 3-D annulus innerRadius <= |x-center| <= outerRadius.
 */
struct HollowBall
{
  inline void setFull()
  { center.setZero(); outerRadius = Inf; innerRadius = -Inf; }
  inline void setEmpty()
  { center.setZero(); outerRadius = -Inf; innerRadius = Inf; }
  inline void setPoint(const Vector3& pt)
  { center = pt; outerRadius = 0; innerRadius = -Inf; }
  inline void setSphere(const Sphere3D& s)
  { center = s.center; outerRadius = innerRadius = s.radius; }
  inline void setBall(const Sphere3D& s)
  { center = s.center; outerRadius = s.radius; innerRadius = -Inf; }
  inline void setBallComplement(const Sphere3D& s)
  { center = s.center; innerRadius = s.radius; outerRadius = Inf; }
  inline bool isEmpty() const { return outerRadius < 0; }
  inline bool isFull() const { return IsInf(outerRadius)==1 && innerRadius<0; }
  inline bool isBall() const { return innerRadius < 0; }
  inline bool isSphere() const { return innerRadius == outerRadius; }
  inline void getOuterSphere(Sphere3D& s) const
  { s.center=center; s.radius=outerRadius; }
  inline void getInnerSphere(Sphere3D& s) const
  { s.center=center; s.radius=innerRadius; }
  bool contains(const Vector3& pt) const;
  bool contains(const HollowBall& b) const;
  bool containsBall(const Sphere3D& s) const;
  bool containsSphere(const Sphere3D& s) const;
  bool intersects(const Line3D& line, ClosedIntervalSet& intervals) const;
  bool intersects(const AxisSweptPoint& circle, AngleSet& arcs) const;
  bool intersects(const Arc3D& arc, AngleSet& arcs) const;
  //returns the parameter of the line that is the closest point
  Real closestPoint(const Line3D& line) const;
  //returns the parameter of the swept point that is the closest point
  Real closestPoint(const AxisSweptPoint& circle) const;

  Vector3 center;
  Real outerRadius;
  Real innerRadius;
};

ostream& operator <<(ostream& out,const AngleInterval& i);
ostream& operator <<(ostream& out,const AngleSet& i);

/** @ingroup Kinematics
 * @brief Bounds the workspace of a robot linkage with a simple bound.
 *
 * The bound is currently an intersection of the (hollow) balls in balls.
 * There's a crude angle bound called maxAngle that doesn't do very much.
 */
struct WorkspaceBound
{
  WorkspaceBound();
  void SetEmpty();
  void SetFull();
  void SetPoint(const Vector3&);
  void SetArc(const Arc3D&);
  void SetCircle(const Circle3D&);
  void SetSphere(const Sphere3D&);
  void SetTransformed(const WorkspaceBound&, RigidTransform& T);
  //sets the bound as a intersect b
  bool SetIntersection(const WorkspaceBound& a, const WorkspaceBound& b);
  bool InplaceIntersection(const WorkspaceBound& b);
  //sets the bound as a+b where + is the minkowski sum
  void SetMinkowskiSum(const WorkspaceBound& a, const WorkspaceBound& b);
  void InplaceMinkowskiSum(const WorkspaceBound& b);
  void RemoveRedundancies();
  bool IsEmpty() const;
  bool IsFull() const;
  bool IsBall() const;
  Real GetWidth() const;
  void GetBounds(AABB3D&) const;
  void GetBounds(Sphere3D&) const;
  void GetBounds(HollowBall&) const;
  bool Contains(const Vector3& pt) const;
  bool Intersects(const Line3D& line, ClosedIntervalSet& intervals) const;
  bool Intersects(const AxisSweptPoint& circle, AngleSet& arcs) const;
  bool Intersects(const Arc3D& arc, AngleSet& arcs) const;
  Real ClosestPoint(const Line3D& line) const;
  Real ClosestPoint(const AxisSweptPoint& circle) const;

  std::vector<HollowBall> balls; 
  Real maxAngle;
};

#if 0
struct WorkspaceBound
{
  WorkspaceBound();
  void SetEmpty();
  void SetFull();
  void SetPoint(const Vector3&);
  void SetArc(const Arc&);
  void SetCircle(const Circle3D&);
  void SetSphere(const Sphere3D&);
  void SetTransformed(const WorkspaceBound&, RigidTransform& T);
  //sets the bound as a intersect b
  bool SetIntersection(const WorkspaceBound& a, const WorkspaceBound& b);
  bool InplaceIntersection(const WorkspaceBound& b);
  //sets the bound as a+b where + is the minkowski sum
  void SetMinkowskiSum(const WorkspaceBound& a, const WorkspaceBound& b);
  void InplaceMinkowskiSum(const WorkspaceBound& b);
  bool IsEmpty() const;
  bool IsSphere() const;
  Real GetWidth() const;
  void GetBounds(AABB3D&) const;
  void GetBounds(Sphere3D&) const;
  bool Intersects(const Line3D& line, ClosedIntervalSet& intervals) const;
  bool Intersects(const AxisSweptPoint& circle, AngleSet& arcs) const;
  bool Intersects(const Arc& arc, AngleSet& arcs) const;
  Real ClosestPoint(const Line3D& line) const;
  Real ClosestPoint(const AxisSweptPoint& circle) const;

  Vector3 center;
  Real outerRadius;
  Real innerRadius;
  Real maxAngle;
};
#endif

#endif

#ifndef MATH3D_GEOMETRY2D_H
#define MATH3D_GEOMETRY2D_H

#include "AABB2D.h"
#include "Line2D.h"
#include "Ray2D.h"
#include "Segment2D.h"
#include "Plane2D.h"
#include "Triangle2D.h"
#include "Box2D.h"
#include "Circle2D.h"
//#include "Ellipse2D.h"
#include "Polygon2D.h"
#include <KrisLibrary/utils/AnyValue.h>
#include <vector>

namespace Math3D {

/** @brief A generic 2D geometric primitive class.
 *
 * Note: Distance() routines may actually report the signed distance,
 * rather than 0 if there is an overlap.
 */
class GeometricPrimitive2D
{
 public:
  enum Type { Empty, Point, Segment, AABB, Triangle, Circle, Box };

  GeometricPrimitive2D();
  GeometricPrimitive2D(const GeometricPrimitive2D& rhs);
  GeometricPrimitive2D(const Vector2& point);
  GeometricPrimitive2D(const Segment2D& segment);
  GeometricPrimitive2D(const AABB2D& aabb);
  GeometricPrimitive2D(const Box2D& box);
  GeometricPrimitive2D(const Circle2D& circle);
  GeometricPrimitive2D(const Triangle2D& triangle);
  static const char* TypeName(Type type);
  const char* TypeName() const { return GeometricPrimitive2D::TypeName(type); }
  void Set(const Vector2& point);
  void Set(const Segment2D& segment);
  void Set(const AABB2D& aabb);
  void Set(const Box2D& box);
  void Set(const Circle2D& circle);
  void Set(const Triangle2D& triangle);
  AABB2D GetAABB() const;
  Box2D GetBB() const;
  RigidTransform GetFrame() const;
  void Transform(const RigidTransform2D& T);
  void ToPolygon(std::vector<Vector2>& outline,int divs=32) const;

  static bool SupportsCollides(Type a,Type b);
  bool SupportsCollides(Type b) const { return GeometricPrimitive2D::SupportsCollides(type,b); }
  bool Collides(const GeometricPrimitive2D& geom) const;
  bool Collides(const Vector2& point) const;
  bool Collides(const Segment2D& segment) const;
  bool Collides(const AABB2D& aabb) const;
  bool Collides(const Box2D& box) const;
  bool Collides(const Circle2D& circle) const;
  bool Collides(const Triangle2D& triangle) const;
  static bool SupportsDistance(Type a,Type b);
  bool SupportsDistance(Type b)  { return GeometricPrimitive2D::SupportsDistance(type,b); }
  Real Distance(const GeometricPrimitive2D& geom) const;
  Real Distance(const Vector2& x) const;
  Real Distance(const Segment2D& segment) const;
  Real Distance(const AABB2D& aabb) const;
  Real Distance(const Box2D& box) const;
  Real Distance(const Circle2D& circle) const;
  Real Distance(const Triangle2D& triangle) const;

  Type type;
  AnyValue data;
};

} //Math3D

#endif


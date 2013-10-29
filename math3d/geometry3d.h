#ifndef MATH3D_GEOMETRY3D_H
#define MATH3D_GEOMETRY3D_H

#include "AABB3D.h"
#include "Line3D.h"
#include "Ray3D.h"
#include "Segment3D.h"
#include "Plane3D.h"
#include "Triangle3D.h"
#include "Box3D.h"
#include "Sphere3D.h"
#include "Circle3D.h"
#include "Cylinder3D.h"
#include "Ellipsoid3D.h"
#include "Polygon3D.h"
#include "Polyhedron3D.h"
#include <utils/AnyValue.h>
#include <vector>
#include <iostream>

//TriMesh declaration
namespace Meshing {
  struct TriMesh;
};

namespace Math3D {

/** @brief A generic geometric primitive class.
 *
 * TODO: make the API comparable to GeometricPrimitive2D.
 */
class GeometricPrimitive3D
{
 public:
  enum Type { Empty, Point, Segment, Triangle, Polygon, Sphere, Ellipsoid, Cylinder, AABB, Box };

  GeometricPrimitive3D();
  GeometricPrimitive3D(const GeometricPrimitive3D& prim);
  GeometricPrimitive3D(const Vector3& pt);
  GeometricPrimitive3D(const Segment3D& line);
  GeometricPrimitive3D(const Triangle3D& t);
  GeometricPrimitive3D(const Polygon3D& p);
  GeometricPrimitive3D(const Sphere3D& s);
  GeometricPrimitive3D(const Ellipsoid3D& s);
  GeometricPrimitive3D(const Cylinder3D& s);
  GeometricPrimitive3D(const AABB3D& s);
  GeometricPrimitive3D(const Box3D& s);
  static const char* TypeName(Type type);
  const char* TypeName() const { return GeometricPrimitive3D::TypeName(type); }
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  RigidTransform GetFrame() const;
  void Transform(const RigidTransform& T);
  void Transform(const Matrix4& T);

  static bool SupportsCollides(Type a,Type b);
  bool SupportsCollides(Type b) const { return GeometricPrimitive3D::SupportsCollides(type,b); }
  bool Collides(const Vector3& pt) const;
  bool Collides(const Segment3D& s) const;
  bool Collides(const Triangle3D& t) const;
  bool Collides(const Polygon3D& p) const;
  bool Collides(const Sphere3D& s) const;
  bool Collides(const Ellipsoid3D& s) const;
  bool Collides(const Cylinder3D& s) const;
  bool Collides(const AABB3D& s) const;
  bool Collides(const Box3D& s) const;
  bool Collides(const GeometricPrimitive3D& g) const;
  static bool SupportsDistance(Type a,Type b);
  bool SupportsDistance(Type b)  { return GeometricPrimitive3D::SupportsDistance(type,b); }
  Real Distance(const Vector3& pt) const;
  Real Distance(const Segment3D& s) const;
  Real Distance(const Triangle3D& t) const;
  Real Distance(const Polygon3D& p) const;
  Real Distance(const Sphere3D& s) const;
  Real Distance(const Ellipsoid3D& s) const;
  Real Distance(const Cylinder3D& s) const;
  Real Distance(const AABB3D& s) const;
  Real Distance(const Box3D& s) const;
  Real Distance(const GeometricPrimitive3D& g) const;
  std::vector<double> ClosestPointParameters(const Vector3& pt) const;
  Vector3 ParametersToPoint(const std::vector<double>& params) const;
  Vector3 ParametersToNormal(const std::vector<double>& params) const;

  Type type;
  AnyValue data;
};

std::ostream& operator <<(std::ostream& out,const GeometricPrimitive3D& g);
std::istream& operator >>(std::istream& in,GeometricPrimitive3D& g);

} //namespace Math3D

#endif

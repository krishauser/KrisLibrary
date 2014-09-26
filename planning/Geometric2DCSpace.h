#ifndef GEOMETRIC_2D_CSPACE_H
#define GEOMETRIC_2D_CSPACE_H

#include "CSpace.h"
#include "ExplicitCSpace.h"
#include <math3d/geometry2d.h>
#include <vector>
using namespace Math3D;
using namespace std;

class Geometric2DCollection
{
 public:
  enum Type { AABB, Triangle, Circle, Box };

  int NumObstacles() const;
  Type ObstacleType(int obstacle) const;
  int ObstacleIndex(int obstacle) const;
  GeometricPrimitive2D Obstacle(int obstacle) const;
  const char* ObstacleTypeName(int obstacle) const;

  void Add(const Triangle2D& tri);
  void Add(const AABB2D& bbox);
  void Add(const Box2D& box);
  //void Add(const Polygon2D& poly);
  void Add(const Circle2D& sphere);
  void Add(const Geometric2DCollection& geom);
  void Add(const GeometricPrimitive2D& geom);
  void Clear();
  void DrawGL() const;
  void DrawOutlinesGL() const;
  void ToPolygons(vector<vector<Vector2> >& polys) const;

  //distance query
  Real Distance(const Vector2& x) const;
  Real Distance(const Circle2D& circle) const;
  //collision query
  bool Collides(const Vector2& x) const;
  bool Collides(const Segment2D& s) const;
  bool Collides(const Circle2D& circle) const;
  bool Collides(const Box2D& box) const;
  bool Collides(const Triangle2D& tri) const;
  bool Collides(const GeometricPrimitive2D& geom) const;
  bool Collides(const Geometric2DCollection& geom) const;
  //distance query
  Real Distance(const Vector2& x,int obstacle) const;
  Real Distance(const Circle2D& circle,int obstacle) const;
  //collision query
  bool Collides(const Vector2& x,int obstacle) const;
  bool Collides(const Segment2D& s,int obstacle) const;
  bool Collides(const Circle2D& circle,int obstacle) const;
  bool Collides(const Box2D& box,int obstacle) const;
  bool Collides(const Triangle2D& tri,int obstacle) const;
  bool Collides(const GeometricPrimitive2D& geom,int obstacle) const;
  bool Collides(const Geometric2DCollection& geom,int obstacle) const;

  void Transform(const RigidTransform2D& T);

  vector<AABB2D> aabbs;
  vector<Box2D> boxes;
  vector<Circle2D> circles;
  vector<Triangle2D> triangles;
};

/** @brief a 2D cspace whose obstacles are geometric primitives.
 *
 * The C-space obstacles are explicitly given as aabbs, boxes, triangles,
 * and spheres.
 */
class Geometric2DCSpace : public ExplicitCSpace, public Geometric2DCollection
{
public:
  Geometric2DCSpace();
  void DrawGL() const;

  Real ObstacleDistance(const Vector2& x) const;
  Real ObstacleDistance(const Circle2D& circle) const;
  bool ObstacleOverlap(const Segment2D& s) const;
  bool ObstacleOverlap(const Circle2D& circle) const;
  bool ObstacleOverlap(const Box2D& box) const;
  bool ObstacleOverlap(const Triangle2D& tri) const;
  bool ObstacleOverlap(const Segment2D& s,int obstacle) const;

  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual int NumObstacles();
  virtual std::string ObstacleName(int obstacle);
  virtual bool IsFeasible(const Config& x);
  virtual bool IsFeasible(const Config& x,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual Real Distance(const Config& x, const Config& y);
  virtual Real ObstacleDistance(const Config& x) { return ObstacleDistance(Vector2(x(0),x(1))); }
  virtual void Properties(PropertyMap&) const;

  bool euclideanSpace;
  Real visibilityEpsilon;
  AABB2D domain;
};

#endif

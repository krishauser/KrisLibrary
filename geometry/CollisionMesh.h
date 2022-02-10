#ifndef GEOMETRY_COLLISION_MESH_H
#define GEOMETRY_COLLISION_MESH_H

#include "AnyGeometryType.h"
#include "AnyGeometryTypeImpl.h"
#include <KrisLibrary/meshing/TriMeshTopology.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <limits.h>

class PQP_Model;
class PQP_Results;

namespace Geometry {

using namespace Math3D;
class ApproximatePenetrationDepth;



/** @ingroup Geometry
 * @brief A triangle mesh along with PQP bounding volume structures that
 * enable fast collision and other proximity queries.
 *
 * The TriMesh (a parent class of TriMeshWithTopology) must be initialized
 * first, then InitCollisions() must be called.
 * The current rigid-body transformation must be specified before making 
 * collision queries.
 * @sa CollisionMeshQuery
 */
class CollisionMesh : public Meshing::TriMeshWithTopology
{
 public:
  CollisionMesh();
  CollisionMesh(const CollisionMesh& model);
  explicit CollisionMesh(const Meshing::TriMesh& mesh);
  explicit CollisionMesh(const Meshing::TriMeshWithTopology& mesh);
  ~CollisionMesh();
  const CollisionMesh& operator = (const CollisionMesh& model);
  void InitCollisions();
  inline void UpdateTransform(const RigidTransform& f) {currentTransform = f;}
  void GetTransform(RigidTransform& f) const {f=currentTransform; }

  PQP_Model* pqpModel;
  RigidTransform currentTransform;
};

class Collider3DTriangleMesh : public Collider3D
{
public:
    Collider3DTriangleMesh(shared_ptr<Geometry3DTriangleMesh> data);
    Collider3DTriangleMesh(const Collider3DTriangleMesh& rhs);
    virtual ~Collider3DTriangleMesh() {}
    virtual shared_ptr<Geometry3D> GetData() const override{ return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset() override;
    virtual AABB3D GetAABB() const override;
    virtual AABB3D GetAABBTight() const override;
    virtual Box3D GetBB() const override;
    virtual RigidTransform GetTransform() const override { return collisionData.currentTransform; }
    virtual void SetTransform(const RigidTransform& T) override { collisionData.currentTransform = T; }
    virtual bool Contains(const Vector3& pt,bool& result) override;
    virtual bool Distance(const Vector3& pt,Real& result) override;
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) override;
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;
    virtual Collider3D* Copy() const override;
    virtual Collider3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) override;
    virtual Collider3D* Slice(const RigidTransform& T,Real tol=0) const override;
    virtual Collider3D* ExtractROI(const AABB3D& bb,int flag=1) const override;
    virtual Collider3D* ExtractROI(const Box3D& bb,int flag=1) const override;

    shared_ptr<Geometry3DTriangleMesh> data;
    CollisionMesh collisionData;
};


/** @ingroup Geometry
 * @brief A general-purpose distance querying class.
 *
 * Given two meshes, allows querying collision, tolerance, and distance using
 * PQP, or querying penetration depth using an approximate computation.
 *
 * All vectors p1, p2 are given in the local frames of m1 and m2 resp.
 * @sa CollisionMesh
 * @sa ApproximatePenetrationDepth
 */
class CollisionMeshQuery
{
 public:
  CollisionMeshQuery();
  CollisionMeshQuery(const CollisionMesh& m1,const CollisionMesh& m2);
  CollisionMeshQuery(const CollisionMeshQuery& q);
  ~CollisionMeshQuery();
  const CollisionMeshQuery& operator = (const CollisionMeshQuery& q);

  bool Collide();
  bool CollideAll();
  Real Distance(Real absErr,Real relErr,Real bound=Inf);
  Real Distance_Coherent(Real absErr,Real relErr,Real bound=Inf);
  bool WithinDistance(Real tol);
  bool WithinDistanceAll(Real tol);
  Real PenetrationDepth(); //note: calls CollideAll(), returns -0 if seperated

  Real Distance_Cached() const;
  Real PenetrationDepth_Cached() const;
  ///Returns the closest points between the mesh after a Distance call.  Results are in in local coordinates.
  void ClosestPoints(Vector3& p1,Vector3& p2) const;
  ///Returns the closest triangles
  void ClosestPair(int& t1,int& t2) const;
  ///Returns a pair of points defining a true result of a WithinDistance call.  Results are in in local coordinates.
  void TolerancePoints(Vector3& p1,Vector3& p2) const;
  ///Returns a pair of triangles triangles defining a true result of a WithinDistance call.
  void TolerancePair(int& t1,int& t2) const;
  //d1 is the direction that m1 can move to get out of m2 (in world coords).  p1 and p2 are in local coordinates, while d1 is in world coordinates
  void PenetrationPoints(Vector3& p1,Vector3& p2,Vector3& d1) const;
  //extracting the pairs of interacting features
  void CollisionPairs(std::vector<int>& t1,std::vector<int>& t2) const;
  ///Returns all pairs of points that are within the given tolerance of a WithinDistance call.  Results are in in local coordinates.
  void TolerancePoints(std::vector<Vector3>& p1,std::vector<Vector3>& t2) const;
  ///Returns all pairs of triangles that are within the given tolerance of a WithinDistance call.
  void TolerancePairs(std::vector<int>& t1,std::vector<int>& t2) const;

  const CollisionMesh *m1, *m2;

 private:
  PQP_Results* pqpResults;
  std::vector<int> tc1,tc2;   //temp, only updated on penetration depth call
  ApproximatePenetrationDepth *penetration1,*penetration2;
};

/* @brief A convenience class that allows meshes to be treated as
 * "fattened" geometry using a given margin.
 *
 * Negative margins are not supported.
 */
class CollisionMeshQueryEnhanced : public CollisionMeshQuery
{
 public:
  CollisionMeshQueryEnhanced();
  CollisionMeshQueryEnhanced(const CollisionMesh& m1,const CollisionMesh& m2);
  CollisionMeshQueryEnhanced(const CollisionMeshQueryEnhanced& q);
  ~CollisionMeshQueryEnhanced();

  bool Collide();
  bool CollideAll();
  Real Distance(Real absErr,Real relErr,Real bound=Inf);
  Real Distance_Coherent(Real absErr,Real relErr,Real bound=Inf);
  bool WithinDistance(Real tol);
  bool WithinDistanceAll(Real tol);
  Real PenetrationDepth(); //note: calls CollideAll(), returns -0 if seperated

  Real Distance_Cached() const;
  Real PenetrationDepth_Cached() const;
  void ClosestPoints(Vector3& p1,Vector3& p2) const;
  void TolerancePoints(Vector3& p1,Vector3& p2) const;
  //d1 is the direction that m1 can move to get out of m2 (in world coords)
  void PenetrationPoints(Vector3& p1,Vector3& p2,Vector3& d1) const;
  //extracting the pairs of interacting features
  void CollisionPairs(std::vector<int>& t1,std::vector<int>& t2) const;
  void TolerancePairs(std::vector<int>& t1,std::vector<int>& t2) const;
  void TolerancePoints(std::vector<Vector3>& p1,std::vector<Vector3>& t2) const;
  Real margin1,margin2;
};



/** @addtogroup Geometry */
/**\@{*/

/// Returns the bounding box containing m
void GetBB(const CollisionMesh& m,Box3D& bb);

/// Checks for collision between s and m.  Returns the
/// index of the tri, or -1 if none, and computes the intersecting point pt.
int Collide(const CollisionMesh& m,const Segment3D& s,Vector3& pt);
/// Returns true if m intersects the given geometry
bool Collide(const CollisionMesh& m,const Sphere3D& s);
bool Collide(const CollisionMesh& m,const AABB3D& b);
bool Collide(const CollisionMesh& m,const Box3D& b);
bool Collide(const CollisionMesh& m1,const CollisionMesh& m2);
bool Collide(const CollisionMesh& m,const GeometricPrimitive3D& g);
bool Collide(const CollisionMesh& m,const Plane3D& p);

///Casts a ray at the mesh. Returns the index of the first triangle hit
///(-1 if none) and stores the colliding point in pt (given in world coordinates).
int RayCast(const CollisionMesh& m,const Ray3D& r,Vector3& pt);

///Same as RayCast, but the ray (and colliding point) are in the local
///frame of the mesh
int RayCastLocal(const CollisionMesh& m,const Ray3D& r,Vector3& pt);

/// Computes a list of triangles that overlap the geometry
void CollideAll(const CollisionMesh& m,const Sphere3D& s,std::vector<int>& tris,int max=INT_MAX);
void CollideAll(const CollisionMesh& m,const Segment3D& s,std::vector<int>& tris,int max=INT_MAX);
void CollideAll(const CollisionMesh& m,const AABB3D& bb,std::vector<int>& tris,int max=INT_MAX);
void CollideAll(const CollisionMesh& m,const Box3D& b,std::vector<int>& tris,int max=INT_MAX);
void CollideAll(const CollisionMesh& m,const GeometricPrimitive3D& g,std::vector<int>& tris,int max=INT_MAX);
void CollideAll(const CollisionMesh& m,const Plane3D& p,std::vector<int>& tris,int max=INT_MAX);


///Returns true if m is within distance d of the geometry
bool WithinDistance(const CollisionMesh& m,const Vector3& p,Real d);
bool WithinDistance(const CollisionMesh& m,const GeometricPrimitive3D& g,Real d);
bool WithinDistance(const CollisionMesh& m1,const CollisionMesh& m2,Real d);

/// Computes the triangles in m within distance d to p on m 
void NearbyTriangles(const CollisionMesh& m,const Vector3& p,Real d,std::vector<int>& tris,int max=INT_MAX);
void NearbyTriangles(const CollisionMesh& m,const GeometricPrimitive3D& g,Real d,std::vector<int>& tris,int max=INT_MAX);
void NearbyTriangles(const CollisionMesh& m1,const CollisionMesh& m2,Real d,std::vector<int>& tris1,std::vector<int>& tris2,int max=INT_MAX);

///Returns the nearest distance from any point in m to g.  O(log n) running time.
///If the distance is > bound, then this terminates early and returns bound.
Real Distance(const CollisionMesh& m,const Vector3& p,Real bound=Inf);
///Same as the above, but with additional outputs
///
///Outputs
///- surfacePt is the closest point on the surface, in world coordinates
///- direction is the unit normal from the surface to p, in world coordinates
Real Distance(const CollisionMesh& m,const Vector3& p,int& closestTri,Vector3& surfacePt,Vector3& direction,Real bound=Inf);
///Returns the nearest distance from any point in m to g.  O(log n) running time for point / sphere, O(n) otherwise.
///If the distance is > bound, then this terminates early and returns bound.
Real Distance(const CollisionMesh& m,const GeometricPrimitive3D& g,Real bound=Inf);
///Same as the above, but with additional outputs
///
///Outputs
///- surfacePt is the closest point on the surface, in world coordinates
///- direction is the unit normal from the surface to the closest / deepest point on g, in world coordinates
Real Distance(const CollisionMesh& m,const GeometricPrimitive3D& g,int& closestTri,Vector3& surfacePt,Vector3& direction,Real bound=Inf);
/// Checks distance between two meshes (convenience function, equivalent to m1.Distance(m2))
Real Distance(const CollisionMesh& m1,const CollisionMesh& m2,Real absErr,Real relErr,Real bound=Inf);

///Finds the closest point pt to p on m and returns the triangle index. The closest point is cplocal, given in the mesh's local frame
///If bound is given, this will quit and return -1 if the closest point is known to be farther than bound.
int ClosestPoint(const CollisionMesh& m,const Vector3& p,Vector3& cplocal,Real bound=Inf);

///Convenience function to compute closest points between two meshes.  v1 and v2 are in their respective mesh's local coordinates
///If bound is given, this will quit and return far away points if the closest points is known to be farther than bound.
void ClosestPoints(const CollisionMesh& m1,const CollisionMesh& m2,Real absErr,Real relErr,Vector3& v1local,Vector3& v2local,Real bound=Inf);

///Returns the point on the mesh that minimizes
///   pWeight||p-x||^2 + nWeight||n-nx||^2.
///The point x is returned in cp, and the triangle index is the return value.
int ClosestPointAndNormal(const Meshing::TriMesh& m,Real pWeight,Real nWeight,const Vector3& p,const Vector3& n,Vector3& cp);

///Same as above, but uses the PQP bounding heirarchy and assumes pWeight = 1.
///
///p and n are assumed given in world coordinates, with the mesh's current transform taken into account.
///
///The closest point is cplocal, which is given in mesh-local coordinates
int ClosestPointAndNormal(const CollisionMesh& mesh,Real nWeight,const Vector3& p,const Vector3& n,Vector3& cplocal);


//should eventually implement these...
//bool WithinDistance(const CollisionMesh& m,const Segment3D& s,Real d);
//bool WithinDistance(const CollisionMesh& m,const AABB3D& bb,Real d);
//bool WithinDistance(const CollisionMesh& m,const Box3D& b,Real d);
//void NearbyTriangles(const CollisionMesh& m,const Segment3D& s,Real d,std::vector<int>& tris);
///Finds the closest point to s on m and returns the triangle
//int ClosestPoints(const CollisionMesh& m,const Segment3D& s,Vector3& vm,Vector3& vs);


} //namespace Geometry

#endif

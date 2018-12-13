#ifndef ROBOT_WITH_GEOMETRY_H
#define ROBOT_WITH_GEOMETRY_H

#include "RobotDynamics3D.h"
#include <KrisLibrary/structs/array2d.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <memory>

/** @ingroup Robot
 * @brief The base class for a robot definition. 
 *
 * All robot geometry is given as triangulated surfaces. 
 * To initialize the robot, one should
 * 1) Initialize the RobotDynamics3D 
 * 2) Load the geometry for each link using LoadGeometry(),
 * 3) Initialize the collision structures (and self collision pairs) using
 *    InitCollisions() and InitSelfCollisionPair().
 */
class RobotWithGeometry : public RobotDynamics3D
{
public:
  typedef Geometry::AnyCollisionGeometry3D CollisionGeometry;
  typedef Geometry::AnyCollisionQuery CollisionQuery;
  
  RobotWithGeometry();
  RobotWithGeometry(const RobotDynamics3D& rhs);
  RobotWithGeometry(const RobotWithGeometry& rhs);
  virtual ~RobotWithGeometry();

  /// These should be used by the initialization routines
  void Initialize(int numLinks);
  bool LoadGeometry(int i,const char* file);
  bool SaveGeometry(int i,const char* file);
  bool IsGeometryEmpty(int i);
  void InitCollisions();
  void CleanupCollisions();
  void InitAllSelfCollisions();
  void InitSelfCollisionPair(int i,int j);
  void InitSelfCollisionPairs(const Array2D<bool>& collision);
  void GetSelfCollisionPairs(Array2D<bool>& collision) const;
  void CleanupSelfCollisions();

  ///Creates this into a mega-robot from several other robots
  void Merge(const std::vector<RobotWithGeometry*>& robots);

  ///Copy, keeping shared references to geometries
  const RobotWithGeometry& operator = (const RobotWithGeometry& rhs);
  ///Copy, creating empty geometries
  const RobotWithGeometry& operator = (const RobotDynamics3D& rhs);

  /// Call this before querying self collisions
  virtual void UpdateGeometry();
  virtual void UpdateGeometry(int i);
  /// Call this before querying environment collisions 
  virtual void InitMeshCollision(CollisionGeometry& mesh);

  virtual bool SelfCollision(Real distance=0);
  /// Query self-collision between links indexed by bodies. Faster than direct enumeration.
  virtual bool SelfCollision(const std::vector<int>& bodies, Real distance=0);
  /// Query self-collision between links in set1 and set 2.  set1 and set2 are required to be disjoint.  Faster than direct enumeration.
  virtual bool SelfCollision(const std::vector<int>& set1,const std::vector<int>& set2,Real distance=0);
  /// Query self-collision between geometries i,j.  Faster than direct enumeration.
  virtual bool SelfCollision(int i, int j, Real distance=0); 
  /// Compute all self collisions (faster than 
  virtual void SelfCollisions(std::vector<std::pair<int,int> >& pairs,Real distance=0);

  virtual bool MeshCollision(CollisionGeometry& mesh);
  virtual bool MeshCollision(int i,Real distance=0);

  virtual void DrawGL();
  virtual void DrawLinkGL(int i);

  std::vector<std::shared_ptr<CollisionGeometry> > geometry;
  ///matrix(i,j) of collisions between bodies, i < j (upper triangular)
  Array2D<CollisionQuery*> selfCollisions;
  std::vector<CollisionQuery*> envCollisions;
};

#endif

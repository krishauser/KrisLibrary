#ifndef ROBOT_WITH_GEOMETRY_H
#define ROBOT_WITH_GEOMETRY_H

#include <robotics/RobotDynamics3D.h>
#include <structs/array2d.h>
#include <geometry/AnyGeometry.h>

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
  virtual ~RobotWithGeometry();

  /// These should be used by the initialization routines
  void Initialize(int numLinks);
  bool LoadGeometry(int i,const char* file);
  bool SaveGeometry(int i,const char* file);
  void InitCollisions();
  void CleanupCollisions();
  void InitAllSelfCollisions();
  void InitSelfCollisionPair(int i,int j);
  void InitSelfCollisionPairs(const Array2D<bool>& collision);
  void GetSelfCollisionPairs(Array2D<bool>& collision) const;
  void CleanupSelfCollisions();

  /// Call this before querying self collisions
  virtual void UpdateGeometry();
  virtual void UpdateGeometry(int i);
  /// Call this before querying environment collisions 
  virtual void InitMeshCollision(CollisionGeometry& mesh);

  virtual bool SelfCollision(Real distance=0);
  /// Query collision between specified bodies
  virtual bool SelfCollision(const std::vector<int>& bodies, Real distance=0);
  /// Query collision between bodies in set1 and set 2
  virtual bool SelfCollision(const std::vector<int>& set1,const std::vector<int>& set2,Real distance=0);
  /// Query collision between bodies i,j
  virtual bool SelfCollision(int i, int j, Real distance=0); 

  virtual bool MeshCollision(CollisionGeometry& mesh);
  virtual bool MeshCollision(int i,Real distance=0);

  virtual void DrawGL();
  virtual void DrawLinkGL(int i);

  std::vector<CollisionGeometry> geometry;
  ///matrix(i,j) of collisions between bodies, i < j (upper triangular)
  Array2D<CollisionQuery*> selfCollisions;
  std::vector<CollisionQuery*> envCollisions;
};

#endif

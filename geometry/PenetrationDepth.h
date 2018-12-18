#ifndef PENETRATION_DEPTH_H
#define PENETRATION_DEPTH_H

#include <KrisLibrary/meshing/TriMeshTopology.h>
#include <KrisLibrary/errors.h>

namespace Geometry {

  using namespace Math3D;

/** @ingroup Geometry
 * @brief Uses a propagation method to calculate an approximate
 * penetration distance of mesh m1 inside m2.
 *
 * The method is similar to Heidelberger 2002.
 * The propagation is initialized using ComputeInitial() with the world-space
 * rigid-body transforms of the two meshes, and lists of overlapping triangles.
 * 
 * Then, the approximate penetration depth is calculated using ComputeDepth().
 * To re-initialize the computation use Reset().
 */
class ApproximatePenetrationDepth
{
public:
  typedef Meshing::TriMeshWithTopology TriMeshWithTopology;
  typedef Meshing::TriMesh TriMesh;

  enum VertexClass { Unvisited=0, Fringe=1, Computed=2, Outside=3 };

  ApproximatePenetrationDepth(const TriMeshWithTopology &m1, const TriMesh &m2);
  void Reset();
  void ComputeInitial(const RigidTransform& f1,const RigidTransform& f2,const int tc1[], const int tc2[],int n);
  void ComputeDepth();

  const TriMeshWithTopology &m1;
  const TriMesh &m2;
  std::vector<VertexClass> vertexClass;
  std::vector<Real> depth;
  std::vector<Vector3> normal;
  std::vector<int> fringe;
  Real maxDepth;
  Vector3 deepestPoint;
  Vector3 deepestNormal;
};

} //namespace Math3D

#endif

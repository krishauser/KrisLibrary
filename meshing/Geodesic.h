#ifndef MESHING_GEODESIC_H
#define MESHING_GEODESIC_H

#include "TriMeshTopology.h"
#include <KrisLibrary/structs/FixedSizeHeap.h>

namespace Meshing {

/** @ingroup Meshing
 * @brief Computes an approximate geodesic distance over a mesh, 
 * from a single source.
 *
 * O(n log n) implementation.
 */
class ApproximateGeodesic
{
 public:
  ApproximateGeodesic(const TriMeshWithTopology& mesh);
  void ComputeVirtualEdges();
  void SolveFromVertex(int v);
  void SolveFromTri(int tri,const Vector3& pt);
  Real Distance(int tri,const Vector3& pt) const;
  void ExpandVert(int v);
  void UpdateDistance(int v,Real d);

  const TriMeshWithTopology& mesh;
  vector<Real> triangleWeights; //by default empty means a coefficient of 1
  vector<Real> vertCosts;
  vector<int> vertColor;
  FixedSizeHeap<Real> h;
  //edges to propagate values to from obtuse triangles to support (along with the unfolded virtual point
  struct VirtualEdge
  {
    int vertex1,vertex2;
    Vector2 planePos1,planePos2;
  };
  vector<VirtualEdge> virtualEdges;
  //co-edges from support vertices to obtuse triangles 
  vector<vector<int> > incomingVirtualEdges; 
};

} //namespace Meshing

#endif

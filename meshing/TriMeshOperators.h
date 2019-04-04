#ifndef MESHING_TRI_MESH_OPERATORS_H
#define MESHING_TRI_MESH_OPERATORS_H

#include "TriMeshTopology.h"
#include <KrisLibrary/utils/IntPair.h>
#include <KrisLibrary/errors.h>
#include <list>

/** @file meshing/TriMeshOperators.h
 * @ingroup Meshing
 * @brief Utilities for operating on, and walking the topology of,
 * triangle meshes.
 */

namespace Meshing {

  /** @addtogroup Meshing */
  /*@{*/

///Returns the angle between two vectors
inline Real Angle(const Vector3& e1,const Vector3& e2)
{
  Real ne1=e1.norm();
  Real ne2=e2.norm();
  if(FuzzyZero(ne2) || FuzzyZero(ne1)) return Zero;
  Real cosa = e1.dot(e2)/(ne1*ne2);
  return Acos(Clamp(cosa,-One,One));
}

///Returns the triangle neighboring t CCW about v
inline int CCWNeighbor(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int eccw=(vindex+1)%3; //the ccw edge about v
  return mesh.triNeighbors[t][eccw];
}

///Returns the triangle neighboring t CW about v
inline int CWNeighbor(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int ecw=(vindex+2)%3; //the cw edge about v
  return mesh.triNeighbors[t][ecw];
}

///Returns the (CCW neighbor,CW neighbor) of t about v
inline IntPair Neighbors(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int eccw=(vindex+1)%3; //the ccw edge about v
  int ecw=(vindex+2)%3; //the cw edge about v
  return IntPair(mesh.triNeighbors[t][eccw],mesh.triNeighbors[t][ecw]);
}

///Returns the vertex adjacent to v on the CCW side of t
inline int CCWAdjacentVertex(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int eccw=(vindex+2)%3; //the ccw edge about v
  return mesh.tris[t][eccw];
}

///Returns the vertex adjacent to v on the CCW side of t
inline int CWAdjacentVertex(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int ecw=(vindex+1)%3; //the cw edge about v
  return mesh.tris[t][ecw];
}

///Returns the vertices on t adjacent to v in (CCW,CW) order
inline IntPair AdjacentVertices(const TriMeshWithTopology& mesh,int t,int v)
{
  int vindex=mesh.tris[t].getIndex(v);
  Assert(vindex>=0);
  int eccw=(vindex+2)%3; //the ccw edge about v
  int ecw=(vindex+1)%3; //the cw edge about v
  return IntPair(mesh.tris[t][eccw],mesh.tris[t][ecw]);
}

///Returns true if the vertex is "floating"
inline bool FloatingVertex(const TriMeshWithTopology& mesh,int v)
{
  Assert(!mesh.incidentTris.empty());
  return mesh.incidentTris[v].empty();
}

///Returns true if the vertex is a boundary vertex
inline bool BoundaryVertex(const TriMeshWithTopology& mesh,int v)
{
  Assert(!mesh.incidentTris.empty());
  Assert(!mesh.triNeighbors.empty());
  for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
    int t=mesh.incidentTris[v][i];
    IntPair n=Neighbors(mesh,t,v);
    if(n.a < 0 || n.b < 0) return true;
  }
  return false;
}

inline Real IncidentTriangleArea(const TriMeshWithTopology& mesh,int v)
{
  Assert(!mesh.incidentTris.empty());
  Real size=0;
  Triangle3D tri;
  for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
    mesh.GetTriangle(mesh.incidentTris[v][i],tri);
    size += tri.area();
  }
  return size;
}

///Divides the triangles connected to v in lists sorted by CCW order.
///Returns true if the vertex is a boundary vertex
bool IncidentTriangleOrdering(const TriMeshWithTopology& mesh,int v,vector<list<int> >& triStrips);

///Returns the integral of the gaussian curvature about v
///assuming the mesh approximates an underlying smooth surface
Real VertexGaussianCurvature(const TriMeshWithTopology& mesh,int v);

///Returns the abs val of the integral of the mean curvature about v
///assuming the mesh approximates an underlying smooth surface
Real VertexAbsMeanCurvature(const TriMeshWithTopology& mesh,int v);

///Merges all vertices in mesh that are closer than tolerance Linf distance from each other
void MergeVertices(TriMesh& mesh,Real tolerance);

///Shifts the mesh's vertices by an amount that causes each adjacent triangle
///to be shifted along its normal by the given amount.  This is a local, approximate
///erosion operator that is much quicker to compute, but the approximation degrades
///with larger amounts of erosion.
///
///If mergeFirst = true, then the points are first merged.  This helps with some
///exploded triangle meshes.  If you know your mesh is not exploded, then you
///can save some overhead by setting this to false.
///
///Returns 0 if there were no local interpenetrations caused by the shrinking,
///otherwise it returns the number of detected local interpenetrations.
int ApproximateShrink(TriMeshWithTopology& mesh,Real amount,bool mergeFirst=true);

  /*@}*/

} //namespace Meshing

#endif

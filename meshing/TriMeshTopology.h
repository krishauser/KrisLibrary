#ifndef MESHING_TRIMESH_TOPOLOGY_H
#define MESHING_TRIMESH_TOPOLOGY_H

#include "TriMesh.h"

namespace Meshing {

/** @ingroup Meshing
 * @brief A callback base class for traversing triangle mesh topology.
 *
 * 2 types of traversal: <br>
 * Triangle walk: nodes are triangles, arcs are edges
 *   Tri() called on new node, TriArc() then Edge() on new edge. <br>
 * Vertex walk: nodes are vertices, arcs are edges
 *   Vertex() called on new node, Edge() on new edge. <br>
 */
struct TriMeshTraversalCallback
{
  virtual ~TriMeshTraversalCallback() {}
  virtual void NewComponent(int c) {}
  virtual void Tri(int t) {}
  virtual void TriArc(int t,int e) {}
  virtual void Edge(int v1,int v2) {}
  virtual void Vertex(int v) {}
};

/** @ingroup Meshing
 * @brief A triangle mesh that contains connectivity relations between
 * vertices and triangles.
 */
struct TriMeshWithTopology : public TriMesh
{
  void ClearTopology();
  void CalcVertexNeighbors();
  void CalcIncidentTris();
  void CalcTriNeighbors();
  bool IsConsistent();

  ///@name Editing
  //@{
  void SplitEdge(int tri,int e,const Vector3& newPt);
  //@}

  ///@name Tri/vertex traversal
  //@{
  void TriBFS(TriMeshTraversalCallback& callback);
  void VertexBFS(TriMeshTraversalCallback& callback);
  void BeginTriWalk();
  void BeginVertexWalk();
  void _TriBFS(int start,TriMeshTraversalCallback& callback);
  void _VertexBFS(int start,TriMeshTraversalCallback& callback);
  //@}

  ///index 0,1,2=neighbor along edge 0,1,2 (or -1 if none)
  typedef IntTriple TriNeighbors;

  vector<vector<int> > vertexNeighbors; ///<neighboring vertices of vertices
  vector<vector<int> > incidentTris;  ///<triangles incident on vertices
  vector<TriNeighbors> triNeighbors;  ///<neighboring triangles of triangles
  vector<int> visited;   ///<temporary
};

} //namespace Meshing

#endif

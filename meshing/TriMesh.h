#ifndef MESHING_TRIMESH_H
#define MESHING_TRIMESH_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/Triangle3D.h>
#include <KrisLibrary/math3d/Ray3D.h>
#include <KrisLibrary/utils/IntTriple.h>
#include <vector>
#include <iosfwd>

/** @defgroup Meshing
 * @brief Classes defining operations on basic triangle meshes
 */

/** @ingroup Meshing
 * @brief The namespace for all classes/functions in the Meshing package.
 */
namespace Meshing {

using namespace Math3D;
using namespace std;

/** @ingroup Meshing
 * @brief A basic triangle mesh.
 *
 * The mesh is represented by a list of vertices (Vector3's) and a list of
 * indexed triangles.  A triangle is represented by a set of 3 integers
 * (an IntTriple) that index into the vertex list to produce the actual
 * triangle vertices.  So a triangle whose vertices are verts[1], verts[2],
 * and verts[3] will be represented by the IntTriple [1,2,3].
 *
 * There are several "types" of indices floating around:
 *
 * Vertex index - an index into the verts array. <br>
 * Triangle index - an index into the tris array. <br>
 * i'th vertex of a triangle t, i={0,1,2} - t[i] is a vertex index. <br>
 * j'th edge of a triangle t, j={0,1,2} - the edge between vertices
 *   indexed by t[j+1] and t[j+2] (indices modulo 3), that is, the edge
 *   opposite the j'th vertex of t.
 */
struct TriMesh
{
  typedef IntTriple Tri;

  ///@name Accessors
  //@{
  ///Returns the v'th vertex of triangle tri
  Vector3& TriangleVertex(int tri,int v);
  const Vector3& TriangleVertex(int tri,int v) const;
  ///Calculates the normal of triangle tri
  Vector3 TriangleNormal(int tri) const;
  ///Returns a Triangle3D structure for triangle tri
  void GetTriangle(int tri,Triangle3D& t) const;
  ///Calculates a list of triangle indices incident to vertex v. O(T) time.
  void GetIncidentTris(int v,vector<int>& tris) const;
  ///Same as above, but appends the triangles to the end of t
  void AppendIncidentTris(int v,vector<int>& t) const; 
  ///Returns the vertex indices (v1,v2) of the e'th edge of triangle tri
  void GetEdge(int tri,int e,int& v1,int& v2) const;
  ///Returns the index of the triangle adjacent to tri, at the e'th edge 
  ///or -1 if there is none.  O(T) time.
  int GetAdjacentTri(int tri,int e) const;
  //@}

  ///@name Calculations
  //@{
  bool IsValid() const;
  void GetAABB(Vector3& bmin, Vector3& bmax) const;
  //returns the closest/collided triangle
  int ClosestPoint(const Vector3& pt,Vector3& cp) const;
  int RayCast(const Ray3D& r,Vector3& pt) const;
  bool Intersects(const Plane3D&) const;
  bool PlaneSplits(const Plane3D&,Real& dmin,Real& dmax) const;
  //@}

  ///@name Modifiers
  //@{
  void Transform(const Matrix4& mat);
  void FlipFaces();
  void Merge(const vector<TriMesh>& meshes);
  void MergeWith(const TriMesh& mesh);
  void RemoveUnusedVerts();
  //@}

  bool Load(const char* fn);
  bool Save(const char* fn) const;

  vector<Vector3> verts;
  vector<Tri> tris;
};

istream& operator >> (istream& in,TriMesh& tri);
ostream& operator << (ostream& out,const TriMesh& tri);
bool LoadMultipleTriMeshes(const char* fn,TriMesh& tri);

} //namespace Meshing

#endif

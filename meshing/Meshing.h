#ifndef MESHING_UTILS_H
#define MESHING_UTILS_H

#include "TriMeshTopology.h"
#include <KrisLibrary/math3d/Triangle2D.h>

namespace Meshing
{
using namespace Math3D;
using namespace std;

/** @ingroup Meshing
 * @brief A class that allows incremental plane-splitting of a triangle mesh.
 *
 * positive[i] is set to true if tri[i] is on the positive side of the planes
 * (up to the tolerance tol).  If deleteNegative is true, the triangles on
 * the negative side of any plane are deleted.
 *
 * origTri stores an index for each triangle of the resulting mesh, indexing
 * the triangle in the mesh originally provided in the constructor.
 */
struct TriSplitter
{
  TriSplitter(TriMeshWithTopology& mesh);
  void Split(const Plane3D& p);

  TriMeshWithTopology& mesh;
  vector<bool> positive;
  vector<int> origTri;
  Real tol;
  bool deleteNegative;

  //temp
  //list of distances
  vector<Real> d;
  //for each triangle, list of (edge#,vert) for newly added vertex
  vector<vector<pair<int,int> > > newVerts;
};

/** @file meshing/Meshing.h
 * @ingroup Meshing
 * @brief Utilities for manipulating triangle meshes
 */

/** @addtogroup Meshing */
/*@{*/

/** @brief Splits the triangle t by the plane p.  Returns the
 * number of resulting triangles (up to 3).
 *
 * The new triangles are given as a set of indices.  0,1,2 are the vertices
 * a,b,c of the original triangle.  3,4 are newPts[0],newPts[1].
 *
 * For each new triangle, triPositive marks true if it is on the positive
 * side of the plane.
 */
int SplitTriangle(const Triangle3D& t,const Plane3D& p,Vector3 newPts[2],IntTriple newTris[3],bool triPositive[3],Real tol);
int SplitTriangle(const Triangle2D& t,const Plane2D& p,Vector2 newPts[2],IntTriple newTris[3],bool triPositive[3],Real tol);
void GetCoplanarTris(const TriMesh& mesh,int t,Real tol,vector<int>& tris);
void GetConnectedCoplanarTris(const TriMeshWithTopology& mesh,int t,Real tol,vector<int>& tris);
//void GetConnectedCoplanarTris(const TriMeshWithTopology& mesh,int t,Real tol,PolygonWithHoles2D& poly);

/*@}*/

} //namespace Meshing;

#endif

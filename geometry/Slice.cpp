#include "Slice.h"
#include <KrisLibrary/Logger.h>

namespace Geometry { 

void Slice(const Meshing::PointCloud3D& pc,const Plane3D& p,Real tol,vector<Vector3>& points,vector<int>& indices)
{
  points.resize(0);
  for(size_t i=0;i<pc.points.size();i++)
    if(Abs(p.distance(pc.points[i])) <= tol) {
      points.push_back(pc.points[i]);
      indices.push_back((int)i);
    }
}

void SliceXY(const Meshing::PointCloud3D& pc,const RigidTransform& T,Real tol,vector<Vector2>& points,vector<int>& indices)
{
  Vector3 tp;
  Vector3 x,y,z;
  T.R.getCol1(x);
  T.R.getCol2(y);
  T.R.getCol3(z);
  points.resize(0);
  Vector3 plocal;
  for(size_t i=0;i<pc.points.size();i++) {
    plocal = pc.points[i] - T.t;
    if(Abs(z.dot(plocal)) <= tol) {
      points.push_back(Vector2(x.dot(plocal),y.dot(plocal)));
      indices.push_back((int)i);
    }
  }
}

///Returns the points in pc within tolerance tol of the plane p.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void Slice(const CollisionPointCloud& pc,const Plane3D& p,Real tol,vector<Vector3>& points,vector<int>& indices)
{
  Plane3D plocal;
  RigidTransform Tinv;
  pc.currentTransform.getInverse(Tinv);
  plocal.setTransformed(p,Tinv);
  AABB3D bblocal_expanded = pc.bblocal;
  if(tol > 0) {
    bblocal_expanded.bmin -= Vector3(tol);
    bblocal_expanded.bmax += Vector3(tol);
  }
  if(!plocal.intersects(bblocal_expanded)) {
    points.resize(0);
    indices.resize(0);
    return;
  }
  if(pc.maxRadius > 0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"TODO: Slice(CollisionPointCloud) with nonzero point radius");
  }
  const Meshing::PointCloud3D& pcdata=pc;
  Slice(pcdata,plocal,tol,points,indices);

  //retrieve world coordinates of points
  RigidTransform ident; ident.setIdentity();
  if(pc.currentTransform != ident) {
    for(size_t i=0;i<points.size();i++)
      points[i] = pc.currentTransform*points[i];
  }
}


///Same as slice, except slices the transformed point cloud T*pc about the
///x-y plane, and returns the points as x-y coordinates
void SliceXY(const CollisionPointCloud& pc,const RigidTransform& T,Real tol,vector<Vector2>& points,vector<int>& indices)
{
  RigidTransform Tlocal;
  Tlocal.mulInverseA(pc.currentTransform,T);
  Plane3D plocal;
  plocal.normal = Tlocal.R*Vector3(0,0,1);
  plocal.offset = plocal.normal.dot(Tlocal.t);
  AABB3D bblocal_expanded = pc.bblocal;
  if(tol > 0) {
    bblocal_expanded.bmin -= Vector3(tol);
    bblocal_expanded.bmax += Vector3(tol);
  }
  if(!plocal.intersects(bblocal_expanded)) {
    points.resize(0);
    return;
  }
  if(pc.maxRadius > 0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"TODO: SliceXY(CollisionPointCloud) with nonzero point radius");
  }
  const Meshing::PointCloud3D& pcdata=pc;
  SliceXY(pcdata,Tlocal,tol,points,indices);
}



///Returns the segments obtained by taking a slice of a mesh at plane p.
///O(T) time, where T is the number of triangles.
void Slice(const Meshing::TriMesh& m,const Plane3D& p,vector<Segment3D>& segments,vector<int>& indices)
{
  segments.resize(0);
  indices.resize(0);
  Triangle3D t;
  Segment3D s;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle(i,t);
    if(t.intersects(p,s)) {
      segments.push_back(s);
      indices.push_back((int)i);
    }
  } 
}

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const Meshing::TriMesh& m,const RigidTransform& T,vector<Segment2D>& segments,vector<int>& indices)
{
  segments.resize(0);
  indices.resize(0);
  Vector3 x,y;
  Plane3D p;
  T.R.getCol1(x);
  T.R.getCol2(y);
  T.R.getCol3(p.normal);
  p.offset = p.normal.dot(T.t);
  Triangle3D t;
  Segment3D s;
  Segment2D s2;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle(i,t);
    if(t.intersects(p,s)) {
      Vector3 a,b;
      a = s.a - T.t;
      b = s.b - T.t;
      s2.a.x = x.dot(a);
      s2.a.y = y.dot(a);
      s2.b.x = x.dot(b);
      s2.b.y = y.dot(b);
      segments.push_back(s2);
      indices.push_back((int)i);
    }
  } 
}

///Returns the segments obtained by taking a slice of a mesh at plane p.
///Possibly faster than the naive Slice method due to the use of a bounding
///volume hierarchy.
void Slice(const CollisionMesh& m,const Plane3D& p,vector<Segment3D>& segments,vector<int>& indices)
{
  CollideAll(m,p,indices);
  int numMissing = 0;
  Triangle3D t;
  Segment3D s;
  for(size_t i=0;i<indices.size();i++) {
    m.GetTriangle(indices[i],t);
    if(t.intersects(p,s)) {
      segments.push_back(s);
    }
    else {
      //weird, 
      indices[i] = indices.back();
      indices.resize(indices.size()-1);
      i--;
      numMissing ++;
    }
  }
  if(numMissing > 2)
    LOG4CXX_WARN(KrisLibrary::logger(),"Slice: collision mesh doesn't correctly collide triangles and plane");
}

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const CollisionMesh& m,const RigidTransform& T,vector<Segment2D>& segments,vector<int>& indices)
{
  Vector3 x,y;
  Plane3D p;
  T.R.getCol1(x);
  T.R.getCol2(y);
  T.R.getCol3(p.normal);
  p.offset = p.normal.dot(T.t);
  Plane3D plocal;
  RigidTransform Tm_inv; Tm_inv.setInverse(m.currentTransform);
  plocal.setTransformed(p,Tm_inv);
  CollideAll(m,p,indices);
  int numMissing = 0;
  Triangle3D t;
  Segment3D s;
  Segment2D s2;
  for(size_t i=0;i<indices.size();i++) {
    m.GetTriangle(indices[i],t);
    if(t.intersects(plocal,s)) {
      Vector3 a,b;
      a = m.currentTransform*s.a - T.t;
      b = m.currentTransform*s.b - T.t;
      s2.a.x = x.dot(a);
      s2.a.y = y.dot(a);
      s2.b.x = x.dot(b);
      s2.b.y = y.dot(b);
      segments.push_back(s2);
    }
    else {
      //weird, 
      indices[i] = indices.back();
      indices.resize(indices.size()-1);
      i--;
      numMissing ++;
    }
  }
  if(numMissing > 2)
    LOG4CXX_WARN(KrisLibrary::logger(),"SliceXY: collision mesh doesn't correctly collide triangles and plane");
}

} //namespace Geometry

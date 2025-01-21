#ifndef GEOMETRY_ROI_H
#define GEOMETRY_ROI_H

#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/CollisionPointCloud.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/Box3D.h>
#include <KrisLibrary/math3d/Sphere3D.h>

namespace Geometry {

#ifndef EXTRACT_ROI_FLAG_DECL
#define EXTRACT_ROI_FLAG_DECL
enum {
    ExtractROIFlagIntersection=0x01,
    ExtractROIFlagTouching=0x02,
    ExtractROIFlagWithin=0x04,
    ExtractROIFlagInvert=0x08
};
#endif //EXTRACT_ROI_FLAG_DECL

///Returns the points in pc within bb.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const AABB3D& bb,Meshing::PointCloud3D& pc_roi,int flag=ExtractROIFlagIntersection);

///Returns the points in pc within bb.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const Box3D& bb,Meshing::PointCloud3D& pc_roi,int flag=ExtractROIFlagIntersection);

///Returns the points in pc within s.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const Sphere3D& s,Meshing::PointCloud3D& pc_roi,int flag=ExtractROIFlagIntersection);

///Returns the points in pc within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
///
///Respects the point cloud's current transform.
void ExtractROI(const CollisionPointCloud& pc,const AABB3D& bb,CollisionPointCloud& pc_roi,int flag=ExtractROIFlagIntersection);

///Returns the points in pc within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
///
///Respects the point cloud's current transform.
void ExtractROI(const CollisionPointCloud& pc,const Box3D& bb,CollisionPointCloud& pc_roi,int flag=ExtractROIFlagIntersection);

///Returns the points in pc within s.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
///
///Respects the point cloud's current transform.
void ExtractROI(const CollisionPointCloud& pc,const Sphere3D& s,CollisionPointCloud& pc_roi,int flag=ExtractROIFlagIntersection);


///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const Meshing::TriMesh& m,const AABB3D& bb,Meshing::TriMesh& m_roi,int flag=ExtractROIFlagIntersection);

///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const Meshing::TriMesh& m,const Box3D& bb,Meshing::TriMesh& m_roi,int flag=ExtractROIFlagIntersection);

///Returns the triangles of a mesh intersected/touching/within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy. (only applied in non-inverted mode)
///
///Respects the mesh's current transform.
void ExtractROI(const CollisionMesh& m,const AABB3D& bb,CollisionMesh& m_roi,int flag=ExtractROIFlagIntersection);

///Returns the triangles of a mesh intersected/touching/within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy. (only applied in non-inverted mode)
///
///Respects the mesh's current transform.
void ExtractROI(const CollisionMesh& m,const Box3D& bb,CollisionMesh& m_roi,int flag=ExtractROIFlagIntersection);

} // namespace Geometry

#endif

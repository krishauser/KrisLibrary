#ifndef CAMERA_CLIP_H
#define CAMERA_CLIP_H

// INCLUSION: The object is completely inside the volume.
// (completely visible)
#define INCLUSION -1
// INTERSECT: The object intersects with one of more of the
// volume's planes. (partially visible)
#define INTERSECT 0
// EXCLUSION: The object is completely outside the volume.
// (invisible)
#define EXCLUSION 1

#include <KrisLibrary/math3d/geometry3d.h>
#include <vector>
using namespace Math3D;

//NOTE: the planes are defined so that normals point outward
class ConvexVolume
{
public:
	int PointOverlap(const Vector3& pt) const;
	int AABBOverlap(const Vector3& bMin, const Vector3& bMax) const;
	int LineOverlap(const Line3D& pt, Real& tmin, Real& tmax) const;

	void Transform(const Matrix4& m);
	void SetTransform(const ConvexVolume&,const Matrix4& m);

	std::vector<Plane3D> planes;
};

#endif

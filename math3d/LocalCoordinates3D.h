#ifndef MATH3D_LOCAL_COORDINATES3D_H
#define MATH3D_LOCAL_COORDINATES3D_H

#include "Point.h"

namespace Math3D {

struct Line3D;
struct Segment3D;
struct Plane3D;

//a rigid-body transformation
struct LocalCoordinates3D
{
	void getBasis (Matrix4&) const;
	void getBasisInv (Matrix4&) const;
	void getTransform (RigidTransform&) const;
	void getTransformInv (RigidTransform&) const;
	void toLocal (const Point3D&, Point3D&) const;
	void toLocalReorient (const Vector3&, Vector3&) const;
	void fromLocal (const Point3D&, Point3D&) const;
	void fromLocalReorient(const Vector3&, Vector3&) const;

	void toLocal (const Line3D&, Line3D&) const;
	void toLocal (const Segment3D&, Segment3D&) const;
	void toLocal (const Plane3D&, Plane3D&) const;
	void fromLocal (const Line3D&, Line3D&) const;
	void fromLocal (const Segment3D&, Segment3D&) const;
	void fromLocal (const Plane3D&, Plane3D&) const;

	bool Read(File& f);
	bool Write(File& f) const;

	Vector3 origin;
	Vector3 xbasis, ybasis, zbasis;	//orthonormal vectors representing the orientation
};

//a rigid-body transformation with a scale in each basis direction
//normalize -> from unscaled coordinates to scaled
//denormalize -> from scaled coordinates to unscaled
struct ScaledLocalCoordinates3D : public LocalCoordinates3D
{
	void getBasisScaled (Matrix4&) const;
	void getBasisScaledInv (Matrix4&) const;
	void normalize (const Vector3&, Vector3&) const;
	void denormalize (const Vector3&, Vector3&) const;

	void toLocalNormalized (const Point3D&, Point3D&) const;
	void toLocalNormalized (const Line3D&, Line3D&) const;
	void toLocalNormalized (const Segment3D&, Segment3D&) const;
	void toLocalNormalized (const Plane3D&, Plane3D&) const;
	void fromLocalNormalized (const Point3D&, Point3D&) const;
	void fromLocalNormalized (const Line3D&, Line3D&) const;
	void fromLocalNormalized (const Segment3D&, Segment3D&) const;
	void fromLocalNormalized (const Plane3D&, Plane3D&) const;

	bool Read(File& f);
	bool Write(File& f) const;

	Vector3 dims;
};

} //namespace Math3D

#endif

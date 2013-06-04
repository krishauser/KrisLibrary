#ifndef MATH3D_LOCAL_COORDINATES2D_H
#define MATH3D_LOCAL_COORDINATES2D_H

#include "Point.h"

namespace Math3D {

struct Line2D;
struct Segment2D;
struct Plane2D;

//a rigid-body transformation
struct LocalCoordinates2D
{
	void toLocal (const Point2D&, Point2D&) const;
	void toLocalReorient (const Vector2&, Vector2&) const;
	void fromLocal (const Point2D&, Point2D&) const;
	void fromLocalReorient(const Vector2&, Vector2&) const;

	void toLocal (const Line2D&, Line2D&) const;
	void toLocal (const Segment2D&, Segment2D&) const;
	void toLocal (const Plane2D&, Plane2D&) const;
	void fromLocal (const Line2D&, Line2D&) const;
	void fromLocal (const Segment2D&, Segment2D&) const;
	void fromLocal (const Plane2D&, Plane2D&) const;

	bool Read(File& f);
	bool Write(File& f) const;

	Vector2 origin;
	Vector2 xbasis, ybasis;	//orthonormal vectors representing the orientation
};

//a rigid-body transformation with a scale in each basis direction
//normalize -> from unscaled coordinates to scaled
//denormalize -> from scaled coordinates to unscaled
struct ScaledLocalCoordinates2D : public LocalCoordinates2D
{
	void normalize (const Vector2&, Vector2&) const;
	void denormalize (const Vector2&, Vector2&) const;

	void toLocalNormalized (const Point2D&, Point2D&) const;
	void toLocalNormalized (const Line2D&, Line2D&) const;
	void toLocalNormalized (const Segment2D&, Segment2D&) const;
	void toLocalNormalized (const Plane2D&, Plane2D&) const;
	void fromLocalNormalized (const Point2D&, Point2D&) const;
	void fromLocalNormalized (const Line2D&, Line2D&) const;
	void fromLocalNormalized (const Segment2D&, Segment2D&) const;
	void fromLocalNormalized (const Plane2D&, Plane2D&) const;

	bool Read(File& f);
	bool Write(File& f) const;

	Vector2 dims;
};

} //namespace Math3D

#endif

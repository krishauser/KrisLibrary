#include "clip.h"
#include <math3d/clip.h>

int ConvexVolume::PointOverlap(const Vector3& pt) const
{
	for(size_t i=0; i<planes.size(); i++)
		if(planes[i].distance(pt) > 0) return EXCLUSION;
	return INCLUSION;
}

int ConvexVolume::AABBOverlap(const Vector3& bMin, const Vector3& bMax) const
{
	Vector3 neg, pos;
	// Assume no intersection
	bool isect=false;
	// For each plane
	for (size_t i=0; i<planes.size(); i++)
	{
		// Find the negative and positive extreme points
		if (planes[i].normal.x > 0.0f)
		{

			if (planes[i].normal.y > 0.0f)
			{
				if (planes[i].normal.z > 0.0f)
				{
					pos.x = bMax.x; pos.y = bMax.y; pos.z = bMax.z;
					neg.x = bMin.x; neg.y = bMin.y; neg.z = bMin.z;
				}
				else
				{
					pos.x = bMax.x; pos.y = bMax.y; pos.z = bMin.z;
					neg.x = bMin.x; neg.y = bMin.y; neg.z = bMax.z;
				}
			}
			else
			{
				if (planes[i].normal.z > 0.0f)
				{
					pos.x = bMax.x; pos.y = bMin.y; pos.z = bMax.z;
					neg.x = bMin.x; neg.y = bMax.y; neg.z = bMin.z;
				}
				else
				{
					pos.x = bMax.x; pos.y = bMin.y; pos.z = bMin.z;
					neg.x = bMin.x; neg.y = bMax.y; neg.z = bMax.z;
				}
			}
		}
		else
		{
			if (planes[i].normal.y > 0.0f)
			{
				if (planes[i].normal.z > 0.0f)
				{
					pos.x = bMin.x; pos.y = bMax.y; pos.z = bMax.z;
					neg.x = bMax.x; neg.y = bMin.y; neg.z = bMin.z;
				}
				else
				{
					pos.x = bMin.x; pos.y = bMax.y; pos.z = bMin.z;
					neg.x = bMax.x; neg.y = bMin.y; neg.z = bMax.z;
				}
			}
			else
			{
				if (planes[i].normal.z > 0.0f)
				{
					pos.x = bMin.x; pos.y = bMin.y; pos.z = bMax.z;
					neg.x = bMax.x; neg.y = bMax.y; neg.z = bMin.z;
				}
				else
				{
					pos.x = bMin.x; pos.y = bMin.y; pos.z = bMin.z;
					neg.x = bMax.x; neg.y = bMax.y; neg.z = bMax.z;
				}
			}
		}

		// Case (1): negative extreme point is outside, and thus
		// the AABB is outside the polyhedron
		if (planes[i].distance(neg) > 0) return EXCLUSION;
		// Case (3): negative extreme point is inside, because
		// case (1) is not true. Thus, if the positive
		// extreme point is outside, then the AABB intersects
		// the plane.
		if (planes[i].distance(pos) >= 0) isect = true;

		// else Case (2): Both extreme points are inside
	}

	// The AABB is visible. If there was an intersection, then return
	// INTERSECT. If the was no intersection, then the AABB is completely
	// inside the polyhedron (INCLUSION).
	if (isect) return INTERSECT;
	return INCLUSION;
}

int ConvexVolume::LineOverlap(const Line3D& l, Real& tmin, Real& tmax) const
{
	Real oldTmin=tmin, oldTmax=tmax;
	for(size_t i=0; i<planes.size(); i++)
	  if(!ClipLine(l.source,l.direction,planes[i],tmin,tmax)) return EXCLUSION;
	if(tmin == oldTmin && tmax == oldTmax) return INCLUSION;
	return INTERSECT;
}

void ConvexVolume::Transform(const Matrix4& m)
{
	for(size_t i=0;i<planes.size();i++) {
		Plane3D p=planes[i];
		planes[i].setTransformed(p,m);
	}
}

void ConvexVolume::SetTransform(const ConvexVolume& v,const Matrix4& m)
{
	planes.resize(v.planes.size());
	for(size_t i=0;i<planes.size();i++) {
		planes[i].setTransformed(v.planes[i],m);
	}
}

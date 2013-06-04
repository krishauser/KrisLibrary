#include "LocalCoordinates2D.h"
#include "Segment2D.h"
#include "Line2D.h"
#include "Plane2D.h"
using namespace Math3D;
using namespace std;



void LocalCoordinates2D::toLocalReorient(const Vector2& vin, Vector2& vout) const
{
	vout.x=dot(vin,xbasis);
	vout.y=dot(vin,ybasis);
}

void LocalCoordinates2D::toLocal(const Vector2& vin, Vector2& vout) const
{
	toLocalReorient(vin - origin, vout);	
}

void LocalCoordinates2D::fromLocalReorient(const Vector2& vin, Vector2& vout) const
{
	vout = vin.x*xbasis + vin.y*ybasis;
}

void LocalCoordinates2D::fromLocal(const Vector2& vin, Vector2& vout) const
{
	fromLocalReorient(vin, vout);	
	vout = vout+origin;
}

void LocalCoordinates2D::toLocal(const Line2D& l, Line2D& out) const
{
	toLocalReorient(l.direction, out.direction);
	toLocal(l.source, out.source);
}

void LocalCoordinates2D::fromLocal(const Line2D& l, Line2D& out) const
{
	fromLocalReorient(l.direction, out.direction);
	fromLocal(l.source, out.source);
}

void LocalCoordinates2D::toLocal(const Segment2D& l, Segment2D& out) const
{
	toLocal(l.a, out.a);
	toLocal(l.b, out.b);
}

void LocalCoordinates2D::fromLocal(const Segment2D& l, Segment2D& out) const
{
	fromLocal(l.a, out.a);
	fromLocal(l.b, out.b);
}

void LocalCoordinates2D::toLocal(const Plane2D& p, Plane2D& out) const
{
	toLocalReorient(p.normal, out.normal);
	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	toLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}

void LocalCoordinates2D::fromLocal(const Plane2D& p, Plane2D& out) const
{
	fromLocalReorient(p.normal, out.normal);

	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	fromLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}


void ScaleXBasis(Matrix3& xform, Real scale)
{
	xform(0,0) *= scale;
	xform(1,0) *= scale;
}

void ScaleYBasis(Matrix3& xform, Real scale)
{
	xform(0,1) *= scale;
	xform(1,1) *= scale;
}


void ScaledLocalCoordinates2D::normalize(const Vector2& vin, Vector2& vout) const
{
	vout.x=vin.x/dims.x;
	vout.y=vin.y/dims.y;
}

void ScaledLocalCoordinates2D::denormalize(const Vector2& vin, Vector2& vout) const
{
	vout.x=vin.x*dims.x;
	vout.y=vin.y*dims.y;
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Point2D& p, Point2D& out) const
{
	toLocal(p, out);
	normalize(out, out);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Point2D& p, Point2D& out) const
{
	denormalize(out, out);
	fromLocal(p, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Line2D& l, Line2D& out) const
{
	toLocal(l, out);
	normalize(out.direction, out.direction);
	normalize(out.source, out.source);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Line2D& l, Line2D& out) const
{
	Line2D temp;
	denormalize(l.direction, temp.direction);
	denormalize(l.source, temp.source);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Segment2D& l, Segment2D& out) const
{
	toLocal(l, out);
	normalize(out.a, out.a);
	normalize(out.b, out.b);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Segment2D& l, Segment2D& out) const
{
	Segment2D temp;
	denormalize(l.a, temp.a);
	denormalize(l.b, temp.b);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Plane2D& p, Plane2D& out) const
{
	toLocalReorient(p.normal, out.normal);
	denormalize(out.normal, out.normal);
	out.normal.inplaceNormalize();

	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	toLocal(v, v_out);
	normalize(v_out, v_out);
	out.offset = dot(v_out, out.normal);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Plane2D& p, Plane2D& out) const
{
	Plane2D p_denorm;
	normalize(p.normal, p_denorm.normal);
	p_denorm.normal.inplaceNormalize();

	Vector2 v = p.normal * p.offset;
	Vector2 v_out;
	denormalize(v, v_out);

	p_denorm.offset = dot(v_out, p_denorm.normal);

	fromLocal(p_denorm, out);
}

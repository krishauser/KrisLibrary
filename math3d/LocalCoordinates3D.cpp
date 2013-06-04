#include "LocalCoordinates3D.h"
#include "Segment3D.h"
#include "Line3D.h"
#include "Plane3D.h"
using namespace Math3D;
using namespace std;


void LocalCoordinates3D::getBasis(Matrix4& basis) const
{
	basis.set(xbasis,ybasis,zbasis,origin);
}

void LocalCoordinates3D::getBasisInv(Matrix4& basis) const
{
	//transpose basis
	basis.setIdentity();
	basis.setRow1(xbasis);
	basis.setRow2(ybasis);
	basis.setRow3(zbasis);
	Vector3 v;
	basis.mulVector(origin, v);
	basis.setCol4(-v);
}

void LocalCoordinates3D::toLocalReorient(const Vector3& vin, Vector3& vout) const
{
	vout.x=dot(vin,xbasis);
	vout.y=dot(vin,ybasis);
	vout.z=dot(vin,zbasis);
}

void LocalCoordinates3D::toLocal(const Vector3& vin, Vector3& vout) const
{
	toLocalReorient(vin - origin, vout);	
}

void LocalCoordinates3D::fromLocalReorient(const Vector3& vin, Vector3& vout) const
{
	vout = vin.x*xbasis + vin.y*ybasis + vin.z*zbasis;
}

void LocalCoordinates3D::fromLocal(const Vector3& vin, Vector3& vout) const
{
	fromLocalReorient(vin, vout);	
	vout = vout+origin;
}

void LocalCoordinates3D::toLocal(const Line3D& l, Line3D& out) const
{
	toLocalReorient(l.direction, out.direction);
	toLocal(l.source, out.source);
}

void LocalCoordinates3D::fromLocal(const Line3D& l, Line3D& out) const
{
	fromLocalReorient(l.direction, out.direction);
	fromLocal(l.source, out.source);
}

void LocalCoordinates3D::toLocal(const Segment3D& l, Segment3D& out) const
{
	toLocal(l.a, out.a);
	toLocal(l.b, out.b);
}

void LocalCoordinates3D::fromLocal(const Segment3D& l, Segment3D& out) const
{
	fromLocal(l.a, out.a);
	fromLocal(l.b, out.b);
}

void LocalCoordinates3D::toLocal(const Plane3D& p, Plane3D& out) const
{
	toLocalReorient(p.normal, out.normal);
	Vector3 v = p.normal*p.offset;
	Vector3 v_out;
	toLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}

void LocalCoordinates3D::fromLocal(const Plane3D& p, Plane3D& out) const
{
	fromLocalReorient(p.normal, out.normal);

	Vector3 v = p.normal*p.offset;
	Vector3 v_out;
	fromLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}


void ScaleXBasis(Matrix4& xform, Real scale)
{
	xform(0,0) *= scale;
	xform(1,0) *= scale;
	xform(2,0) *= scale;
}

void ScaleYBasis(Matrix4& xform, Real scale)
{
	xform(0,1) *= scale;
	xform(1,1) *= scale;
	xform(2,1) *= scale;
}

void ScaleZBasis(Matrix4& xform, Real scale)
{
	xform(0,2) *= scale;
	xform(1,2) *= scale;
	xform(2,2) *= scale;
}

void ScaledLocalCoordinates3D::getBasisScaled(Matrix4& basis) const
{
	basis.set(xbasis*dims.x, ybasis*dims.y, zbasis*dims.z, origin);
}

void ScaledLocalCoordinates3D::getBasisScaledInv(Matrix4& basis) const
{
	//transpose basis
	basis.setIdentity();
	basis.setRow1(xbasis);
	basis.setRow2(ybasis);
	basis.setRow3(zbasis);

	Vector3 v;
	basis.mulVector(origin, v);
	basis.setCol4(-v);

	ScaleXBasis(basis,Inv(dims.x));
	ScaleYBasis(basis,Inv(dims.y));
	ScaleZBasis(basis,Inv(dims.z));
}

void ScaledLocalCoordinates3D::normalize(const Vector3& vin, Vector3& vout) const
{
	vout.x=vin.x/dims.x;
	vout.y=vin.y/dims.y;
	vout.z=vin.z/dims.z;
}

void ScaledLocalCoordinates3D::denormalize(const Vector3& vin, Vector3& vout) const
{
	vout.x=vin.x*dims.x;
	vout.y=vin.y*dims.y;
	vout.z=vin.z*dims.z;
}

void ScaledLocalCoordinates3D::toLocalNormalized(const Point3D& p, Point3D& out) const
{
	toLocal(p, out);
	normalize(out, out);
}

void ScaledLocalCoordinates3D::fromLocalNormalized(const Point3D& p, Point3D& out) const
{
	denormalize(out, out);
	fromLocal(p, out);
}

void ScaledLocalCoordinates3D::toLocalNormalized(const Line3D& l, Line3D& out) const
{
	toLocal(l, out);
	normalize(out.direction, out.direction);
	normalize(out.source, out.source);
}

void ScaledLocalCoordinates3D::fromLocalNormalized(const Line3D& l, Line3D& out) const
{
	Line3D temp;
	denormalize(l.direction, temp.direction);
	denormalize(l.source, temp.source);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates3D::toLocalNormalized(const Segment3D& l, Segment3D& out) const
{
	toLocal(l, out);
	normalize(out.a, out.a);
	normalize(out.b, out.b);
}

void ScaledLocalCoordinates3D::fromLocalNormalized(const Segment3D& l, Segment3D& out) const
{
	Segment3D temp;
	denormalize(l.a, temp.a);
	denormalize(l.b, temp.b);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates3D::toLocalNormalized(const Plane3D& p, Plane3D& out) const
{
	toLocalReorient(p.normal, out.normal);
	denormalize(out.normal, out.normal);
	out.normal.inplaceNormalize();

	Vector3 v = p.normal*p.offset;
	Vector3 v_out;
	toLocal(v, v_out);
	normalize(v_out, v_out);
	out.offset = dot(v_out, out.normal);
}

void ScaledLocalCoordinates3D::fromLocalNormalized(const Plane3D& p, Plane3D& out) const
{
	Plane3D p_denorm;
	normalize(p.normal, p_denorm.normal);
	p_denorm.normal.inplaceNormalize();

	Vector3 v = p.normal * p.offset;
	Vector3 v_out;
	denormalize(v, v_out);

	p_denorm.offset = dot(v_out, p_denorm.normal);

	fromLocal(p_denorm, out);
}

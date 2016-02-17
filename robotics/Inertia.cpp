#include "Inertia.h"
#include <math3d/basis.h>

Vector3 CenterOfMass(const GeometricPrimitive3D& geom)
{
  switch(geom.type) {
  case GeometricPrimitive3D::Empty:
    return Vector3(0.0);
  case GeometricPrimitive3D::Point:
    return *AnyCast_Raw<Vector3>(&geom.data);
  case GeometricPrimitive3D::Segment:
    return 0.5*(AnyCast_Raw<Segment3D>(&geom.data)->a+AnyCast_Raw<Segment3D>(&geom.data)->b);
  case GeometricPrimitive3D::Sphere:
    return AnyCast_Raw<Sphere3D>(&geom.data)->center;
  case GeometricPrimitive3D::Triangle:
    {
      const Triangle3D* tri = AnyCast_Raw<Triangle3D>(&geom.data);
      return (tri->a+tri->b+tri->c)/3.0;
    }
  case GeometricPrimitive3D::AABB:
    return 0.5*(AnyCast_Raw<AABB3D>(&geom.data)->bmin+AnyCast_Raw<AABB3D>(&geom.data)->bmax);
  case GeometricPrimitive3D::Box:
    return AnyCast_Raw<Box3D>(&geom.data)->center();
  case GeometricPrimitive3D::Cylinder:
    return AnyCast_Raw<Cylinder3D>(&geom.data)->center;
  case GeometricPrimitive3D::Ellipsoid:
    return AnyCast_Raw<Ellipsoid3D>(&geom.data)->origin;
  default:
    FatalError("Can't do COM of GeometricPrimitive yet");
    return Vector3();
  }
}

Matrix3 InertiaMatrix(const GeometricPrimitive3D& geom,Real mass)
{
  switch(geom.type) {
  case GeometricPrimitive3D::Empty:
  case GeometricPrimitive3D::Point:
    return Matrix3(0.0);
  case GeometricPrimitive3D::Segment:
    {
      Matrix3 Ilocal;
      const Segment3D* s=AnyCast_Raw<Segment3D>(&geom.data);
      Real len=s->a.distance(s->b);
      CylinderInertiaMatrix(0,len,mass,Ilocal);
      if(len==0) return Ilocal;
      Vector3 x,y,z;
      x=(s->b-s->a)/len;
      GetCanonicalBasis(x,y,z);
      Matrix3 R(x,y,z),I;
      I.mulTransposeB(R*Ilocal,R);
      return I;
    }
  case GeometricPrimitive3D::Sphere:
    {
      Matrix3 I;
      SphereInertiaMatrix(AnyCast_Raw<Sphere3D>(&geom.data)->radius,mass,I);
      return I;
    }
  case GeometricPrimitive3D::AABB:
    {
      Vector3 dims = AnyCast_Raw<AABB3D>(&geom.data)->bmax-AnyCast_Raw<AABB3D>(&geom.data)->bmin;
      Matrix3 I;
      BoxInertiaMatrix(dims.x,dims.y,dims.z,mass,I);
      return I;
    }
  case GeometricPrimitive3D::Box:
    {
      const Box3D* b=AnyCast_Raw<Box3D>(&geom.data);
      Matrix3 Ilocal,I;
      BoxInertiaMatrix(b->dims.x,b->dims.y,b->dims.z,mass,Ilocal);
      RigidTransform T = geom.GetFrame();
      I.mulTransposeB(T.R*I,T.R);
      return I;
    }
  case GeometricPrimitive3D::Ellipsoid:
    {
      const Ellipsoid3D* b=AnyCast_Raw<Ellipsoid3D>(&geom.data);
      Matrix3 Ilocal,I;
      EllipsoidInertiaMatrix(b->dims.x,b->dims.y,b->dims.z,mass,Ilocal);
      RigidTransform T = geom.GetFrame();
      I.mulTransposeB(T.R*I,T.R);
      return I;
    }
  default:
    FatalError("Can't do inertia for that geom type yet");
    return Matrix3();
  }
}

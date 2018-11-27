#include <KrisLibrary/Logger.h>
#include "drawgeometry.h"
#include "drawextra.h"
#include <typeinfo>

using namespace Math3D;

namespace GLDraw {

  void draw(const GeometricPrimitive2D& geom)
  {
    switch(geom.type) {
    case GeometricPrimitive3D::Empty:
      break;
    case GeometricPrimitive2D::Point:
      {
	glBegin(GL_POINTS);
	glVertex2v(*AnyCast_Raw<Vector2>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Segment:
      {
	const Segment2D* seg=AnyCast_Raw<Segment2D>(&geom.data);
	glBegin(GL_LINES);
	glVertex2v(seg->a);
	glVertex2v(seg->b);
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Circle:
      {
	const Circle2D* circle = AnyCast_Raw<Circle2D>(&geom.data);
	drawCircle2D(circle->center,circle->radius);
      }
      break;
    case GeometricPrimitive2D::AABB:
      {
	const AABB2D* aabb=AnyCast_Raw<AABB2D>(&geom.data);
	glBegin(GL_QUADS);
	glVertex2f(aabb->bmin.x,aabb->bmin.y);
	glVertex2f(aabb->bmax.x,aabb->bmin.y);
	glVertex2f(aabb->bmax.x,aabb->bmax.y);
	glVertex2f(aabb->bmin.x,aabb->bmax.y);
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Box:
      {
	const Box2D* box=AnyCast_Raw<Box2D>(&geom.data);
	glBegin(GL_QUADS);
	glVertex2v(box->origin);
	glVertex2v(box->origin+box->dims.x*box->xbasis);
	glVertex2v(box->origin+box->dims.x*box->xbasis+box->dims.y*box->ybasis);
	glVertex2v(box->origin+box->dims.y*box->ybasis);
	glEnd();
	break;
      }
    case GeometricPrimitive2D::Triangle:
      {
	const Triangle2D* tri=AnyCast_Raw<Triangle2D>(&geom.data);
	glBegin(GL_TRIANGLES);
	glVertex2v(tri->a);
	glVertex2v(tri->b);
	glVertex2v(tri->c);
	glEnd();
	break;
      }
    default:
      return;
    }
  }
  void draw(const GeometricPrimitive3D& geom)
  {
    switch(geom.type) {
    case GeometricPrimitive3D::Empty:
      break;
    case GeometricPrimitive3D::Point:
      {
	glBegin(GL_POINTS);
	glVertex3v(*AnyCast_Raw<Vector3>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive3D::Segment:
      {
	const Segment3D* seg=AnyCast_Raw<Segment3D>(&geom.data);
	glBegin(GL_LINES);
	glVertex3v(seg->a);
	glVertex3v(seg->b);
	glEnd();
      }
      break;
      /*
    case GeometricPrimitive3D::Circle:
      {
	const Circle3D* circle = AnyCast_Raw<Circle3D>(&geom.data);
	glPushMatrix();
	glTranslate(circle->center);
	drawCircle(circle->axis,circle->radius);
	glPopMatrix();
      }
      break;
      */
    case GeometricPrimitive3D::AABB:
      {
	const AABB3D* aabb=AnyCast_Raw<AABB3D>(&geom.data);
	drawBoundingBox(aabb->bmin,aabb->bmax);
      }
      break;
    case GeometricPrimitive3D::Box:
      {
	const Box3D* box=AnyCast_Raw<Box3D>(&geom.data);
	Matrix4 m;
	box->getBasis(m);
	glPushMatrix();
	glMultMatrix(m);
	drawBoxCorner(box->dims.x,box->dims.y,box->dims.z);
	glPopMatrix();
	break;
      }
    case GeometricPrimitive3D::Triangle:
      {
	const Triangle3D* tri=AnyCast_Raw<Triangle3D>(&geom.data);
	drawTriangle(tri->a,tri->b,tri->c);
	break;
      }
    case GeometricPrimitive3D::Polygon:
      {
	const Polygon3D* p=AnyCast_Raw<Polygon3D>(&geom.data);
	Plane3D plane;
	p->getPlane(0,plane);
	glNormal3v(plane.normal);
	glBegin(GL_TRIANGLE_FAN);
	glVertex3v(p->vertices[0]);
	for(size_t i=1;i+1<p->vertices.size();i++) {
	  glVertex3v(p->vertices[i]);
	  glVertex3v(p->vertices[i+1]);
	}
	glEnd();
	break;
      }
    case GeometricPrimitive3D::Sphere:
      {
	const Sphere3D* s=AnyCast_Raw<Sphere3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawSphere(s->radius,32,32);
	glPopMatrix();
	break;
      }
    case GeometricPrimitive3D::Cylinder:
      {
	const Cylinder3D* s=AnyCast_Raw<Cylinder3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawCylinder(s->axis*s->height,s->radius,32);
	glPopMatrix();
	break;
      }
      break;
    default:
            LOG4CXX_ERROR(KrisLibrary::logger(),"draw: Unsupported geometry type\n");
      return;
    }
  }

  void draw(const ConvexPolygon2D& geom)
  {
    glBegin(GL_TRIANGLE_FAN);
    for(size_t i=0;i<geom.vertices.size();i++)
      glVertex2v(geom.vertices[i]);
    glEnd();
  }

}

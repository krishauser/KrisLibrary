#include "drawgeometry.h"
#include "drawextra.h"
#include <typeinfo>

using namespace Math3D;

namespace GLDraw {

#if 0
  void draw(const GeometricPrimitive2D& geom)
  {
    switch(geom.type) {
    case GeometricPrimitive2D::Point:
      {
	glBegin(GL_POINTS);
	glVertex2v(*AnyCast<Vector2>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Segment:
      {
	const Segment2D* seg=AnyCast<Segment2D>(&geom.data);
	glBegin(GL_LINES);
	glVertex2v(seg->a);
	glVertex2v(seg->b);
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Circle:
      {
	const Circle2D* circle = AnyCast<Circle2D>(&geom.data);
	drawCircle2D(circle->center,circle->radius);
      }
      break;
    case GeometricPrimitive2D::AABB:
      {
	const AABB2D* aabb=AnyCast<AABB2D>(&geom.data);
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
	const Box2D* box=AnyCast<Box2D>(&geom.data);
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
	const Triangle2D* tri=AnyCast<Triangle2D>(&geom.data);
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
    case GeometricPrimitive3D::Point:
      {
	glBegin(GL_POINTS);
	glVertex3v(*AnyCast<Vector3>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive3D::Segment:
      {
	const Segment3D* seg=AnyCast<Segment3D>(&geom.data);
	glBegin(GL_LINES);
	glVertex3v(seg->a);
	glVertex3v(seg->b);
	glEnd();
      }
      break;
      /*
    case GeometricPrimitive3D::Circle:
      {
	const Circle3D* circle = AnyCast<Circle3D>(&geom.data);
	glPushMatrix();
	glTranslate(circle->center);
	drawCircle(circle->axis,circle->radius);
	glPopMatrix();
      }
      break;
      */
      /*
    case GeometricPrimitive3D::AABB:
      {
	const AABB3D* aabb=AnyCast<AABB3D>(&geom.data);
	drawBoundingBox(aabb->bmin,aabb->bmax);
      }
      break;
      */
    case GeometricPrimitive3D::Box:
      {
	const Box3D* box=AnyCast<Box3D>(&geom.data);
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
	const Triangle3D* tri=AnyCast<Triangle3D>(&geom.data);
	drawTriangle(tri->a,tri->b,tri->c);
	break;
      }
    case GeometricPrimitive3D::Polygon:
      {
	const Polygon3D* p=AnyCast<Polygon3D>(&geom.data);
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
	const Sphere3D* s=AnyCast<Sphere3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawSphere(s->radius,32,32);
	glPopMatrix();
	break;
      }
    case GeometricPrimitive3D::Cylinder:
      {
	const Cylinder3D* s=AnyCast<Cylinder3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawCylinder(s->axis*s->height,s->radius,32);
	glPopMatrix();
	break;
      }
      break;
    default:
      fprintf(stderr,"draw: Unsupported geometry type\n");
      return;
    }
  }
#else

  void draw(const GeometricPrimitive2D& geom)
  {
    printf("cast %s:\n",geom.data.type().name());    
    printf("%s:\n",typeid(Vector2).name());
    printf("Not done yet, weird linker error\n");
  }
  void draw(const GeometricPrimitive3D& geom)
  {
    //AnyValue blah = 4;
    //printf("%d:\n",*AnyCast<int>(&blah));
    printf("geomtype \"%s\":\n",geom.TypeName());    
    printf("geom \"%s\":\n",geom.data.type().name());    
    printf("void \"%s\":\n",typeid(void).name());    
    printf("cylinder %s:\n",typeid(Cylinder3D).name());
    printf("eq? %d:\n",(&typeid(Cylinder3D) == &geom.data.type() ? 1: 0));
    //printf("%s %s\n",(&geom.data)->type().name(),
    //const Cylinder3D* s=AnyCast<Cylinder3D>(&geom.data);
    //printf("AnyCast %s:\n",typeid(*s).name());
    printf("Not done yet, weird linker error\n");
  }
#endif

  void draw(const ConvexPolygon2D& geom)
  {
    glBegin(GL_TRIANGLE_FAN);
    for(size_t i=0;i<geom.vertices.size();i++)
      glVertex2v(geom.vertices[i]);
    glEnd();
  }

}

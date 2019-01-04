#ifndef GL_DRAWEXTRA_H
#define GL_DRAWEXTRA_H

#include "GL.h"
#include <KrisLibrary/math3d/primitives.h>

///if 1, never uses the glXd functions and converts them all to floats
#define GLDRAW_USE_FLOATS_ONLY 1

namespace GLDraw {

using namespace Math3D;

void drawPoint(const Vector3& pt);
void drawLineSegment(const Vector3& a,const Vector3& b);
void drawTriangle(const Vector3& a,const Vector3& b,const Vector3& c);
void drawQuad(const Vector3& a,const Vector3& b,const Vector3& c,const Vector3& d);

void drawWireCircle2D(const Vector2& center,float r,int numIncrements=16);
void drawWireCircle(const Vector3& axis,float r, int numIncrements=16);
void drawWireSphere(float r, int numIncrements=16);
void drawWireArc(float r, const Vector3& axis, const Vector3& dir, float min, float max);
void drawOrientedWireSphere(float r, const Matrix4& basis);
void drawWirePyramid(float l, float w, float h);
void drawWireBox(float l, float w, float h);
void drawWireBoxCorner(float l, float w, float h);
void drawOrientedWireBox(float l, float w, float h, const Matrix4& basis);
void drawWireBoundingBox(const Vector3& bmin,const Vector3& bmax);
void drawWireOctahedron(float l, float w, float h);
void drawWireCylinder(const Vector3& h,Real r,int numSteps=16);
void drawWireCone(const Vector3& h, float r, int numSteps=16);
void drawWireConeFlipped(const Vector3& h, float r, int numSteps=16);

void drawCircle2D(const Vector2& center,float r,int numIncrements=16);
void drawCircle(const Vector3& axis,float r, int numIncrements=16);
void drawSphere(float r, int numSlices, int numStacks);
void drawArc(float r1,float r2, const Vector3& axis, const Vector3& dir, float min, float max);
void drawPyramid(float l, float w, float h);
void drawBox(float l, float w, float h);
void drawBoxCorner(float l, float w, float h);
void drawOrientedBox(float l, float w, float h, const Matrix4& basis);
void drawBoundingBox(const Vector3& bmin,const Vector3& bmax);
void drawOctahedron(float l, float w, float h);
void drawCylinder(const Vector3& h,Real r,int numSteps=16);
void drawCone(const Vector3& h, float r, int numSteps=16);
void drawConeFlipped(const Vector3& h, float r, int numSteps=16);

void drawCoords(float len);
void drawCoordWidget(float len,float axisWidth=0.05,float arrowLen=0.2,float arrowWidth=0.1);
void drawXYGrid(int n, float spacing);
void drawXZGrid(int n, float spacing);
void drawXYCheckerboard(int n, float spacing, float col1[4],float col2[4]);


//overloaded aliases for float/double objects

inline void glTexCoord2v(const GLfloat* v) { glTexCoord2fv(v); }
inline void glTexCoord3v(const GLfloat* v) { glTexCoord3fv(v); }
inline void glTexCoord4v(const GLfloat* v) { glTexCoord4fv(v); }

inline void glNormal3v(const GLfloat* v) { glNormal3fv(v); }

inline void glVertex2v(const GLfloat* v) { glVertex2fv(v); }
inline void glVertex3v(const GLfloat* v) { glVertex3fv(v); }
inline void glVertex4v(const GLfloat* v) { glVertex4fv(v); }

inline void glRasterPos2v(const GLfloat* v) { glRasterPos2fv(v); }
inline void glRasterPos3v(const GLfloat* v) { glRasterPos3fv(v); }
inline void glRasterPos4v(const GLfloat* v) { glRasterPos4fv(v); }

inline void glLoadMatrix(const GLfloat* m) { glLoadMatrixf(m); }
inline void glMultMatrix(const GLfloat* m) { glMultMatrixf(m); }

inline void glTranslate(const GLfloat* t) { glTranslatef(t[0],t[1],t[2]); }
#if GLDRAW_USE_FLOATS_ONLY
inline void glTexCoord2v(const GLdouble* v) { glTexCoord2f((GLfloat)v[0],(GLfloat)v[1]); }
inline void glTexCoord3v(const GLdouble* v) { glTexCoord3f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2]); }
inline void glTexCoord4v(const GLdouble* v) { glTexCoord4f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2],(GLfloat)v[3]); }
inline void glNormal3v(const GLdouble* v) { glNormal3f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2]); }
inline void glVertex2v(const GLdouble* v) { glVertex2f((GLfloat)v[0],(GLfloat)v[1]); }
inline void glVertex3v(const GLdouble* v) { glVertex3f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2]); }
inline void glVertex4v(const GLdouble* v) { glVertex4f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2],(GLfloat)v[3]); }
inline void glRasterPos2v(const GLdouble* v) { glRasterPos2f((GLfloat)v[0],(GLfloat)v[1]); }
inline void glRasterPos3v(const GLdouble* v) { glRasterPos3f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2]); }
inline void glRasterPos4v(const GLdouble* v) { glRasterPos4f((GLfloat)v[0],(GLfloat)v[1],(GLfloat)v[2],(GLfloat)v[3]); }
#else
inline void glTexCoord2v(const GLdouble* v) { glTexCoord2dv(v); }
inline void glTexCoord3v(const GLdouble* v) { glTexCoord3dv(v); }
inline void glTexCoord4v(const GLdouble* v) { glTexCoord4dv(v); }
inline void glNormal3v(const GLdouble* v) { glNormal3dv(v); }
inline void glVertex2v(const GLdouble* v) { glVertex2dv(v); }
inline void glVertex3v(const GLdouble* v) { glVertex3dv(v); }
inline void glVertex4v(const GLdouble* v) { glVertex4dv(v); }
inline void glRasterPos2v(const GLdouble* v) { glRasterPos2dv(v); }
inline void glRasterPos3v(const GLdouble* v) { glRasterPos3dv(v); }
inline void glRasterPos4v(const GLdouble* v) { glRasterPos4dv(v); }
#endif //GLDRAW_USE_FLOATS_ONLY
inline void glLoadMatrix(const GLdouble* m) { glLoadMatrixd(m); }
inline void glMultMatrix(const GLdouble* m) { glMultMatrixd(m); }
inline void glTranslate(const GLdouble* t) { glTranslated(t[0],t[1],t[2]); }

} //namespace GLDraw

#endif

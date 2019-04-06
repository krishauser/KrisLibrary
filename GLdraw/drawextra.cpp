#include "drawextra.h"
#include <iostream>
#include <math/complex.h>
#include <math3d/geometry3d.h>
#include <math3d/basis.h>



namespace GLDraw {

void drawPoint(const Vector3& pt)
{
	glBegin(GL_POINTS);
	glVertex3v(pt);
	glEnd();
}

void drawLineSegment(const Vector3& a,const Vector3& b)
{
	glBegin(GL_LINES);
	glVertex3v(a);
	glVertex3v(b);
	glEnd();
}

void drawTriangle(const Vector3& a,const Vector3& b,const Vector3& c)
{
	glBegin(GL_TRIANGLES);
	glNormal3v(Triangle3D::normal(a,b,c));
	glVertex3v(a);
	glVertex3v(b);
	glVertex3v(c);
	glEnd();
}


void drawQuad(const Vector3& a,const Vector3& b,const Vector3& c,const Vector3& d)
{
	glBegin(GL_TRIANGLE_FAN);
	glNormal3v(Triangle3D::normal(a,b,c));
	glVertex3v(a);
	glVertex3v(b);
	glVertex3v(c);
	glVertex3v(d);
	glEnd();
}

void drawWireCircle2D(const Vector2& center,float radius,int numIncrements)
{
	float inc=float(Pi*Two/numIncrements);
	Complex x(radius,0),dx;
	dx.setPolar(One,inc);

	glBegin(GL_LINE_LOOP);
	for(int i=0;i<numIncrements;i++) {
		glVertex2f(x.x+center.x,x.y+center.y);
		x = x*dx;
	}
	glEnd();
}

void drawWireCircle(const Vector3& axis,float r, int numIncrements)
{
	float inc = fTwoPi/numIncrements;
	int i;
	Vector3 u,v;
	GetCanonicalBasis(axis,u,v);
	Complex x,dx;
	dx.setPolar(One,inc);

	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<numIncrements; i++) {
		glVertex3v(x.x*u+x.y*v);
		x=x*dx;
	}
	glEnd();
}

void drawWireSphere(float r, int numIncrements)
{
	float inc = fTwoPi/numIncrements;
	int i;

	Complex x,dx;
	dx.setPolar(One,inc);

	//x-y circle
	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<numIncrements; i++)
	{
		glVertex3f(x.x, x.y, 0);
		x = x*dx;
	}
	glEnd();

	//y-z circle
	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<numIncrements; i++)
	{
		glVertex3f(0, x.x, x.y);
		x = x*dx;
	}
	glEnd();

	//x-z circle
	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<numIncrements; i++)
	{
		glVertex3f(x.y, 0, x.x);
		x = x*dx;
	}
	glEnd();
}


void drawOrientedWireSphere(float r, const Matrix4& basis)
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrix(basis);
	drawWireSphere(r);
	glPopMatrix();
}

void drawWireArc(float r, const Vector3& axis, const Vector3& dir, float min, float max)
{
	float theta=DtoR(min);
	int steps = (int)ceil(fabs((max-min)*32.0/360.0));
	float inc = DtoR(max-min)/steps;
	int i;

	//get the matrix such that the x axis is lined up with dir, z axis is lined up with axis
	Matrix4 mat;
	mat.set(dir,cross(axis,dir),axis,Vector3(0.0f));

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrix(mat);

	Complex x,dx;
	dx.setPolar(One,inc);

	glBegin(GL_LINE_STRIP);
	x.setPolar(r,theta);
	for(i=0; i<=steps; i++)
	{
		glVertex3f(x.x, x.y, 0);
		x = x*dx;
	}
	glEnd();
	glPopMatrix();
}

void drawWireCylinder(const Vector3& h,Real r,int numSteps)
{
  Real len = h.length();
  Vector3 axis = h/len;

  float inc = fTwoPi/numSteps;
  int i;
  Vector3 u,v;
  GetCanonicalBasis(axis,u,v);
  Complex x,dx;
  dx.setPolar(One,inc);
  
  glBegin(GL_LINE_LOOP);
  x.set(r,0);
  for(i=0; i<numSteps; i++) {
    glVertex3v(x.x*u+x.y*v);
    x=x*dx;
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  x.set(r,0);
  for(i=0; i<numSteps; i++) {
    glVertex3v(h + x.x*u+x.y*v);
    x=x*dx;
  }
  glEnd();

  glBegin(GL_LINES);
  x.set(r,0);
  for(i=0; i<numSteps; i++) {
    glVertex3v(x.x*u+x.y*v);
    glVertex3v(h + x.x*u+x.y*v);
    x=x*dx;
  }
  glEnd();
}


void drawWirePyramid(float l, float w, float h)
{
	float L = l*0.5f, W = w*0.5f;
	glBegin(GL_LINE_LOOP);
	glVertex3f(-L, -W, 0);
	glVertex3f(L, -W, 0);
	glVertex3f(L, W, 0);
	glVertex3f(-L, W, 0);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-L, -W, 0);
	glVertex3f(0, 0, h);
	glVertex3f(L, -W, 0);
	glVertex3f(0, 0, h);
	glVertex3f(L, W, 0);
	glVertex3f(0, 0, h);
	glVertex3f(-L, W, 0);
	glVertex3f(0, 0, h);
	glEnd();
}


void drawWireBox(float l, float w, float h)
{
	float L = l*0.5f, W = w*0.5f, H = h*0.5f;
	drawWireBoundingBox(Vector3(-L,-W,-H),Vector3(L,W,H));
}

void drawWireBoxCorner(float l, float w, float h)
{
	drawWireBoundingBox(Vector3(0,0,0),Vector3(l,w,h));
}

void drawOrientedWireBox(float l, float w, float h, const Matrix4& basis)
{
	glPushMatrix();
	glMultMatrix(basis);
	drawWireBoxCorner(l,w,h);
	glPopMatrix();
}

void drawWireOctahedron(float l, float w, float h)
{
	float L = l*0.5f, W = w*0.5f, H = h*0.5f;

	glBegin(GL_LINE_LOOP);
	glVertex3f(-L, 0, 0);
	glVertex3f(0, -W, 0);
	glVertex3f(L, 0, 0);
	glVertex3f(0, W, 0);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(-L, 0, 0);
	glVertex3f(0, 0, -H);
	glVertex3f(L, 0, 0);
	glVertex3f(0, 0, H);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(0, -W, 0);
	glVertex3f(0, 0, -H);
	glVertex3f(0, W, 0);
	glVertex3f(0, 0, H);
	glEnd();
}


void drawWireBoundingBox(const Vector3&a,const Vector3&b)
{
	//bottom
	glBegin(GL_LINE_LOOP);
	glVertex3f(a.x, a.y, a.z);
	glVertex3f(b.x, a.y, a.z);
	glVertex3f(b.x, b.y, a.z);
	glVertex3f(a.x, b.y, a.z);
	glEnd();

	//top
	glBegin(GL_LINE_LOOP);
	glVertex3f(a.x, a.y, b.z);
	glVertex3f(b.x, a.y, b.z);
	glVertex3f(b.x, b.y, b.z);
	glVertex3f(a.x, b.y, b.z);
	glEnd();

	//sides
	glBegin(GL_LINES);
	glVertex3f(a.x, a.y, a.z);
	glVertex3f(a.x, a.y, b.z);
	glVertex3f(b.x, a.y, a.z);
	glVertex3f(b.x, a.y, b.z);
	glVertex3f(b.x, b.y, a.z);
	glVertex3f(b.x, b.y, b.z);
	glVertex3f(a.x, b.y, a.z);
	glVertex3f(a.x, b.y, b.z);
	glEnd();
}

void drawWireCone(const Vector3& h, float r, int steps)
{
	int i;
	float inc = fTwoPi/steps;

	Vector3 xb,yb;
	Vector3 hdir = h; hdir.inplaceNormalize();
	GetCanonicalBasis(hdir,xb,yb);

	Vector3 point;

	Complex x,dx;
	dx.setPolar(1,inc);
	glBegin(GL_LINES);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y;
		glVertex3v(h);
		glVertex3v(point);

		x = x*dx;
	}
	glEnd();

	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y;
		glVertex3v(point);

		x = x*dx;
	}
	glEnd();
}

void drawWireConeFlipped(const Vector3& h, float r, int steps)
{
	int i;
	float inc = fTwoPi/steps;

	Vector3 xb,yb;
	Vector3 hdir=h; hdir.inplaceNormalize();
	GetCanonicalBasis(hdir,xb,yb);

	Vector3 point;

	Complex x,dx;
	dx.setPolar(1,inc);
	glBegin(GL_LINES);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y + h;
		glVertex3f(0,0,0);
		glVertex3v(point);

		x = x*dx;
	}
	glEnd();

	glBegin(GL_LINE_LOOP);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y + h;
		glVertex3v(point);

		x = x*dx;
	}
	glEnd();
}


void drawCircle2D(const Vector2& center,float radius,int numIncrements)
{
	float inc=Pi*Two/numIncrements;
	Complex x(radius,0),dx;
	dx.setPolar(One,inc);

	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(center.x,center.y);
	for(int i=0;i<=numIncrements;i++) {
		glVertex2f(x.x+center.x,x.y+center.y);
		x = x*dx;
	}
	glEnd();
}

void drawCircle(const Vector3& axis,float r, int numIncrements)
{
	float inc = fTwoPi/numIncrements;
	int i;
	Vector3 u,v;
	GetCanonicalBasis(axis,u,v);
	Complex x,dx;
	dx.setPolar(One,inc);

	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0,0,0);
	x.set(r,0);
	for(i=0; i<=numIncrements; i++) {
		glVertex3v(x.x*u+x.y*v);
		x=x*dx;
	}
	glEnd();
}


void drawPyramid(float l, float w, float h)
{
	float L = l*0.5f, W = w*0.5f;
	drawQuad(Vector3(-L,-W,0),Vector3(-L,W,0),Vector3(L,W,0),Vector3(L,-W,0));
	drawTriangle(Vector3(0,0,h),Vector3(-L,-W,0),Vector3(L,-W,0));
	drawTriangle(Vector3(0,0,h),Vector3(L,-W,0),Vector3(L,W,0));
	drawTriangle(Vector3(0,0,h),Vector3(L,W,0),Vector3(-L,W,0));
	drawTriangle(Vector3(0,0,h),Vector3(-L,W,0),Vector3(-L,-W,0));
}

void drawBox(float l, float w, float h)
{
	float L = l*0.5f, W = w*0.5f, H = h*0.5f;
	drawBoundingBox(Vector3(-L,-W,-H),Vector3(L,W,H));
}

void drawBoxCorner(float l, float w, float h)
{
	drawBoundingBox(Vector3(0,0,0),Vector3(l,w,h));
}

void drawOrientedBox(float l, float w, float h, const Matrix4& basis)
{
	glPushMatrix();
	glMultMatrix(basis);
	drawBoxCorner(l,w,h);
	glPopMatrix();
}

void drawBoundingBox(const Vector3&a,const Vector3&b)
{
	//bottom
	drawQuad(Vector3(a.x, a.y, a.z),
			Vector3(a.x, b.y, a.z),
			Vector3(b.x, b.y, a.z),
			Vector3(b.x, a.y, a.z));
	//top
	drawQuad(Vector3(a.x, a.y, b.z),
			Vector3(b.x, a.y, b.z),
			Vector3(b.x, b.y, b.z),
			Vector3(a.x, b.y, b.z));
	//left
	drawQuad(Vector3(a.x, a.y, a.z),
			Vector3(a.x, a.y, b.z),
			Vector3(a.x, b.y, b.z),
			Vector3(a.x, b.y, a.z));
	//right
	drawQuad(Vector3(b.x, a.y, a.z),
			Vector3(b.x, b.y, a.z),
			Vector3(b.x, b.y, b.z),
			Vector3(b.x, a.y, b.z));

	//back
	drawQuad(Vector3(a.x, a.y, a.z),
			Vector3(b.x, a.y, a.z),
			Vector3(b.x, a.y, b.z),
			Vector3(a.x, a.y, b.z));
	//front
	drawQuad(Vector3(a.x, b.y, a.z),
			Vector3(a.x, b.y, b.z),
			Vector3(b.x, b.y, b.z),
			Vector3(b.x, b.y, a.z));
}

void drawSphere(float r, int numSlices, int numStacks)
{
	Real thetaInc = Pi/Real(numStacks);
	Real phiInc = TwoPi/Real(numSlices);
	Real phi=Zero;
	Real theta;
	for(int i=0;i<numSlices;i++,phi+=phiInc) {
		Real x1=Cos(phi);
		Real x2=Cos(phi+phiInc);
		Real y1=Sin(phi);
		Real y2=Sin(phi+phiInc);
		theta=Zero;
		glBegin(GL_TRIANGLE_STRIP);
		glNormal3f(0,0,1);
		glVertex3f(0,0,r);
		theta+=thetaInc;
		for(int j=1;j<numStacks;j++,theta+=thetaInc) {
			Real cz=Cos(theta);
			Real sz=Sin(theta);
			glNormal3f(x1*sz,y1*sz,cz);
			glVertex3f(r*x1*sz,r*y1*sz,r*cz);
			glNormal3f(x2*sz,y2*sz,cz);
			glVertex3f(r*x2*sz,r*y2*sz,r*cz);
		}
		glNormal3f(0,0,-1);
		glVertex3f(0,0,-r);
		glEnd();
	}
}

void drawArc(float r1,float r2, const Vector3& axis, const Vector3& dir, float min, float max)
{
	float theta=DtoR(min);
	int steps = (int)ceil(fabs((max-min)*32.0/360.0));
	float inc = DtoR(max-min)/steps;
	int i;

	//get the matrix such that the x axis is lined up with dir, z axis is lined up with axis
	Matrix4 mat;
	mat.set(dir,cross(axis,dir),axis,Vector3(0.0f));

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrix(mat);

	Complex x,dx;
	dx.setPolar(One,inc);

	glBegin(GL_TRIANGLE_STRIP);
	x.setPolar(1,theta);
	for(i=0; i<=steps; i++)
	{
	  glNormal3f(0,0,1);
	  glVertex3f(x.x*r1, x.y*r1, 0);
	  glVertex3f(x.x*r2, x.y*r2, 0);
	  x = x*dx;
	}
	glEnd();
	glPopMatrix();
}

void drawCylinder(const Vector3& h,Real r,int steps)
{
  int i;
  float inc = Two*Pi/steps;
  
  Real height=h.norm();
  Vector3 hdir=h/height;
  Vector3 xb,yb;
  GetCanonicalBasis(hdir,xb,yb);

  Complex x,dx;
  dx.setPolar(1,inc);
  Vector3 point,point2;
  Vector3 n,n2;

  //base
  glBegin(GL_TRIANGLE_FAN);
  glNormal3v(-hdir);
  glVertex3f(0,0,0);
  x.set(r,0);
  for(i=0; i<=steps; i++) {
    point = xb*(-x.x) + yb*x.y;
    glVertex3v(point);
    
    x = x*dx;
  }
  glEnd();

  //cap
  glBegin(GL_TRIANGLE_FAN);
  glNormal3v(hdir);
  glVertex3v(h);
  x.set(r,0);
  for(i=0; i<=steps; i++) {
    point = xb*(x.x) + yb*(-x.y) + h;
    glVertex3v(point);
    
    x = x*dx;
  }
  glEnd();

  //sides
  glBegin(GL_TRIANGLE_STRIP);
  dx.setPolar(1,-inc);
  x.set(r,0);
  for(i=0; i<=steps; i++) {
    point = xb*x.x + yb*x.y;
    n = (-x.y*xb + x.x*yb);
    n.inplaceNormalize();

    glNormal3v(n);
    glVertex3v(point);
    glNormal3v(n);
    glVertex3v(point+h);
    x = x*dx;
  }
  glEnd();

}

void drawCone(const Vector3& h, float r, int steps)
{
	int i;
	float inc = fTwoPi/steps;

	Real height=h.norm();
	Vector3 hdir=h/height;
	Vector3 xb,yb;
	GetCanonicalBasis(hdir,xb,yb);

	Complex x,dx;
	dx.setPolar(1,inc);
	Vector3 point,point2;
	Vector3 n,n2;

	//base
	glBegin(GL_TRIANGLE_FAN);
	glNormal3v(-hdir);
	glVertex3f(0,0,0);
	x.set(r,0);
	for(i=0; i<=steps; i++)
	{
	  point = xb*(-x.x) + yb*x.y;
	  glVertex3v(point);

	  x = x*dx;
	}
	glEnd();

	//sides
	glBegin(GL_TRIANGLES);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y;
		n = (-x.y*xb + x.x*yb)/r*height + r*hdir;
		n.inplaceNormalize();
		x = x*dx;
		point2 = xb*x.x + yb*x.y;
		n2 = (-x.y*xb + x.x*yb)/r*height + r*hdir;
		n2.inplaceNormalize();

		glNormal3v(hdir);
		glVertex3v(h);
		glNormal3v(n);
		glVertex3v(point);
		glNormal3v(n2);
		glVertex3v(point2);
	}
	glEnd();
}

void drawConeFlipped(const Vector3& h, float r, int steps)
{
	int i;
	float inc = fTwoPi/steps;

	Real height=h.norm();
	Vector3 hdir=h/height;
	Vector3 xb,yb;
	GetCanonicalBasis(hdir,xb,yb);

	Complex x,dx;
	dx.setPolar(1,inc);
	Vector3 point,point2;
	Vector3 n,n2;

	//base
	glBegin(GL_TRIANGLE_FAN);
	glNormal3v(hdir);
	glVertex3v(h);
	x.set(r,0);
	for(i=0; i<=steps; i++)
	{
	  point = xb*(x.x) + yb*x.y + h;
	  glVertex3v(point);

	  x = x*dx;
	}
	glEnd();

	//sides
	glBegin(GL_TRIANGLES);
	x.set(r,0);
	for(i=0; i<steps; i++)
	{
		point = xb*x.x + yb*x.y + h;
		n = (-xb*x.y + yb*x.x)*height/r - hdir*r;
		n.inplaceNormalize();
		x = x*dx;
		point2 = xb*x.x + yb*x.y + h;
		n2 = (-xb*x.y + yb*x.x)*height/r - hdir*r;
		n2.inplaceNormalize();
		glNormal3v(-hdir);
		glVertex3f(0,0,0);
		glNormal3v(n2);
		glVertex3v(point2);
		glNormal3v(n);
		glVertex3v(point);
	}
	glEnd();
}


void drawCoords (float len)
{
	glBegin(GL_LINES);

	glColor3f(1, 1, 0);
	glVertex3f(0, 0, 0);
	glColor3f(1, 0, 0);
	glVertex3f(len, 0, 0);

	glColor3f(1, 1, 0);
	glVertex3f(0, 0, 0);
	glColor3f(0, 1, 0);
	glVertex3f(0, len, 0);
	
	glColor3f(1, 1, 0);
	glVertex3f(0, 0, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, len);
	
	glEnd();
}

void drawCoordWidget(float size,float thickness,float arrowSize,float arrowWidth)
{
  glEnable(GL_LIGHTING);
  float white[]={1,1,1,1};
  float red[]={1,0,0,1};
  float green[]={0,1,0,1};
  float blue[]={0,0,1,1};

  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,white);
  drawSphere(size*thickness*2.0,16,8);
  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,red);
  drawCylinder(Vector3(size*(1.0-arrowSize),0,0),size*thickness,8);
  glPushMatrix();
  glTranslatef(size*(1.0-arrowSize),0,0);
  drawCone(Vector3(size*arrowSize,0,0),size*arrowWidth,8);
  glPopMatrix();
  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,green);
  drawCylinder(Vector3(0,size*(1.0-arrowSize),0),size*thickness,8);
  glPushMatrix();
  glTranslatef(0,size*(1.0-arrowSize),0);
  drawCone(Vector3(0,size*arrowSize,0),size*arrowWidth,8);
  glPopMatrix();
  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,blue);
  drawCylinder(Vector3(0,0,size*(1.0-arrowSize)),size*thickness,8);
  glPushMatrix();
  glTranslatef(0,0,size*(1.0-arrowSize));
  drawCone(Vector3(0,0,size*arrowSize),size*arrowWidth,8);
  glPopMatrix();
}

void drawXYGrid(int n, float spacing)
{
	float start = -spacing*(n/2);
	float end = spacing*(n-n/2);
	glBegin(GL_LINES);
	for(int a=0; a<n+1; a++)
	{
		glVertex3f(start, a*spacing+start, 0);
		glVertex3f(end, a*spacing+start, 0);

		glVertex3f(a*spacing+start, start, 0);
		glVertex3f(a*spacing+start, end, 0);
	}
	glEnd();
}

void drawXZGrid(int n, float spacing)
{
	float start = spacing*(n/2);
	glBegin(GL_LINES);
	for(int a=0; a<n+1; a++)
	{
		glVertex3f(-start, 0, a*spacing-start);
		glVertex3f(start, 0, a*spacing-start);

		glVertex3f(a*spacing-start, 0, -start);
		glVertex3f(a*spacing-start, 0, start);
	}
	glEnd();
}


void drawXYCheckerboard(int n, float spacing, float col1[4],float col2[4])
{
  glColor4fv(col1);
  glBegin(GL_TRIANGLES);
  float x0=-spacing*n*0.5,y0=-spacing*n*0.5;
  float x,y;
  x=x0;
  for(int i=0;i<n;i++) {
    x += spacing;
    int start=(i&1);
    y=y0;  if(start) y+=spacing;
    for(int j=start;j<n;j+=2) {
      y += spacing+spacing;
      glVertex3f(x,y,0);
      glVertex3f(x+spacing,y,0);
      glVertex3f(x+spacing,y+spacing,0);
      glVertex3f(x,y,0);
      glVertex3f(x+spacing,y+spacing,0);
      glVertex3f(x,y+spacing,0);
    }
  }

  glColor4fv(col2);
  x=x0;
  for(int i=0;i<n;i++) {
    x += spacing;
    int start=1-(i&1);
    y=y0;  if(start) y+=spacing;
    for(int j=start;j<n;j+=2) {
      y += spacing+spacing;
      glVertex3f(x,y,0);
      glVertex3f(x+spacing,y,0);
      glVertex3f(x+spacing,y+spacing,0);
      glVertex3f(x,y,0);
      glVertex3f(x+spacing,y+spacing,0);
      glVertex3f(x,y+spacing,0);
    }
  }
  glEnd();
}


} //namespace GLDraw

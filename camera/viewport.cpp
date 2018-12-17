#include "viewport.h"
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

namespace Camera {

Viewport::Viewport()
  :perspective(true),
   scale(1.0),
   x(0),y(0),w(320),h(240), n(20), f(2000)

{
}

void Viewport::setFOV(float rads)
{
	scale = float(0.5*Inv(Tan(rads*0.5)));
}

float Viewport::getFOV() const
{
    return float(Atan(1.0/(2.0*scale))*2);
}

float Viewport::getHFOV() const
{
    return float(Atan(1.0/(2.0*scale)*float(h)/float(w))*2);
}

void Viewport::setPerspective(bool persp)
//right is y->x z->y -x->z
//left is -y->x z->y x->z
//top is x->x y->y z->z
//bottom is x->x -y->y -z->z
//front is x->x z->y y->z
//back is -x->x z->y -y->z
{
	perspective = persp;
}

bool Viewport::clicked(int mx, int my) const 
{
	return (x<=mx && mx<=x+w && y<=my && my<=y+h);
}

void Viewport::zoom(float s)
{
	scale += s;
	if(scale < 0)
		scale = 0;
}

void Viewport::scroll(float x, float y, float z)
{
	Vector3 xb(xDir()),yb(yDir()),zb(zDir());
	xform.t += xb*x+yb*y+zb*z;
}

void Viewport::getMovementVector(float x, float y, Vector3& v) const 
{
	Vector3 xb(xDir()), yb(yDir());
	xb.inplaceMul(x/scale);
	yb.inplaceMul(y/scale);
	v.add(xb,yb);
}

void Viewport::getMovementVectorAtDistance(float x, float y, float dist, Vector3& v) const
{
	Vector3 xb(xDir()), yb(yDir());
	Real imagePlaneDepth = w*scale;
	xb.inplaceMul(x*dist/imagePlaneDepth);
	yb.inplaceMul(y*dist/imagePlaneDepth);
	v.add(xb,yb);
}
/*
float Viewport::getDepth(int mx, int my) const
{
	float val;
	glReadPixels(mx,my, 1,1, GL_DEPTH_COMPONENT, GL_FLOAT, &val);

	//do the inverse of the perspective transform:
	//z -> zbuffer  = -b/z + a
	//b = n*f / (f-n)
	//a = f / (f-n)
	//do the inverse of that transformation
	if(perspective)
	{
		val = (n*f) / (f - val * (f-n)) / scale;
	}
	else
	//the non-perspective transform is:
	//z -> zbuffer = (z-n)/(f-n)
	{
		val = (f-n)*val + n;
	}
	return val;
}*/


void Viewport::getViewVector(Vector3& v) const 
{
  v.set(zDir());
  v.inplaceNegative();
}








void Viewport::getClickSource(float mx, float my, Vector3& v) const 
{

	Vector3 vv;
	v = xform.t;

	if(perspective)
	{
	}
	else
	{
		mx = mx - x - w/2;
		my = my - y - h/2;
		Vector3 xb(xDir()), yb(yDir());
		v += (mx*xb + my*yb)/scale;
	}
}

void Viewport::getClickVector(float mx, float my, Vector3& v) const
{
	//world vector is
	//x*screen's x basis +
	//y*screen's y basis -
	//(screen width / 2 / tan lens angle) * screen's z basis
	getViewVector(v);
	if(perspective)
	{
		int cx, cy;
		cx = x+w/2;
		cy = y+h/2;

		mx = mx - (float)cx;
		my = my - (float)cy;

		Vector3 xb(xDir()), yb(yDir());
		v += (mx*xb + my*yb)/(w*scale);
	}
}

bool Viewport::project(const Vector3& pt,float& mx,float& my,float& mz) const
{
  Vector3 localpt;
  xform.mulInverse(pt,localpt);
  if(perspective) {
    mx = -localpt.x/localpt.z;
    my = -localpt.y/localpt.z;
    mz = -localpt.z;
    mx *= scale;
    my *= scale;
  }
  else {
    mx = localpt.x*scale;
    my = localpt.y*scale;
    mz = -localpt.z;
  }
  mx = x + w/2 + mx*w;
  my = y + h/2 + my*w;
  return clicked((int)mx,(int)my) && (mz >= n && mz <= f);
}


ostream& operator << (ostream& out, const Viewport& v)
{
  out<<"VIEWPORT"<<endl;
  out<<"FRAME "<<v.x<<" "<<v.y<<" "<<v.w<<" "<<v.h<<endl;
  out<<"PERSPECTIVE "<<v.perspective<<endl;
  out<<"SCALE "<<v.scale<<endl;
  out<<"NEARPLANE "<<v.n<<endl;
  out<<"FARPLANE "<<v.f<<endl;
  out<<"CAMTRANSFORM "<<endl<<v.xform;
  return out;
}


#define READWORD(word) \
  in>>str; \
  if(str!=word) { in.setstate(ios::badbit); return in; }

istream& operator >> (istream& in, Viewport& v)
{
  string str;
  READWORD("VIEWPORT")
  READWORD("FRAME")      in>>v.x>>v.y>>v.w>>v.h;
  READWORD("PERSPECTIVE")in>>v.perspective;
  READWORD("SCALE")      in>>v.scale;
  READWORD("NEARPLANE")  in>>v.n;
  READWORD("FARPLANE")   in>>v.f;
  READWORD("CAMTRANSFORM")in>>v.xform;
  return in;
}

} //namespace CAMERA

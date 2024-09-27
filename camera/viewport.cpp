#include "viewport.h"
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

namespace Camera {

Viewport::Viewport(int _w,int _h,bool opengl_orientation)
  :perspective(true),
   x(0),y(0),w(_w),h(_h), n(0.1), f(1000),
   fx(_w*0.5),fy(_w*0.5),cx(_w*0.5),cy(_h*0.5),ori(CameraConventions::XYZ)
{
	if(!opengl_orientation)
		ori = CameraConventions::ROS;
}

void Viewport::resize(int _w, int _h)
{
	float xratio = float(_w)/float(w);
	float yratio = float(_h)/float(h);
	w = _w;
	h = _h;
	cx *= xratio;
	cy *= yratio;
	fx *= xratio;
	fy *= xratio;
}

void Viewport::setScale(float _scale)
{
	fx = _scale*w*0.5;
	fy = _scale*w*0.5;
}

void Viewport::setFOV(float rads)
{
	if(perspective) {
		setScale(float(1.0/Tan(rads*0.5)));
	}
	else {
		setScale(2.0/rads);
	}
}

float Viewport::getFOV() const
{
	if(perspective) {
		float scale = 2.0*fx/w;
		return float(Atan(1.0/scale)*2);
	}
	else {
		return w/fx;
	}
}

float Viewport::getVerticalFOV() const
{
	if(perspective) {
		float scale = 2.0*fy/h;
		return float(Atan(1.0/scale)*2);
	}
	else {
		return h/fy;
	}

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


void Viewport::scroll(float x, float y, float z)
{
	pose.t += pose.R*Vector3(x,y,z);
}

Vector3 Viewport::right() const
{
	return Vector3(pose.R.col1());
}

Vector3 Viewport::up() const
{
	if(ori == CameraConventions::ROS) {
		return -Vector3(pose.R.col2());
	}
	else {
		return Vector3(pose.R.col2());
	}
}

Vector3 Viewport::forward() const
{
	if(ori == CameraConventions::ROS) {
		return Vector3(pose.R.col3());
	}
	else {
		return -Vector3(pose.R.col3());
	}
}

void Viewport::getMovementVector(float dx, float dy, Vector3& v) const 
{
	if(perspective) fprintf(stderr,"Viewport::getMovementVector: warning, not maningful for perspective projection\n");
	float xscale = 2.0*fx/w;
	float yscale = 2.0*fy/w;
	Vector3 vlocal(dx/xscale,dy/yscale,0);
	pose.R.mul(vlocal,v);
}

void Viewport::getMovementVectorAtDistance(float dx, float dy, float dist, Vector3& v) const
{
	if(!perspective) {
		getMovementVector(dx,dy,v);
	}
	else {
		Vector3 vlocal(dx*dist/fx,dy*dist/fy,0);
		pose.R.mul(vlocal,v);
	}
}

float Viewport::getSizeAtDistance(float world_size, float dist) const
{
	if(perspective) {
		return fx * world_size / dist;
	}
	else {
		return world_size * fx;
	}
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
  v.set(pose.R.col3());

  if(ori != CameraConventions::ROS)
	v.inplaceNegative();
}








void Viewport::getClickSource(float mx, float my, Vector3& v) const 
{

	Vector3 vv;
	v = pose.t;

	if(perspective)
	{
	}
	else
	{
		mx = mx - x - cx;
		my = my - y - cy;
		Vector3 vlocal(mx/fx,my/fy,0);
		v += pose.R*vlocal;
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
		mx = mx - cx;
		my = my - cy;

		Vector3 vlocal(mx/fx,my/fy,0);
		v += pose.R*vlocal;
		v.inplaceNormalize();
	}
}

bool Viewport::project(const Vector3& pt,float& mx,float& my,float& mz) const
{
  Vector3 localpt;
  pose.mulInverse(pt,localpt);
  if(perspective) {
	if(ori != CameraConventions::ROS) {
		mx = -localpt.x/localpt.z;
		my = -localpt.y/localpt.z;
		mz = -localpt.z;
	}
	else {
		mx = localpt.x/localpt.z;
		my = localpt.y/localpt.z;
		mz = localpt.z;
	}
    mx *= fx;
    my *= fy;
  }
  else {
	if(ori != CameraConventions::ROS) {
		mx = localpt.x*fx;
		my = localpt.y*fy;
		mz = -localpt.z;
	}
	else {
		mx = localpt.x*fx;
		my = localpt.y*fy;
		mz = localpt.z;
	}
  }
  mx += cx;
  my += cy;
  return clicked((int)mx,(int)my) && (mz >= n && mz <= f);
}


void Viewport::projectionMatrix(Matrix4& P) const
{
	P.setZero();
	P(3,3) = 1;
	P(0,0) = fx;
	P(1,1) = fy;
	P(0,3) = cx;
	P(1,3) = cy;
	if(ori != CameraConventions::ROS) {
		//z is backward
		P(0,0) *= -1;
		P(1,1) *= -1;
		P(2,2) = -1;
	}
	else
		P(2,2) = 1;
}


ostream& operator << (ostream& out, const Viewport& v)
{
  out<<"VIEWPORT"<<endl;
  out<<"FRAME "<<v.x<<" "<<v.y<<" "<<v.w<<" "<<v.h<<endl;
  out<<"PERSPECTIVE "<<v.perspective<<endl;
  out<<"INTRINSICS "<<v.fx<<" "<<v.fy<<" "<<v.cx<<" "<<v.cy<<endl;
  out<<"NEARPLANE "<<v.n<<endl;
  out<<"FARPLANE "<<v.f<<endl;
  out<<"CAMTRANSFORM "<<endl<<v.pose;
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
  in>>str;
  if(str=="INTRINSICS") {
	in>>v.fx>>v.fy>>v.cx>>v.cy;
  }
  else if(str=="SCALE") {
	v.cx = v.w/2;
	v.cy = v.h/2;
	float scale;
	in >> scale;
	v.setScale(scale);
  }
  else {
	in.setstate(ios::badbit);
	return in;
  }
  READWORD("NEARPLANE")  in>>v.n;
  READWORD("FARPLANE")   in>>v.f;
  READWORD("CAMTRANSFORM")in>>v.pose;
  return in;
}

} //namespace CAMERA

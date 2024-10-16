#include "viewport.h"
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

namespace Camera {

Viewport::Viewport(int _w,int _h,bool opengl_orientation)
  :perspective(true),
   x(0),y(0),w(_w),h(_h), n(0.1), f(1000),
   fx(_w*0.5),fy(_w*0.5),cx(_w*0.5),cy(_h*0.5),ori(CameraConventions::OpenGL)
{
	pose.setIdentity();
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

void Viewport::setOrientation(CameraConventions::CamOrientation o)
{
	if(o != ori) {
		pose.R = CameraConventions::orientationMatrix(ori,o)*pose.R;
		ori = o;
	}
}

void Viewport::setScale(float _scale)
{
	fx = _scale*w*0.5;
	fy = _scale*w*0.5;
}

AABB2D Viewport::getViewRectangle(float depth, bool halfPixelAdjustment) const
{
	AABB2D bb;
	if(halfPixelAdjustment) {
		bb.bmin.x = (0.5-cx)/fx;
		bb.bmax.x = (w-0.5-cx)/fx;
		bb.bmin.y = (0.5-cy)/fy;
		bb.bmax.y = (h-0.5-cy)/fy;
	}
	else {
		bb.bmin.x = (-cx)/fx;
		bb.bmax.x = (w-cx)/fx;
		bb.bmin.y = (-cy)/fy;
		bb.bmax.y = (h-cy)/fy;
	}
	if(perspective) {
		if(depth <= 0) depth = n;
		bb.bmin.x *= depth;
		bb.bmax.x *= depth;
		bb.bmin.y *= depth;
		bb.bmax.y *= depth;
	}
	return bb;
}


void Viewport::setFOV(float xrads, float yrads)
{
	if(perspective) {
		fx = float(w)*0.5/Tan(xrads*0.5);
		if(yrads < 0)
			fy = fx;
		else
			fy = float(h)*0.5/Tan(yrads*0.5);
	}
	else {
		fx = float(w)/xrads;
		if(yrads < 0)
			fy = fx;
		else
			fy = float(h)/yrads;
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
	v = pose.t;

	if(perspective)
	{
	}
	else
	{
		mx = mx + 0.5 - x - cx;
		my = my + 0.5 - y - cy;
		Vector3 vlocal(mx/fx,my/fy,0);
		v += pose.R*vlocal;
	}
}

void Viewport::getClickVector(float mx, float my, Vector3& v) const
{
	getViewVector(v);
	if(perspective)
	{
		mx = mx + 0.5 - x - cx;
		my = my + 0.5 - x  - cy;

		Vector3 vlocal(mx/fx,my/fy,0);
		v += pose.R*vlocal;
		v.inplaceNormalize();
	}
}

void Viewport::deproject(float mx, float my, Vector3& src,Vector3& dir) const 
{
	getViewVector(dir);
	src = pose.t;

	mx = mx - cx;
	my = my - cy;
	Vector3 vlocal(mx/fx,my/fy,0);

	if(perspective)
	{
		dir += pose.R*vlocal;
		dir.inplaceNormalize();
	}
	else
	{
		Vector3 vlocal(mx/fx,my/fy,0);
		src += pose.R*vlocal;
	}
}


void Viewport::getAllRays(std::vector<Vector3>& sources, std::vector<Vector3>& directions, bool halfPixelAdjustment, bool normalize, bool topdown) const
{
	sources.resize(w*h);
	directions.resize(w*h);
	Real invfx = 1.0/fx;
	Real invfy = 1.0/fy;
	if(perspective) {
		Real mz = (ori == CameraConventions::ROS ? 1 : -1);
		int k=0;
		Real my = -cy;
		if(halfPixelAdjustment) my += 0.5;
		my *= invfy;
		if(topdown) {
			my = (h-1)*invfy+my;
			invfy = -invfy;
		}
		for(int i=0;i<h;i++) {
			Real mx = -cx;
			if(halfPixelAdjustment) mx += 0.5;
			mx *= invfx;
			for(int j=0;j<w;j++,k++) {	
				sources[k] = pose.t;
				directions[k] = pose.R*Vector3(mx,my,mz);
				mx += invfx;
			}
			my += invfy;
		}
		if(normalize) 
			for(auto& d:directions) d.inplaceNormalize();
	}
	else {
		Vector3 z(pose.R.col3());
		if(ori == CameraConventions::OpenGL) z.inplaceNegative();
		int k=0;
		Real my = -cy;
		if(halfPixelAdjustment) my += 0.5;
		my *= invfy;
		if(topdown) {
			my = (h-1)*invfy+my;
			invfy = -invfy;
		}
		for(int i=0;i<h;i++) {
			Real mx = -cx;
			if(halfPixelAdjustment) mx += 0.5;
			mx *= invfx;
			for(int j=0;j<w;j++,k++) {	
				sources[k] = pose.t + pose.R*Vector3(mx,my,0);
				directions[k] = z;
				mx += invfx;
			}
			my += invfy;
		}
	}
}

void Viewport::getAllRays(Array2D<Vector3>& sources, Array2D<Vector3>& directions, bool halfPixelAdjustment, bool normalize, bool topdown) const
{
	sources.resize(w,h);
	directions.resize(w,h);
	Real invfx = 1.0/fx;
	Real invfy = 1.0/fy;
	if(perspective) {
		Real mz = (ori == CameraConventions::ROS ? 1 : -1);
		int k=0;
		Real my = -cy;
		if(halfPixelAdjustment) my += 0.5;
		my *= invfy;
		if(topdown) {
			my = (h-1)*invfy+my;
			invfy = -invfy;
		}
		for(int i=0;i<h;i++) {
			Real mx = -cx;
			if(halfPixelAdjustment) mx += 0.5;
			mx *= invfx;
			for(int j=0;j<w;j++,k++) {	
				sources(j,i) = pose.t;
				directions(j,i) = pose.R*Vector3(mx,my,mz);
				mx += invfx;
			}
			my += invfy;
		}
		if(normalize) 
			for(auto& d:directions) d.inplaceNormalize();
	}
	else {
		Vector3 z(pose.R.col3());
		if(ori == CameraConventions::OpenGL) z.inplaceNegative();
		int k=0;
		Real my = -cy;
		if(halfPixelAdjustment) my += 0.5;
		my *= invfy;
		if(topdown) {
			my = (h-1)*invfy+my;
			invfy = -invfy;
		}
		for(int i=0;i<h;i++) {
			Real mx = -cx;
			if(halfPixelAdjustment) mx += 0.5;
			mx *= invfx;
			for(int j=0;j<w;j++,k++) {	
				sources(j,i) = pose.t + pose.R*Vector3(mx,my,0);
				directions(j,i) = z;
				mx += invfx;
			}
			my += invfy;
		}
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


void Viewport::imageMatrix(Matrix4& P) const
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

void Viewport::projectionMatrix(Matrix4& P,bool world) const
{
	P.setZero();
	Vector3 xb(pose.R.col1());
	Vector3 yb(pose.R.col2());
	Vector3 zb(pose.R.col3());
	if(!world) {
		xb.set(1.0,0.0,0.0);
		yb.set(0.0,1.0,0.0);
		zb.set(0.0,0.0,1.0);
	}
	if(perspective) {
		//x = e1^T(R^-1*(p - t)) = xb^T p - xb^T t
		//y = e2^T(R^-1*(p - t)) = yb^T p - yb^T t
		//z = e3^T(R^-1*(p - t)) = zb^T p - zb^T t
		//u = (x/z*fx + cx)/w = (x*fx/w + cx/w*z) / z = r / q
		//v = (y/z*fy + cy)/h = (y*fy/h + cy/h*z) / z = s / q
		//w = t/q  so  w(pf) = 1 = t(pf)/q(pf) and w(pn) = 0 = t(pn)/q(pn) for pf all far points and pn all near points
		//    f = t(pf) and 0 = t(pn).   Letting t(p) = c*z(p) + d, we have c*f + d = f and c*n + d = 0 so
		//    c = f/(f-n), d = -n*f/(f-n)
		//q = z
		Vector3 ub = xb*fx/w + zb*cx/w;
		Vector3 vb = yb*fy/h + zb*cy/h;
		P.setRow1(ub);  P(0,3) = (-xb.dot(pose.t)*fx-zb.dot(pose.t)*cx)/w;
		P.setRow2(vb);  P(1,3) = (-yb.dot(pose.t)*fy-zb.dot(pose.t)*cy)/h;
		P.setRow3(zb*(f/(f-n))); P(2,3) = -n*f/(f-n);
		P.setRow4(zb); P(3,3) = -zb.dot(pose.t);
	}
	else {
		//u = (x*fx + cx)/w = (x*fx/w + cx/w) 
		//v = (y*fy + cy)/h = (y*fy/h + cy/h)
		//w = (z-n)/(f-n) = z/(f-n) - n/(f-n)
		Vector3 ub = xb*fx/w;
		Vector3 vb = yb*fy/h;
        P.setRow1(ub); P(0,3) = cx/w-xb.dot(pose.t)*fx/w;
        P.setRow2(vb); P(1,3) = cx/w-yb.dot(pose.t)*fy/h;
		P.setRow3(zb/(f-n)); P(2,3) = (-zb.dot(pose.t)-n)/(f-n);
		P(3,3) = 1;
	}
}


bool Viewport::operator == (const Viewport& rhs) const
{
	return (perspective == rhs.perspective &&
		x == rhs.x && y == rhs.y && w == rhs.w && h == rhs.h &&
		n == rhs.n && f == rhs.f &&
		fx == rhs.fx && fy == rhs.fy && cx == rhs.cx && cy == rhs.cy &&
		pose == rhs.pose && ori == rhs.ori);
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

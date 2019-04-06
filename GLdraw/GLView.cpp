#include <KrisLibrary/Logger.h>
#include "GLView.h"
#include "GL.h"
#include "drawextra.h"
#include <stdio.h>
using namespace std;
using namespace GLDraw;

GLView::GLView()
:x(0),y(0),w(1),h(1)
{}

void GLView::setCurrentGL()
{
	glViewport((GLint)x,(GLint)y,(GLsizei)w,(GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrix(projection);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrix(modelview);
}

void GLView::getCurrentGL()
{
	int vp[4];
#ifdef MATH_DOUBLE
	GLdouble mv[16],pr[16];
	glGetIntegerv(GL_VIEWPORT,vp);
	glGetDoublev(GL_PROJECTION_MATRIX,pr);
	glGetDoublev(GL_MODELVIEW_MATRIX,mv);
#else
	GLfloat mv[16],pr[16];
	glGetIntegerv(GL_VIEWPORT,vp);
	glGetFloatv(GL_PROJECTION_MATRIX,pr);
	glGetFloatv(GL_MODELVIEW_MATRIX,mv);
#endif //MATH_DOUBLE
	x=(Real)vp[0]; y=(Real)vp[1]; w=(Real)vp[2]; h=(Real)vp[3];
	projection.set(pr);
	modelview.set(mv);
	updateInverses();
}

void GLView::updateInverses()
{
  if(!modelviewInverse.setInverse(modelview)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"GLView: Modelview matrix is singular!\n");
  }
  if(!projectionInverse.setInverse(projection)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"GLView: Projection matrix is singular!\n");
  }
}

void GLView::setViewport(const Camera::Viewport& v)
{
	x=(Real)v.x;
	y=(Real)v.y;
	w=(Real)v.w;
	h=(Real)v.h;
	v.getCameraMatrix(modelview);

	if(v.perspective)
	{
		Real aspect = w/h;
		Real width = v.n*Half/v.scale;
		Real height = v.n*Half/(aspect*v.scale);
		getFrustumMatrix(-width, width, -height, height, v.n, v.f, projection);
	}
	else
	{
		Real width = w*Half/v.scale;
		Real height = h*Half/v.scale;
		getOrthoMatrix(-width, width, -height, height, v.n, v.f, projection);
	}
	updateInverses();
}

bool GLView::getViewport(Camera::Viewport& v) const
{
	v.x=(int)x; v.y=(int)y; v.w=(int)w; v.h=(int)h;
	v.setCameraMatrix(modelview);
	const Matrix4& p=projection;

	//now the projmat...
	bool error=false;
	if(p(3,3)==One) {
		//ortho
		v.perspective=false;
		//m00=2/(r-l)=2/(w/scale)=scale*2/w;
		//m11=2/(t-b)=2/(h/scale)=scale*2/h;
		//m22=-2/(f-n)
		//m23=-(f+n)/(f-n)
		//[f*m22-n*m22=-2  ] => [m22   -m22 ] * [f] = [-2]
		//[f*m23-n*m23=-f-n]    [m23+1 -m23+1] * [n]   [0 ]
		v.scale=(float)(p(0,0)*(w/2));
		Matrix2 A;
		A(0,0)=p(2,2); A(0,1)=-p(2,2);
		A(1,0)=p(2,3)+1; A(1,1)=-p(2,3)+1;
		if(!A.inplaceInverse()) error=true;
		Vector2 nf;
		A.mul(Vector2(-2,Zero),nf);
		v.n=(float)nf.x;
		v.f=(float)nf.y;
		
#ifdef _DEBUG
		if(v.scale != (float)(p(1,1)*h*0.5)) error=true;
		  //no projection translation
		if(p(0,3)!=Zero) error=true;
		if(p(1,3)!=Zero) error=true;
		//these ought to be zero
		if(p(3,2)!=Zero) error=true;
		if(p(3,1)!=Zero) error=true;
		if(p(3,0)!=Zero) error=true;
		if(p(2,1)!=Zero) error=true;
		if(p(2,0)!=Zero) error=true;
		if(p(1,0)!=Zero) error=true;
		if(p(1,2)!=Zero) error=true;
		if(p(0,2)!=Zero) error=true;
		if(p(0,1)!=Zero) error=true;
#endif
	}
	else if(p(3,3)==Zero) {
		//perspective
		v.perspective=true;
		//m00=2n/(r-l)=2n/n/scale=2*scale
		//m11=2n/(t-b)=2n/n/(aspect*scale)=2*aspect*scale
		//m22=-(f+n)/(f-n)
		//m23=-2fn/(f-n)
		//f*(m22+1)+n*(-m22+1)=0
		//f*m23 +n*(-m23)=2fn
		//f=n*(m22-1)/(m22+1)=n*c
		//so n*(c*m23-m23)=2*c*n*n
		//c*m23-m23=2*c*n
		v.scale= (float)(p(0,0)*Half);
		if(p(2,2)-One != Zero) {
			//solve for n
			Real c=(p(2,2)-1)/(p(2,2)+1);
			v.n= (float)(p(2,3)*Half-p(2,3)*Half/c);
			v.f= (float)(v.n*c);
		}
		else error=true;

#ifdef _DEBUG
		if(v.scale!=p(1,1)*Half*h/w) error=true;
		//no translation
		if(p(0,2)!=Zero) error=true;
		if(p(1,2)!=Zero) error=true;

		if(p(0,1)!=Zero) error=true;
		if(p(0,3)!=Zero) error=true;
		if(p(1,0)!=Zero) error=true;
		if(p(1,3)!=Zero) error=true;
		if(p(2,0)!=Zero) error=true;
		if(p(2,1)!=Zero) error=true;
		if(p(3,0)!=Zero) error=true;
		if(p(3,1)!=Zero) error=true;
		if(p(3,2)!=-One) error=true;
#endif
	}
	else error=true;
	
	if(error) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"GLView: There was an error in the projection matrix\n");
	  return false;
	}
	return true;
}




bool GLView::getFrustumMatrix(Real l,Real r,Real b,Real t,Real n,Real f,Matrix4& m)
{
	if(f<Zero) return false;
	if(n<Zero) return false;
	if(r==l) return false;
	if(b==t) return false;
	if(f==n) return false;
	m.setZero();
	m(0,0)=Two*n/(r-l);
	m(0,2)=(r+l)/(r-l);
	m(1,1)=Two*n/(t-b);
	m(1,2)=(t+b)/(t-b);
	m(2,2)=-(f+n)/(f-n);
	m(2,3)=-Two*f*n/(f-n);
	m(3,2)=-One;
	return true;
}

bool GLView::getOrthoMatrix(Real l,Real r,Real b,Real t,Real n,Real f,Matrix4& m)
{
	if(r==l) return false;
	if(b==t) return false;
	if(f==n) return false;
	m.setZero();
	m(0,0)=Two/(r-l);
	m(0,3)=-(r+l)/(r-l);
	m(1,1)=Two/(t-b);
	m(1,3)=-(t+b)/(t-b);
	m(2,2)=-Two/(f-n);
	m(2,3)=-(f+n)/(f-n);
	m(3,3)=One;
	return true;
}

void GLView::worldToScreen(const Vector4& p, Vector3& out)
{
	Vector4 pobj;
	Vector3 phom;
	worldToObject(p,pobj);
	pobj.get(phom);
	out.x = (phom.x+One)*w + x;
	out.y = (phom.y+One)*h + y;
	out.z = phom.z;
}

void GLView::worldToObject(const Vector4& p, Vector4& out)
{
	Vector4 pcam;
	worldToCamera(p,pcam);
	projection.mul(pcam,out);
}

void GLView::worldToCamera(const Vector4& p, Vector4& out)
{
	modelview.mul(p,out);
}


void GLView::screenToWorld(const Vector3& p, Vector4& out)
{
	Vector3 phom;
	phom.x = (p.x-x)*Two/w-One;
	phom.y = (p.y-y)*Two/h-One;
	phom.z = p.z;
	Vector4 pobj;
	pobj.set(phom.x,phom.y,phom.z,One);
	//there's a choice here of w
	//we should pick the one that gives out.w=One (just rescale later)
	objectToWorld(pobj,out);
	if(out.w!=Zero) {
		Real invw=Inv(out.w);
		out*=invw;
	}
}

void GLView::objectToWorld(const Vector4& p, Vector4& out)
{
	Vector4 pcam;
	projectionInverse.mul(p,pcam);
	cameraToWorld(pcam,out);
}

void GLView::cameraToWorld(const Vector4& p, Vector4& out)
{
	modelviewInverse.mul(p,out);
}

void GLView::pointWorldToObject(const Vector3& p, Vector3& out)
{
	Vector4 out4;
	worldToObject(Vector4(p),out4);
	out4.get(out);
}

void GLView::pointWorldToCamera(const Vector3& p, Vector3& out)
{
	modelview.mulPoint(p,out);
}

void GLView::pointObjectToWorld(const Vector3& p, Vector3& out)
{
	Vector4 out4;
	objectToWorld(Vector4(p),out4);
	out4.get(out);
}

void GLView::pointCameraToWorld(const Vector3& p, Vector3& out)
{
	modelviewInverse.mulPoint(p,out);
}

/*void GLView::vectorWorldToObject(const Vector3& v, Vector3& out);
void GLView::vectorWorldToCamera(const Vector3& p, Vector3& out);
void GLView::vectorObjectToWorld(const Vector3& v, Vector3& out);
void GLView::vectorCameraToWorld(const Vector3& p, Vector3& out);
void GLView::normalWorldToObject(const Vector3& v, Vector3& out);
void GLView::normalWorldToCamera(const Vector3& p, Vector3& out);
void GLView::normalObjectToWorld(const Vector3& v, Vector3& out);
void GLView::normalCameraToWorld(const Vector3& p, Vector3& out);
*/

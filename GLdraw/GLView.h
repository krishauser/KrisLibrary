#ifndef GL_VIEW_H
#define GL_VIEW_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/camera/viewport.h>

namespace GLDraw {

using namespace Math3D;

//pscreen = V*homogeneous(P*M*pworld)
//pcamera=M*pworld
//pobject=P*M*pworld
class GLView
{
public:
	GLView();
	void setCurrentGL();
	void getCurrentGL();
	void updateInverses();  //must call this after each change of modelview/projection matrices to update the inverse transforms

	void setViewport(const Camera::Viewport& v);
	bool getViewport(Camera::Viewport& v) const;

	static bool getFrustumMatrix(Real l,Real r,Real b,Real t,Real n,Real f,Matrix4& m);
	static bool getOrthoMatrix(Real l,Real r,Real b,Real t,Real n,Real f,Matrix4& m);

	void worldToScreen(const Vector4& p, Vector3& out);
	void worldToObject(const Vector4& p, Vector4& out);
	void worldToCamera(const Vector4& p, Vector4& out);
	void screenToWorld(const Vector3& p, Vector4& out);
	void objectToWorld(const Vector4& p, Vector4& out);
	void cameraToWorld(const Vector4& p, Vector4& out);

	void pointWorldToObject(const Vector3& p, Vector3& out);
	void pointWorldToCamera(const Vector3& p, Vector3& out);
	void pointObjectToWorld(const Vector3& p, Vector3& out);
	void pointCameraToWorld(const Vector3& p, Vector3& out);
	void vectorWorldToObject(const Vector3& v, Vector3& out);
	void vectorWorldToCamera(const Vector3& p, Vector3& out);
	void vectorObjectToWorld(const Vector3& v, Vector3& out);
	void vectorCameraToWorld(const Vector3& p, Vector3& out);
	void normalWorldToObject(const Vector3& v, Vector3& out);
	void normalWorldToCamera(const Vector3& p, Vector3& out);
	void normalObjectToWorld(const Vector3& v, Vector3& out);
	void normalCameraToWorld(const Vector3& p, Vector3& out);

	Real x,y,w,h;   //viewport
	Matrix4 modelview;
	Matrix4 projection;

	Matrix4 modelviewInverse;
	Matrix4 projectionInverse;
};

} //namespace GLDraw

#endif

#ifndef CAMERA_VIEWPORT_H
#define CAMERA_VIEWPORT_H

#include "camera.h"

namespace Camera {

/** @brief A class containing viewport information for a perspective or
 * orthographic camera.
 * 
 * Defaults: perspective camera, w x h provided to constructor, field of view
 * 90 degrees, near and far planes 0.1 and 1000.
 * 
 * If you want to use orthographic projection, call setPerspective(false)
 * first before setting other parameters.
 * 
 * A perspective camera maps the local point (x,y,z) to the pixel (fx*x/|z|+cx,fy*y/|z|+cy).
 * An orthographic camera maps the local point to the pixel (fx*x+cx,fy*y+cy).
 * 
 * This by default uses OpenGL coordinate convention
 * in which x is to the right of the image, y is upward in the image, and z is backward. 
 * Also, up in the image corresponds with increasing pixel y coordinate. ((0,0) is lower left)
 * 
 * If you want to use the other common coordinate convention, pass opengl_orientation=false
 * to the constructor.
 * 
 * This convention has x to the right, y is downward, and z is forward.
 * Also, up in the image corresponds to decreasing pixel y coordinate ((0,0) is upper left)
 */
class Viewport
{
public:
	Viewport(int w=640,int h=480, bool opengl_orientation=true);
	/// Sets the width and height of the viewport, keeping the the focal lengths and
	///principal point proportional to their current settings. 
	void resize(int w,int h);
	///Sets the viewport as a perspective or orthographic viewport
	void setPerspective(bool enabled);
	///Sets the absolute field of view in the width direction.  Value is
	///in radians for perspective projection and in world units for
	///orthographic projection.
	void setFOV(float rads);
	///Returns the absolute field of view in the width direction.  Result
	///is in radians for perspective projection and in world units for
	///orthographic projection.
	float getFOV() const;
	///Returns the absolute field of view in the vertical direction.  Result
	///is in radians for perspective projection and in world units for
	///orthographic projection.
	float getVerticalFOV() const;
	/// Sets a uniform scale factor for the viewport.  For perspective cameras,
	/// a scale of 1 shows the range [-1,1] at distance 1. For orthographic cameras,
	/// a scale of 1 shows the range [-1,1].
	void setScale(float scale);

	///Returns true if the point (mx,my) is within the viewport
	bool clicked(int mx, int my) const;
	///Scrolls the viewport by the vector (x,y,z) given in the
	///local camera frame.
	void scroll(float x, float y, float z=0);

	/// World coordinates of the right vector 
	Vector3 right() const;
	/// World coordinates of the up vector
	Vector3 up() const;
	/// World coordinates of the forward vector
	Vector3 forward() const;

	///Returns the forward direction in world coordinates 
	void getViewVector(Vector3&) const;
	///Returns a screen-parallel world-space vector for a screen-space
	///displacement of dx,dy.  The magnitude only makes sense for orthographic
	///projection.
	void getMovementVector(float dx, float dy, Vector3&) const;
	///Returns a screen-parallel world-space vector that displaces a point
	///at distance dist by dx in the screen's x direction and dy in the y
	///direction
	void getMovementVectorAtDistance(float dx, float dy, float dist, Vector3&) const;
	///Returns the screen-space size of an object of world-space size world_size
	///at a particular distance 
	float getSizeAtDistance(float world_size, float dist) const;
	
	///find the direction on the image plane corresponding to a
	///screen space point (mx,my).  Result is in world space.  Note: return vector
	///is normalized.
	void getClickVector(float mx, float my, Vector3&) const;
	///find the point on the image plane corresponding to a
	///screen space point (mx,my).  Result is in world space.
	void getClickSource(float mx, float my, Vector3&) const;

	///Computes the screen coordinates of a point pt in world space.
	///Returns true if the point is within the view frustum.
	bool project(const Vector3& pt,float& mx,float& my,float& mz) const;

	///Returns the camera's projection matrix.  That is, the screen coordinates are
	///proj*(q.x/q.z,q.y/q.z,q.z,1) for a perspective
	///camera quere q is the local coordinates q=pose^-1*p.  The output z value has
	///positive z indicating forward.
	///For an orthographic camera, the screen coordinates are proj*(q.x,q.y,q.z,1).
	void projectionMatrix(Matrix4& proj) const;


	bool perspective;
 
	int x,y;    ///< Starting corner of viewport
	int w,h;    ///< Width and height of viewport
	float n, f; ///< Near and far clipping planes
	float fx, fy;  ///< Focal length in pixels
	float cx, cy;  ///< Principal point relative to (x,y) in pixels

	RigidTransform pose;   ///< The camera's pose in world coordinates
	CameraConventions::Orientation ori; ///< The orientation of the camera's y and z axes
};

std::ostream& operator << (std::ostream& out, const Viewport&);
std::istream& operator >> (std::istream& in, Viewport&);

} //namespace Camera

#endif

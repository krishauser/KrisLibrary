#ifndef CAMERA_VIEWPORT_H
#define CAMERA_VIEWPORT_H

#include "camera.h"

namespace Camera {

/** @brief A class containing viewport / camera information.  This uses OpenGL coordinate convention
 * in which x is to the right of the image, y is upward in the image, and z is backward. 
 */
class Viewport : public Camera
{
public:
	Viewport();
	///Sets the viewport as a perspective or orthographic viewport
	void setPerspective(bool enabled);
	///Sets the absolute field of view in the width direction (deprecated name)
	inline void setLensAngle(float rads) { setFOV(rads); }
	///Sets the absolute field of view in the width direction
	void setFOV(float rads);
	///Returns the absolute field of view in the width direction
	float getFOV() const;
	///Returns the absolute field of view in the height direction
	float getHFOV() const;

	///Returns true if the point (mx,my) is within the viewport
	bool clicked(int mx, int my) const;
	///Multiplies the camera's zoom by the factor s
	void zoom(float s);
	///Scrolls the viewport by the vector x,y in a screen-parallel
	///local camera frame.
	void scroll(float x, float y, float z=0);

	///Returns the forward direction in world coordinates 
	void getViewVector(Vector3&) const;
	///Returns a screen-parallel world-space vector for a screen-space
	///displacement of dx,dy
	void getMovementVector(float dx, float dy, Vector3&) const;
	///Returns a screen-parallel world-space vector that displaces a point
	///at distance dist by dx in the screen's x direction and dy in the y
	///direction
	void getMovementVectorAtDistance(float dx, float dy, float dist, Vector3&) const;
	
	///find the point/direction on the image plane corresponding to a
	///screen space point
	void getClickVector(float mx, float my, Vector3&) const;
	void getClickSource(float mx, float my, Vector3&) const;

	///Computes the screen coordinates of a point pt in world space.
	///Returns true if the point is within the view frustum.
	bool project(const Vector3& pt,float& mx,float& my,float& mz) const;

	bool perspective;
	float scale;
 
	int x,y,w,h;
	float n, f;
};

std::ostream& operator << (std::ostream& out, const Viewport&);
std::istream& operator >> (std::istream& in, Viewport&);

} //namespace Camera

#endif

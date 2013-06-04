#ifndef CAMERA_VIEWPORT_H
#define CAMERA_VIEWPORT_H

#include "camera.h"

namespace Camera {

class Viewport : public Camera
{
public:
	Viewport();
	void setPerspective(bool);
	void setLensAngle(float rads);

	bool clicked(int mx, int my) const;
	void zoom(float s);
	void scroll(float x, float y, float z=0);

	//returns the forward direction 
	void getViewVector(Vector3&) const;
	//returns a screen-parallel world-space vector for a screen-space
	//displacement of dx,dy
	void getMovementVector(float dx, float dy, Vector3&) const;
	//returns a screen-parallel world-space vector that displaces a point
	//at distance dist by dx in the screen's x direction and dy in the y
	//direction
	void getMovementVectorAtDistance(float dx, float dy, float dist, Vector3&) const;
	
	//find the point/direction on the image plane corresponding to a screen
	//space point
	void getClickVector(float mx, float my, Vector3&) const;
	void getClickSource(float mx, float my, Vector3&) const;

	//Computes the screen coordinates of a point. Returns true if the point
	//is within the view frustum.
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

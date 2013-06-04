#ifndef CAMERA_FRUSTUM_H
#define CAMERA_FRUSTUM_H

#include "clip.h"
#include "viewport.h"

namespace Camera {

struct Frustum : public ConvexVolume
{
	enum { Right=0,Left=1,Top=2,Bottom=3,Front=4,Back=5 };

	Frustum()
	{
		planes.resize(6);
	}

	void MakeFromViewport(const Viewport&);
	void MakeFromProjectionMatrix(const Matrix4&);
	void MakeFromViewMatrices(const Matrix4& modelview,const Matrix4& projection);
};

} //namespace Camera

#endif

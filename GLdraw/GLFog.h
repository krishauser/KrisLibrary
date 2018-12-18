#ifndef GL_FOG_H
#define GL_FOG_H

#include "GLColor.h"
#include <KrisLibrary/math/math.h>

namespace GLDraw {

using namespace Math;

class GLFog
{
public:
	enum Type { None, Linear, Exponential, ExponentialSquared };

	GLFog();

	void setNone();
	void setLinear(Real start, Real end);
	void setExponential(Real density);
	void setExponentialSquared(Real density);
	void setCurrent();
	void unsetCurrent();

	Type type;
	GLColor color;
	Real start,end;
	Real density;
};

} //namespace GLDraw

#endif

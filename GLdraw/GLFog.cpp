#include "GLFog.h"
#include "GL.h"

using namespace GLDraw;

GLFog::GLFog()
:type(None),start(0),end(1),density(1)
{
}

void GLFog::setCurrent()
{
#ifndef NO_OPENGL
	glFogfv(GL_FOG_COLOR, color.rgba);
	switch(type) {
	case None:
		glDisable(GL_FOG);
		break;
	case Linear:
		glEnable(GL_FOG);
		glFogi(GL_FOG_MODE, GL_LINEAR);
		glFogf(GL_FOG_START, start);
		glFogf(GL_FOG_END, end);
		break;
	case Exponential:
		glEnable(GL_FOG);
		glFogi(GL_FOG_MODE, GL_EXP);
		glFogf(GL_FOG_DENSITY, density);
		break;
	case ExponentialSquared:
		glEnable(GL_FOG);
		glFogi(GL_FOG_MODE, GL_EXP2);
		glFogf(GL_FOG_DENSITY, density);
		break;
	}
#endif //NO_OPENGL
}

void GLFog::unsetCurrent()
{
#ifndef NO_OPENGL
	glDisable(GL_FOG);
#endif //NO_OPENGL
}

void GLFog::setLinear(Real s, Real e)
{
	type=Linear;
	start=s;
	end=e;
}

void GLFog::setExponential(Real d)
{
	type=Exponential;
	density=d;
}

void GLFog::setExponentialSquared(Real d)
{
	type=ExponentialSquared;
	density=d;
}


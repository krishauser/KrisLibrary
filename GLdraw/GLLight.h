#ifndef GL_LIGHT_H
#define GL_LIGHT_H

#include "GLColor.h"
#include <KrisLibrary/math3d/primitives.h>

namespace GLDraw {

using namespace Math3D;

/** @ingroup GLDraw
 * @brief An OpenGL light.
 */
class GLLight
{
public:
	GLLight();
	GLLight(const Vector3& direction);
	GLLight(const Vector3& position,const Vector3& direction);
	GLLight(const GLLight& light);

	void setColor(const GLColor& col);
	void setPointLight(const Vector3& pos);
	void setDirectionalLight(const Vector3& dir);
	void setSpotLight(const Vector3& pos,const Vector3& dir,float exponent=0,float cutoff=180);

	void setCurrentGL(const int id=0);

	Vector4 position;
	float att2,att1,att0;	//quadratic,linear,constant attenuation
	GLColor diffuse, specular;
	Vector3 spot_direction;
	float spot_exponent,spot_cutoff;
};

} //namespace GLDraw

#endif

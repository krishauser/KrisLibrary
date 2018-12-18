#ifndef GL_MATERIAL_H
#define GL_MATERIAL_H

#include "GL.h"
#include "GLColor.h"

namespace GLDraw {

/** @ingroup GLDraw
 * @brief An OpenGL material.
 */
struct GLMaterial
{
	GLMaterial();
	void setCurrentGL(GLenum tgt = GL_FRONT) const;
	void setCurrentGL_Front() const;
	void setCurrentGL_Back() const;
	void setCurrentGL_FrontAndBack() const;

	GLColor ambient, diffuse, specular, emission;
	GLfloat specularExponent;
};

} //namespace GLDraw

#endif

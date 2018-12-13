#include "GLMaterial.h"

using namespace GLDraw;

GLMaterial::GLMaterial()
:specularExponent(0)
{}

void GLMaterial::setCurrentGL(GLenum tgt) const
{
	glMaterialfv(tgt, GL_AMBIENT, ambient.rgba);
	glMaterialfv(tgt, GL_DIFFUSE, diffuse.rgba);
	glMaterialfv(tgt, GL_SPECULAR, specular.rgba);
	glMaterialf(tgt, GL_SHININESS, specularExponent);
	glMaterialfv(tgt, GL_EMISSION, emission.rgba);
}


void GLMaterial::setCurrentGL_Front() const
{
	setCurrentGL(GL_FRONT);
}


void GLMaterial::setCurrentGL_Back() const
{
	setCurrentGL(GL_BACK);
}

void GLMaterial::setCurrentGL_FrontAndBack() const
{
	setCurrentGL(GL_FRONT_AND_BACK);
}

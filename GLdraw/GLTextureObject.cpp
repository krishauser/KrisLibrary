#include "GLTextureObject.h"
#include "GL.h"
#include <iostream>
using namespace std;

using namespace GLDraw;

GLTextureObject::GLTextureObject()
:glName(0)
{}

GLTextureObject::GLTextureObject(GLTextureObject& obj)
{
	stealObject(obj);
}

GLTextureObject::~GLTextureObject()
{
	cleanup();
}

bool GLTextureObject::isNull() const
{
	return glName == 0;
}

void GLTextureObject::stealObject(GLTextureObject& obj)
{
	cleanup();
	glName = obj.glName;
	obj.glName = 0;
}

void GLTextureObject::generate()
{
  if(glName == 0)
    glGenTextures(1, &glName);
  else
    cout<<"Warning, GLTextureObject.generate() called on a non-null object"<<endl;
}

void GLTextureObject::cleanup()
{
	if(glName != 0)
	{
		glDeleteTextures(1, &glName);
		glName = 0;
	}
}

void GLTextureObject::bind(unsigned int target) const
{
	glBindTexture(target,glName);
}

void GLTextureObject::unbind(unsigned int target) const
{
	glBindTexture(target,0);
}

#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "GLTextureObject.h"
#include "GL.h"
#include <iostream>
using namespace std;

using namespace GLDraw;

GLTextureObject::GLTextureObject()
{}

GLTextureObject::GLTextureObject(const GLTextureObject& obj)
{
  glName = obj.glName;
}

GLTextureObject::~GLTextureObject()
{
  cleanup();
}

bool GLTextureObject::isNull() const
{
	return glName == NULL;
}

void GLTextureObject::generate()
{
  if(glName == 0) {
    glName = new unsigned int;
    glGenTextures(1, glName);
  }
  else
    LOG4CXX_WARN(logger,"Warning, GLTextureObject.generate() called on a non-null object"<<"\n");
}

void GLTextureObject::cleanup()
{

  if(glName && glName.getRefCount()==1) {
    glDeleteTextures(1, glName);
  }
  glName = 0;
}

void GLTextureObject::bind(unsigned int target) const
{
  if(glName) {
    glBindTexture(target,*glName);
  }
}

void GLTextureObject::unbind(unsigned int target) const
{
  if(glName) {
    glBindTexture(target,0);
  }
}

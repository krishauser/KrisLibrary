#include <KrisLibrary/Logger.h>
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
    glName.reset(new unsigned int);
    glGenTextures(1, glName.get());
  }
  else
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, GLTextureObject.generate() called on a non-null object");
}

void GLTextureObject::cleanup()
{

  if(glName && glName.use_count()==1) {
    glDeleteTextures(1, glName.get());
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

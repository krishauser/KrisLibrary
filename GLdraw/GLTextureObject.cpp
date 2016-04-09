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
#ifndef NO_OPENGL
  if(glName == 0) {
    glName = new unsigned int;
    glGenTextures(1, glName);
  }
  else
    cout<<"Warning, GLTextureObject.generate() called on a non-null object"<<endl;
#endif //NO_OPENGL
}

void GLTextureObject::cleanup()
{
#ifndef NO_OPENGL
  if(glName && glName.getRefCount()==1) {
    glDeleteTextures(1, glName);
  }
  glName = 0;
#endif //NO_OPENGL
}

void GLTextureObject::bind(unsigned int target) const
{
#ifndef NO_OPENGL
  if(glName) {
    glBindTexture(target,*glName);
  }
#endif //NO_OPENGL
}

void GLTextureObject::unbind(unsigned int target) const
{
#ifndef NO_OPENGL
  if(glName) {
    glBindTexture(target,0);
  }
#endif //NO_OPENGL
}

#include "GLTexture2D.h"
#include "GL.h"

using namespace GLDraw;

GLTexture2D::GLTexture2D()
{
}

void GLTexture2D::setLuminance(const unsigned char* data,int m,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_2D);
  glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,m,n,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setRGB(const unsigned char* data,int m,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_2D);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,m,n,0,GL_RGB,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setRGBA(const unsigned char* data,int m,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_2D);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,m,n,0,GL_RGBA,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setAlpha(const unsigned char* data,int m,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_2D);
  glTexImage2D(GL_TEXTURE_2D,0,GL_ALPHA,m,n,0,GL_ALPHA,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setFilterNearest()
{
  texObj.bind(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setFilterLinear()
{
  texObj.bind(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setWrapClamp()
{
  texObj.bind(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setWrapRepeat()
{
  texObj.bind(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_2D);
}

void GLTexture2D::setCurrentGL()
{
  texObj.bind(GL_TEXTURE_2D);
}

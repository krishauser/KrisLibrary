#include "GLTexture1D.h"
#include "GL.h"

using namespace GLDraw;

#define FLOAT_TO_UCHAR(x) (unsigned char)(x*255.0)

GLTexture1D::GLTexture1D()
{
}

void GLTexture1D::setLuminance(const ColorGradient& grad,int n)
{
  unsigned char* buf = new unsigned char[n];
  for(int i=0;i<n;i++) {
    float u = float(i)/float(n-1);
    GLColor c;
    grad.Eval(u,c);
    buf[i] = FLOAT_TO_UCHAR(c.getLuminance());
  }
  setLuminance(buf,n);
  delete [] buf;
}

void GLTexture1D::setRGB(const ColorGradient& grad,int n)
{
  unsigned char* buf = new unsigned char[n*3];
  for(int i=0;i<n;i++) {
    float u = float(i)/float(n-1);
    GLColor c;
    grad.Eval(u,c);
    buf[i*3] = FLOAT_TO_UCHAR(c.rgba[0]);
    buf[i*3+1] = FLOAT_TO_UCHAR(c.rgba[1]);
    buf[i*3+2] = FLOAT_TO_UCHAR(c.rgba[2]);
  }
  setRGB(buf,n);
  delete [] buf;
}

void GLTexture1D::setRGBA(const ColorGradient& grad,int n)
{
  unsigned char* buf = new unsigned char[n*4];
  for(int i=0;i<n;i++) {
    float u = float(i)/float(n-1);
    GLColor c;
    grad.Eval(u,c);
    buf[i*4] = FLOAT_TO_UCHAR(c.rgba[0]);
    buf[i*4+1] = FLOAT_TO_UCHAR(c.rgba[1]);
    buf[i*4+2] = FLOAT_TO_UCHAR(c.rgba[2]);
    buf[i*4+3] = FLOAT_TO_UCHAR(c.rgba[3]);
  }
  setRGBA(buf,n);
  delete [] buf;
}

void GLTexture1D::setAlpha(const ColorGradient& grad,int n)
{
  unsigned char* buf = new unsigned char[n];
  for(int i=0;i<n;i++) {
    float u = float(i)/float(n-1);
    GLColor c;
    grad.Eval(u,c);
    buf[i] = FLOAT_TO_UCHAR(c.rgba[3]);
  }
  setAlpha(buf,n);
  delete [] buf;
}

void GLTexture1D::setLuminance(const unsigned char* data,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_1D);
  glTexImage1D(GL_TEXTURE_1D,0,GL_LUMINANCE,n,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setRGB(const unsigned char* data,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_1D);
  glTexImage1D(GL_TEXTURE_1D,0,GL_RGB8,n,0,GL_RGB,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setRGBA(const unsigned char* data,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_1D);
  glTexImage1D(GL_TEXTURE_1D,0,GL_RGBA8,n,0,GL_RGBA,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setAlpha(const unsigned char* data,int n)
{
  if(texObj.isNull()) texObj.generate();
  texObj.bind(GL_TEXTURE_1D);
  glTexImage1D(GL_TEXTURE_1D,0,GL_ALPHA,n,0,GL_ALPHA,GL_UNSIGNED_BYTE,data);  
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setFilterNearest()
{
  texObj.bind(GL_TEXTURE_1D);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setFilterLinear()
{
  texObj.bind(GL_TEXTURE_1D);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setWrapClamp()
{
  texObj.bind(GL_TEXTURE_1D);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setWrapRepeat()
{
  texObj.bind(GL_TEXTURE_1D);
  glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  texObj.unbind(GL_TEXTURE_1D);
}

void GLTexture1D::setCurrentGL()
{
  texObj.bind(GL_TEXTURE_1D);
}

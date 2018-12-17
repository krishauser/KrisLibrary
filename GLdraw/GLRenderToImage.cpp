#include "GLRenderToImage.h"
#include "GLView.h"
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/errors.h>
#include <memory.h>
#if HAVE_GLEW
#include <GL/glew.h>
#else
#include "GL.h"
#endif //HAVE_GLEW
using namespace std;
using namespace GLDraw;

#if HAVE_GLEW
  #ifndef GL_BGRA
  #ifndef GL_BGRA_EXT
  #error "GL_BGRA is not defined on your system?"
  #endif //GL_BGRA_EXT
  #define GL_BGRA GL_BGRA_EXT
  #endif //GL_BGRA
#endif // HAVE_GLEW


GLRenderToImage::GLRenderToImage()
  :width(0),height(0),color_tex(0),fb(0),depth_rb(0)
{}

GLRenderToImage::~GLRenderToImage()
{
  if(color_tex) glDeleteTextures(1, &color_tex);
#if HAVE_GLEW
  if(depth_rb) glDeleteRenderbuffersEXT(1, &depth_rb);
  if(fb) glDeleteFramebuffersEXT(1, &fb);
#endif //HAVE_GLEW
  color_tex = 0;
  depth_rb = 0;
  fb = 0;
}

bool GLRenderToImage::Setup(int w,int h)
{
#if HAVE_GLEW
  if(!GLEW_EXT_framebuffer_object) {
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
      glewExperimental=GL_TRUE;
      err = glewInit(); 
      if (GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
        LOG4CXX_WARN(KrisLibrary::logger(),"GLRenderToImage::glewInit() error: "<<glewGetErrorString(err));
        LOG4CXX_WARN(KrisLibrary::logger(),"  This usually happens when an OpenGL context has not been initialized.");
        return false;
      }
      
    }
    if(!GLEW_EXT_framebuffer_object) {
      return false;
    }
  }
  width = w;
  height = h;
  if(color_tex == 0) { 
    //RGBA8 2D texture, 24 bit depth texture, 256x256
    glGenTextures(1, &color_tex);
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //NULL means reserve texture memory, but texels are undefined
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
  }
  if(fb == 0) {
    //-------------------------
    glGenFramebuffersEXT(1, &fb);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
    //Attach 2D texture to this FBO
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, color_tex, 0);
  }
  if(depth_rb == 0) {
    //-------------------------
    glGenRenderbuffersEXT(1, &depth_rb);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, w, h);
    //-------------------------
    //Attach depth buffer to FBO
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);
  }
  //-------------------------
  //Does the GPU support current FBO configuration?
  GLenum status;
  status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  switch(status)
  {
  case GL_FRAMEBUFFER_COMPLETE_EXT:
    break;
  default:
    //Delete resources
    glDeleteTextures(1, &color_tex);
    glDeleteRenderbuffersEXT(1, &depth_rb);
    //Bind 0, which means render to back buffer, as a result, fb is unbound
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    glDeleteFramebuffersEXT(1, &fb);
    color_tex = 0;
    depth_rb = 0;
    fb = 0;
    return false;
  }
  return true;
#else
  return false;
#endif //HAVE_GLEW
}

void GLRenderToImage::Begin(const Camera::Viewport& vp)
{
  Begin();
  GLView view;
  view.setViewport(vp);
  view.setCurrentGL();
}

void GLRenderToImage::Begin()
{
  if(!fb) return;
#if HAVE_GLEW
  //and now you can render to GL_TEXTURE_2D
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif
}

void GLRenderToImage::End()
{
#if HAVE_GLEW
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
#endif 
}

void GLRenderToImage::GetRGBA(vector<unsigned int>& image)
{
  image.resize(width*height);
  glBindTexture(GL_TEXTURE_2D, color_tex);
  glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_INT_8_8_8_8,&image[0]);
  //OpenGL images start in lower left, so flip vertically
  int rowsize = 4*width;
  int stride = width;
  vector<unsigned char> temp(rowsize);
  for(int i=0;i<height/2;i++) {
    int iflip = height-1-i;
    memcpy(&temp[0],&image[i*stride],rowsize);
    memcpy(&image[i*stride],&image[iflip*stride],rowsize);
    memcpy(&image[iflip*stride],&temp[0],rowsize);
  }
  for(int i=0;i<width*height;i++) {
    unsigned char* argb = reinterpret_cast<unsigned char*>(&image[i]);
    unsigned char a=argb[0],b=argb[1],g=argb[2],r=argb[3];
    argb[0] = b;
    argb[1] = g;
    argb[2] = r;
    argb[3] = a;
  }
}
void GLRenderToImage::GetRGBA(vector<unsigned char>& image)
{
  image.resize(4*width*height);
  glBindTexture(GL_TEXTURE_2D, color_tex);
  glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_INT_8_8_8_8,&image[0]);
  //OpenGL images start in lower left, so flip vertically
  int rowsize = 4*width;
  int stride = 4*width;
  vector<unsigned char> temp(rowsize);
  for(int i=0;i<height/2;i++) {
    int iflip = height-1-i;
    memcpy(&temp[0],&image[i*stride],rowsize);
    memcpy(&image[i*stride],&image[iflip*stride],rowsize);
    memcpy(&image[iflip*stride],&temp[0],rowsize);
  }
}

void GLRenderToImage::GetRGBA(vector<vector<unsigned int> >& image)
{
  vector<unsigned int> pix;
  GetRGBA(pix);
  image.resize(height);
  for(int i=0;i<height;i++) {
    image[i].resize(width);
    copy(pix.begin()+i*width,pix.begin()+(i+1)*width,image[i].begin());
  }
}

void GLRenderToImage::GetRGBA(Image& image)
{
  image.initialize(width,height,Image::A8R8G8B8);
  vector<unsigned char> bytes;
  GetRGBA(bytes);
  Assert(bytes.size() == image.num_bytes);
  memcpy(image.data,&bytes[0],image.num_bytes);
  //switch the red and the blue
  for(int i=0;i<width*height;i++) {
    unsigned char* argb = image.data + i*4;
    unsigned char a=argb[0],r=argb[1],g=argb[2],b=argb[3];
    argb[0] = b;
    argb[1] = g;
    argb[2] = r;
    argb[3] = a;
  }
}

void GLRenderToImage::GetZBuffer(vector<float>& image)
{
#if HAVE_GLEW
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
  image.resize(width*height);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &image[0]);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 

  int rowsize = sizeof(float)*width;
  int stride = width;
  vector<unsigned char> temp(rowsize);
  for(int i=0;i<height/2;i++) {
    int iflip = height-1-i;
    memcpy(&temp[0],&image[i*stride],rowsize);
    memcpy(&image[i*stride],&image[iflip*stride],rowsize);
    memcpy(&image[iflip*stride],&temp[0],rowsize);
  }
#endif 
}

void GLRenderToImage::GetDepth(const Camera::Viewport& vp,vector<float>& image)
{
  GetZBuffer(image);
  //don't forget to flip vertically
  Real zmininv = 1.0/vp.n;
  Real zscale = (1.0/vp.n-1.0/vp.f);
  //nonlinear depth normalization
  //normal linear interpolation would give u = (z - zmin)/(zmax-zmin)
  //instead we gt u = (1/zmin-1/z)/(1/zmin-1/zmax)
  //so 1/z = 1/zmin - u(1/zmin-1/zmax)
  for(size_t i=0;i<image.size();i++) {
    if(image[i] == 1.0) { //nothing seen
      image[i] = vp.f;
    }
    else {
      image[i] = float(1.0/(zmininv - image[i]*zscale));
    }
  }
}

void GLRenderToImage::GetDepth(const Camera::Viewport& vp,vector<vector<float> >& image)
{
  vector<float> pix;
  GetDepth(vp,pix);
  image.resize(height);
  for(int i=0;i<height;i++) {
    image[i].resize(width);
    copy(pix.begin()+i*width,pix.begin()+(i+1)*width,image[i].begin());
  }
}

void GLRenderToImage::GetDepth(const Camera::Viewport& vp,Image& image)
{
  image.initialize(width,height,Image::FloatA);
  vector<float> pix;
  GetDepth(vp,pix);
  Assert(pix.size()*sizeof(float) == image.num_bytes);
  memcpy(image.data,&pix[0],image.num_bytes);
}

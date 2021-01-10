#include "GLRenderToImage.h"
#include "GLView.h"
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/Timer.h>
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

#define DEBUG_TIMING 0

//whether to use the EXT version or the ARB version
static bool use_ext = false;

GLRenderToImage::GLRenderToImage()
  :width(0),height(0),fb(0),color_tex(0),color_rb(0),depth_tex(0),depth_rb(0)
{}

GLRenderToImage::~GLRenderToImage()
{
  if(color_tex) glDeleteTextures(1, &color_tex);
  if(depth_tex) glDeleteTextures(1, &depth_tex);
#if HAVE_GLEW
  if(use_ext) {
    if(color_rb) glDeleteRenderbuffersEXT(1, &color_rb);
    if(depth_rb) glDeleteRenderbuffersEXT(1, &depth_rb);
    if(fb) glDeleteFramebuffersEXT(1, &fb);
  }
  else {
    if(color_rb) glDeleteRenderbuffers(1, &color_rb);
    if(depth_rb) glDeleteRenderbuffers(1, &depth_rb);
    if(fb) glDeleteFramebuffers(1, &fb);
  }
#endif //HAVE_GLEW
  color_tex = 0;
  color_rb = 0;
  depth_tex = 0;
  depth_rb = 0;
  fb = 0;
}

bool GLRenderToImage::Setup(int w,int h,bool want_color_tex,bool want_depth_tex)
{
#if HAVE_GLEW
  if(!GLEW_ARB_framebuffer_object && !GLEW_EXT_framebuffer_object) {
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
    if(!GLEW_ARB_framebuffer_object && !GLEW_EXT_framebuffer_object) {
      LOG4CXX_WARN(KrisLibrary::logger(),"GLRenderToImage: GLEW finds that framebuffer objects not supported.");
      return false;
    }
    use_ext = !GLEW_ARB_framebuffer_object;
  }
  width = w;
  height = h;
  if(fb == 0) {
    if(use_ext) {
      glGenFramebuffersEXT(1, &fb);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
    }
    else {
      glGenFramebuffers(1, &fb);
      glBindFramebuffer(GL_FRAMEBUFFER, fb);
    }
  }
  if(want_color_tex) {
    if(color_tex == 0) { 
      //RGBA8 2D texture, 24 bit depth texture, 256x256
      glGenTextures(1, &color_tex);
      glBindTexture(GL_TEXTURE_2D, color_tex);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      //NULL means reserve texture memory, but texels are undefined
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      glBindTexture(GL_TEXTURE_2D, 0);
      //Attach 2D texture to the FBO
      if(use_ext) 
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, color_tex, 0);
      else
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_tex, 0);
    }
  }
  else {
    if(color_rb == 0) {
      if(use_ext) {
        glGenRenderbuffersEXT(1, &color_rb);
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, color_rb);
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, w, h);
        //Attach colorbuffer to FBO
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, color_rb);
      }
      else {
        glGenRenderbuffers(1, &color_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, color_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h);
        //Attach colorbuffer to FBO
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rb);
      }
    }
  }

  if(want_depth_tex) {
    if(depth_tex == 0) {
      //RGBA8 2D texture, 24 bit depth texture, 256x256
      glGenTextures(1, &depth_tex);
      glBindTexture(GL_TEXTURE_2D, depth_tex);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      //NULL means reserve texture memory, but texels are undefined
      glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
      glBindTexture(GL_TEXTURE_2D, 0);
      //Attach 2D texture to the FBO
      if(use_ext) 
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, depth_tex, 0);
      else
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_tex, 0);
    }
  }
  else {
    if(depth_rb == 0) {
      if(use_ext) {
        //-------------------------
        glGenRenderbuffersEXT(1, &depth_rb);
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, w, h);
        //-------------------------
        //Attach depth buffer to FBO
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);
      }
      else {
        glGenRenderbuffers(1, &depth_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rb);
      }
    }
  }
  //-------------------------
  //Does the GPU support current FBO configuration?
  GLenum status;
  if(use_ext)
    status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  else
    status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  switch(status)
  {
  case GL_FRAMEBUFFER_COMPLETE_EXT:
    break;
  default:
    //Delete resources
    if(color_tex)
      glDeleteTextures(1, &color_tex);
    if(depth_tex)
      glDeleteTextures(1, &depth_tex);
    if(use_ext) {
      //Bind 0, which means render to back buffer, as a result, fb is unbound
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
      if(color_rb) glDeleteRenderbuffersEXT(1, &color_rb);
      if(depth_rb) glDeleteRenderbuffersEXT(1, &depth_rb);
      glDeleteFramebuffersEXT(1, &fb);
    }
    else {
      //Bind 0, which means render to back buffer, as a result, fb is unbound
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
      if(color_rb) glDeleteRenderbuffers(1, &color_rb);
      if(depth_rb) glDeleteRenderbuffers(1, &depth_rb);
      glDeleteFramebuffers(1, &fb);
    }
    color_tex = 0;
    color_rb = 0;
    depth_tex = 0;
    depth_rb = 0;
    fb = 0;
    LOG4CXX_WARN(KrisLibrary::logger(),"GLRenderToImage: Some error setting up the framebuffer?");
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

void GLRenderToImage::Begin(const GLView& vp)
{
  Begin();
  vp.setCurrentGL();
}


void GLRenderToImage::Begin()
{
  if(!fb) return;
#if HAVE_GLEW
  //and now you can render to GL_TEXTURE_2D
  if(use_ext)
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
  else
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif
  GLenum err;
  while ((err = glGetError()) != GL_NO_ERROR) {
      printf("GLRenderToImage::Begin(): OpenGL error: %d\n",err);
  }
}

void GLRenderToImage::End()
{
#if HAVE_GLEW
  if(use_ext)
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
  else
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif 
  GLenum err;
  while ((err = glGetError()) != GL_NO_ERROR) {
      printf("GLRenderToImage::End(): OpenGL error: %d\n",err);
  }
}

void GLRenderToImage::GetRGBA(vector<unsigned int>& image)
{
  image.resize(width*height);
  if(color_tex) {
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_INT_8_8_8_8,&image[0]);
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else if(color_rb) {
    if(use_ext) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8, &image[0]);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    }
    else {
      glBindFramebuffer(GL_FRAMEBUFFER, fb); 
      glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8, &image[0]);
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
    }
  }
  else {
    return;
  }
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
  if(color_tex) {
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_INT_8_8_8_8,&image[0]);
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else if(color_rb) {
    if(use_ext) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8, &image[0]);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    }
    else {
      glBindFramebuffer(GL_FRAMEBUFFER, fb); 
      glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8, &image[0]);
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
    }
  }
  else {
    return;
  }
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
  if(width != image.w || height != image.h || Image::R8G8B8A8 != image.format)
    image.initialize(width,height,Image::R8G8B8A8);
  vector<unsigned char> bytes;
  GetRGBA(bytes);
  Assert(bytes.size() == image.num_bytes);
  memcpy(image.data,&bytes[0],image.num_bytes);
}

void GLRenderToImage::GetRGB(vector<unsigned char>& image)
{
  image.resize(3*width*height);
  if(color_tex) {
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_UNSIGNED_BYTE,&image[0]);
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else if(color_rb) {
    if(use_ext) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &image[0]);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    }
    else {
      glBindFramebuffer(GL_FRAMEBUFFER, fb); 
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &image[0]);
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
    }
  }
  else {
    image.resize(0);
    return;
  }
  //OpenGL images start in lower left, so flip vertically
  int rowsize = 3*width;
  int stride = 3*width;
  vector<unsigned char> temp(rowsize);
  for(int i=0;i<height/2;i++) {
    int iflip = height-1-i;
    memcpy(&temp[0],&image[i*stride],rowsize);
    memcpy(&image[i*stride],&image[iflip*stride],rowsize);
    memcpy(&image[iflip*stride],&temp[0],rowsize);
  }
}

void GLRenderToImage::GetRGB(Image& image)
{
  if(width != image.w || height != image.h || Image::R8G8B8 != image.format)
    image.initialize(width,height,Image::R8G8B8);
  /*
  vector<unsigned char> bytes;
  GetRGB(bytes);
  Assert(bytes.size() == image.num_bytes);
  memcpy(image.data,&bytes[0],image.num_bytes);
  */
  if(color_tex) {
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_UNSIGNED_BYTE,image.data);
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else if(color_rb) {
    if(use_ext) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image.data);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    }
    else {
      glBindFramebuffer(GL_FRAMEBUFFER, fb); 
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image.data);
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
    }
  }
  else {
    return;
  }
  //OpenGL images start in lower left, so flip vertically
  int rowsize = 3*width;
  int stride = 3*width;
  vector<unsigned char> temp(rowsize);
  for(int i=0;i<height/2;i++) {
    int iflip = height-1-i;
    memcpy(&temp[0],&image.data[i*stride],rowsize);
    memcpy(&image.data[i*stride],&image.data[iflip*stride],rowsize);
    memcpy(&image.data[iflip*stride],&temp[0],rowsize);
  }
}


void GLRenderToImage::GetZBuffer(vector<float>& image)
{
#if HAVE_GLEW
  #if DEBUG_TIMING
    Timer timer;
  #endif //DEBUG_TIMING
  image.resize(width*height);
  if(depth_tex) {
    glBindTexture(GL_TEXTURE_2D, depth_tex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT,GL_FLOAT,&image[0]);
    glBindTexture(GL_TEXTURE_2D,0);
  }
  else if(depth_rb) {
    if(use_ext) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &image[0]);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
    }
    else {
      glBindFramebuffer(GL_FRAMEBUFFER, fb); 
      glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &image[0]);
      glBindFramebuffer(GL_FRAMEBUFFER, 0); 
    }  
  }
  #if DEBUG_TIMING
    printf("glReadPixels time %f\n",timer.ElapsedTime());
  #endif //DEBUG_TIMING

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
  #if DEBUG_TIMING
    Timer timer;
  #endif //DEBUG_TIMING
  float num = vp.n*vp.f;
  float dist = vp.f-vp.n;
  for(size_t i=0;i<image.size();i++) {
    image[i] = num/(vp.f - image[i]*dist);
  }
  #if DEBUG_TIMING
    printf("Zbuffer to depth conversion %f\n",timer.ElapsedTime());
  #endif //DEBUG_TIMING
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
  if(width != image.w || height != image.h || Image::FloatA != image.format)
    image.initialize(width,height,Image::FloatA);
  vector<float> pix;
  GetDepth(vp,pix);
  Assert(pix.size()*sizeof(float) == image.num_bytes);
  memcpy(image.data,&pix[0],image.num_bytes);
}

void GLRenderToImage::GetDepth(const GLView& vp,vector<float>& image)
{
  Camera::Viewport view;
  vp.getViewport(view);
  GetDepth(view,image);
}

void GLRenderToImage::GetDepth(const GLView& vp,vector<vector<float> >& image)
{
  Camera::Viewport view;
  vp.getViewport(view);
  GetDepth(view,image);
}

void GLRenderToImage::GetDepth(const GLView& vp,Image& image)
{
  Camera::Viewport view;
  vp.getViewport(view);
  GetDepth(view,image);
}

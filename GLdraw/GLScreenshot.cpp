#include "GLScreenshot.h"
#include "GL.h"
#include "GLError.h"
#include <stdio.h>
//#define GDI_AVAILABLE
#if defined WIN32 && defined GDI_AVAILABLE
#include <ole2.h>
#include <image/gdi.h>
#endif //WIN32 && GDI_AVAILABLE
#include <image/ppm.h>

#include <memory.h>

void flipRGBImage(unsigned char* image,int width,int height)
{
  int stride = width*3;
  unsigned char* row = new unsigned char[stride];
  for(int i=0;i<height/2;i++) {
    //swap rows i,height-i
    memcpy(row,image+i*stride,stride);
    memcpy(image+i*stride,image+(height-i-1)*stride,stride);
    memcpy(image+(height-i-1)*stride,row,stride);
  }
  delete [] row;
}

void GLSaveScreenshot(const char* filename)
{
#if (defined WIN32 && defined GDI_AVAILABLE)
  // These are important to get screen captures to work correctly
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT,vp);
  int x=vp[0];
  int y=vp[1];
  int width = vp[2];
  int height = vp[3];

  //row-major array, bottom corner is first row
  Image image;
  image.initialize(width,height,Image::R8G8B8);
  //glReadBuffer(GL_FRONT);
  glReadBuffer(GL_BACK);
  glReadPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE,image.data);
  bool errors = CheckGLErrors("SaveScreenshot",false);
  if(errors) return;
  //printf("Flipping RGB image...\n");
  flipRGBImage(image.data,width,height);
  //printf("Done, now exporting...\n");
  ExportImageGDIPlus(filename,image);
  //printf("Done, saving screenshot.\n");
  /*
  ImageOperator op(image);
  ImageOperator op2;
  op2.initialize(640,480);
  op.stretchBlitBilinear(op2);
  Image image2;
  op2.output(image2,Image::R8G8B8);
  ExportImageGDIPlus(filename,image2);
  */
#else
  fprintf(stderr,"Warning, saving screenshot in PPM format...\n");
  GLSaveScreenshotPPM(filename);
#endif
}

void GLSaveScreenshotPPM(const char* filename)
{
  // These are important to get screen captures to work correctly
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT,vp);
  int x=vp[0];
  int y=vp[1];
  int width = vp[2];
  int height = vp[3];

  unsigned char* data = new unsigned char[width*height*3];
  glReadBuffer(GL_BACK);
  glReadPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE,data);
  flipRGBImage(data,width,height);
  WritePPM_RGB_Binary(data,width,height,filename);
  delete [] data;
}


#ifndef GLDRAW_GLUT_STRING_H
#define GLDRAW_GLUT_STRING_H

#if HAVE_GLUT

#if defined (__APPLE__) || defined (MACOSX)
#include <KrisLibrary/Logger.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

inline void glutBitmapString(void* fontface,const char* str)
{
  while(*str) {
    glutBitmapCharacter(fontface,*str);
    str++;
  }
}

inline void glutStrokeString(void* fontface,const char* str)
{
  while(*str) {
    glutStrokeCharacter(fontface,*str);
    str++;
  }
}

inline int glutBitmapStringWidth(void* fontface,const char* str)
{
  int w=0;
  while(*str) {
    w += glutBitmapWidth(fontface,*str);
    str++;
  }
  return w;
}

inline int glutStrokeStringWidth(void* fontface,const char* str)
{
  int w=0;
  while(*str) {
    w += glutStrokeWidth(fontface,*str);
    str++;
  }
  return w;
}

#else

inline void glutBitmapString(void* fontface,const char* str)
{}

inline void glutStrokeString(void* fontface,const char* str)
{}

inline int glutBitmapStringWidth(void* fontface,const char* str)
{
  return 0;
}

inline int glutStrokeStringWidth(void* fontface,const char* str)
{
  return 0;
}


#endif //HAVE_GLUT


inline void glutBitmapInt(void* fontface,int i)
{
  char buf[64];
  snprintf(buf,64,"%d",i);
  glutBitmapString(fontface,buf);
}

inline void glutStrokeInt(void* fontface,int i)
{
  char buf[64];
  snprintf(buf,64,"%d",i);
  glutStrokeString(fontface,buf);
}

inline int glutBitmapIntWidth(void* fontface,int i)
{
  char buf[64];
  snprintf(buf,64,"%d",i);
  return glutBitmapStringWidth(fontface,buf);
}

inline int glutStrokeIntWidth(void* fontface,int i)
{
  char buf[64];
  snprintf(buf,64,"%d",i);
  return glutStrokeStringWidth(fontface,buf);
}

inline void glutBitmapFloat(void* fontface,float i)
{
  char buf[64];
  snprintf(buf,64,"%.2f",i);
  glutBitmapString(fontface,buf);
}

inline void glutStrokeFloat(void* fontface,float i)
{
  char buf[64];
  snprintf(buf,64,"%.2f",i);
  glutStrokeString(fontface,buf);
}

inline int glutBitmapFloatWidth(void* fontface,float i)
{
  char buf[64];
  snprintf(buf,64,"%.2f",i);
  return glutBitmapStringWidth(fontface,buf);
}

inline int glutStrokeFloatWidth(void* fontface,float i)
{
  char buf[64];
  snprintf(buf,64,"%.2f",i);
  return glutStrokeStringWidth(fontface,buf);
}

inline void glutBitmapDouble(void* fontface,double i)
{
  char buf[64];
  snprintf(buf,64,"%.2g",i);
  glutBitmapString(fontface,buf);
}

inline void glutStrokeDouble(void* fontface,double i)
{
  char buf[64];
  snprintf(buf,64,"%.2g",i);
  glutStrokeString(fontface,buf);
}

inline int glutBitmapDoubleWidth(void* fontface,double i)
{
  char buf[64];
  snprintf(buf,64,"%.2g",i);
  return glutBitmapStringWidth(fontface,buf);
}

inline int glutStrokeDoubleWidth(void* fontface,double i)
{
  char buf[64];
  snprintf(buf,64,"%.2g",i);
  return glutStrokeStringWidth(fontface,buf);
}

#endif

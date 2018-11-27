#include <KrisLibrary/Logger.h>
#include "GLError.h"
#include <iostream>
#include <stdio.h>
using namespace std;

const char* GLErrorString(GLenum err)
{
  switch(err) {
  case GL_NO_ERROR: return "GL_NO_ERROR";
  case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
  case GL_INVALID_VALUE: return "GL_INVALID_VALUE"; 
  case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
  case GL_STACK_OVERFLOW: return "GL_STACK_OVERFLOW";
  case GL_STACK_UNDERFLOW: return "GL_STACK_UNDERFLOW";
  case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
    //case GL_TABLE_TOO_LARGE: return "GL_TABLE_TOO_LARGE";
  default: return "GLErrorString(): invalid error code";
  }
}

bool CheckGLErrors(const char* name,bool pause)
{
  bool res=false;
  GLenum err;
  while((err=glGetError()) != GL_NO_ERROR) {
    LOG4CXX_ERROR(KrisLibrary::logger(),name<<" "<<GLErrorString(err));
    res=true;
  }
  if(res&&pause)
    KrisLibrary::loggerWait();
  return res;
}


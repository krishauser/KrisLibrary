#ifndef GLDRAW_GL_ERROR_H
#define GLDRAW_GL_ERROR_H

#include "GL.h"

#ifndef NO_OPENGL

//Returns the string corresponding to the GL error code err
const char* GLErrorString(GLenum err);

//Checks for GL errors, prints them with the header string name,
//pauses if pause=true.  Returns false if no error occurred.
bool CheckGLErrors(const char* name="GL error",bool pause=true);

#define GL_ERROR_QUIT 0
#define DEBUG_GL_ERRORS() { \
  GLenum err; \
  while((err=glGetError()) != GL_NO_ERROR) { \
    fprintf(stderr,"glError %s found at %s:%u\n",GLErrorString(err),__FILE__,__LINE__); \
    if(GL_ERROR_QUIT) exit(1); \
  } \
}

#endif //NO_OPENGL

#endif

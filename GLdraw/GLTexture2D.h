#ifndef GL_TEXTURE_2D_H
#define GL_TEXTURE_2D_H

#include "GLTextureObject.h"

namespace GLDraw {

/** @ingroup GLDraw
 * @brief 2D texture data for use in OpenGL.
 */
class GLTexture2D
{
 public:
  GLTexture2D();
  void setLuminance(const unsigned char* data,int m,int n);
  void setRGB(const unsigned char* data,int m,int n);
  void setRGBA(const unsigned char* data,int m,int n);
  void setAlpha(const unsigned char* data,int m,int n);
  void setFilterLinear();
  void setFilterNearest();
  void setWrapClamp();
  void setWrapRepeat();
  void setCurrentGL();

  GLTextureObject texObj;
};

} //namespace GLDraw

#endif


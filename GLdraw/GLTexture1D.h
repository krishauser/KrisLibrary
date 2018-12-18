#ifndef GL_TEXTURE_1D_H
#define GL_TEXTURE_1D_H

#include "GLTextureObject.h"
#include "ColorGradient.h"

namespace GLDraw {

/** @ingroup GLDraw
 * @brief 1D texture data for use in OpenGL.
 */
class GLTexture1D
{
 public:
  GLTexture1D();
  void setLuminance(const unsigned char* data,int n);
  void setRGB(const unsigned char* data,int n);
  void setRGBA(const unsigned char* data,int n);
  void setAlpha(const unsigned char* data,int n);
  void setLuminance(const ColorGradient& grad,int n);
  void setRGB(const ColorGradient& grad,int n);
  void setRGBA(const ColorGradient& grad,int n);
  void setAlpha(const ColorGradient& grad,int n);
  void setFilterLinear();
  void setFilterNearest();
  void setWrapClamp();
  void setWrapRepeat();
  void setCurrentGL();

  GLTextureObject texObj;
};

} //namespace GLDraw

#endif


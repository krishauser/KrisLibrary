#ifndef COLOR_GRADIENT_H
#define COLOR_GRADIENT_H

#include "GLColor.h"
#include <vector>

namespace GLDraw {

struct ColorGradient
{
  ColorGradient();
  ColorGradient(const GLColor& a);
  ColorGradient(const GLColor& a,const GLColor& b);
  void SetConstant(const GLColor& c);
  void SetBlend(const GLColor& a,const GLColor& b);
  void SetHue(float hue);
  void SetAlpha(float alpha);
  void Eval(float u,GLColor& c) const;

  std::vector<float> params;
  std::vector<GLColor> colors;
};

} //namespace GLDraw

#endif


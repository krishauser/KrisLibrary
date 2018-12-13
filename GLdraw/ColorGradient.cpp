#include "ColorGradient.h"
#include <stdlib.h>

using namespace GLDraw;

ColorGradient::ColorGradient()
{}

ColorGradient::ColorGradient(const GLColor& a)
{
  SetConstant(a);
}

ColorGradient::ColorGradient(const GLColor& a,const GLColor& b)
{
  SetBlend(a,b);
}

void ColorGradient::SetConstant(const GLColor& c)
{
  params.resize(1);
  colors.resize(1);
  params[0] = 0;
  colors[0] = c;
}

void ColorGradient::SetBlend(const GLColor& a,const GLColor& b)
{
  params.resize(2);
  colors.resize(2);
  params[0] = 0;
  colors[0] = a;
  params[1] = 1;
  colors[1] = b;
}

void ColorGradient::SetHue(float hue)
{
  for(size_t i=0;i<colors.size();i++) {
    float h,s,v;
    colors[i].getHSV(h,s,v);
    h = hue;
    colors[i].setHSV(h,s,v);
  }
}

void ColorGradient::SetAlpha(float alpha)
{
  for(size_t i=0;i<colors.size();i++) 
    colors[i].rgba[3]=alpha;
}

void ColorGradient::Eval(float u,GLColor& c) const
{
  if(colors.empty()) c.setBlack(0.0);
  if(u < params.front()) c = colors.front();
  for(size_t i=1;i<params.size();i++)
    if(u < params[i]) {
      float t=(u-params[i-1])/(params[i]-params[i-1]);
      c.blend(colors[i-1],colors[i],t);
      return;
    }
  c = colors.back();
}


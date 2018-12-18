#ifndef GL_COLOR_H
#define GL_COLOR_H

namespace GLDraw {

/** @ingroup GLDraw
 * @brief An rgba color.
 */
struct GLColor
{
  GLColor(float r=1,float g=1,float b=1,float a=1);
  GLColor(const float rgba[4]);
  GLColor(const GLColor& col);
  
  inline operator float* () { return rgba; }
  inline operator const float* () const { return rgba; }
  const GLColor& operator = (const GLColor& col);
  bool operator == (const GLColor& col) const;
  bool operator != (const GLColor& col) const;
  float& operator [] (int i) { return rgba[i]; }
  const float& operator [] (int i) const { return rgba[i]; }
  float& red() { return rgba[0]; }
  float& green() { return rgba[1]; }
  float& blue() { return rgba[2]; }
  float& alpha() { return rgba[3]; }

  void set(float r,float g,float b,float a=1);
  void set(const float rgba[4]);
  void set(const GLColor& col);
  inline void setBlack(float a=1) { set(0,0,0,a); }
  inline void setWhite(float a=1) { set(1,1,1,a); }
  inline void setGray(float c=1) { set(c,c,c); }
  inline void setRed(float r=1) { set(r,0,0); }
  inline void setGreen(float g=1) { set(0,g,0); }
  inline void setBlue(float b=1) { set(0,0,b); }
  inline void setCyan(float c=1) { set(0,c,c); }
  inline void setMagenta(float c=1) { set(c,0,c); }
  inline void setYellow(float c=1) { set(c,c,0); }
  void setRandom();
  void setHSV(float h,const float s,const float v);
  float getLuminance() const;
  void getHSV(float& h, float& s, float& v) const;
  
  void clamp(float min=0.f,float max=1.f);
  void compose(const GLColor& a,const GLColor& b);
  void scale(const GLColor& a,float c);
  void add(const GLColor& a,const GLColor& b);
  void blend(const GLColor& a,const GLColor& b,float t);
  
  void setCurrentGL() const;
  
  float rgba[4];
};

} //namespace GLDraw

#endif

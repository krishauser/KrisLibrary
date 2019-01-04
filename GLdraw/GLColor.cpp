#include "GLColor.h"
#include "GL.h"
#include <utils.h>
#include <math.h>
#include <errors.h>
#include <math/random.h>

using namespace GLDraw;

GLColor::GLColor(float r,float g,float b,float a)
{
	set(r,g,b,a);
}

GLColor::GLColor(const float rgba[4])
{
	set(rgba);
}

GLColor::GLColor(const GLColor& col)
{
	set(col);
}

const GLColor& GLColor::operator = (const GLColor& col)
{
  set(col.rgba);
  return *this;
}

bool GLColor::operator == (const GLColor& col) const
{
  return (rgba[0]==col.rgba[0] &&
	  rgba[1]==col.rgba[1] &&
	  rgba[2]==col.rgba[2] &&
	  rgba[3]==col.rgba[3]);
}

bool GLColor::operator != (const GLColor& col) const
{
  return !operator==(col);
}

void GLColor::set(float r,float g,float b,float a)
{
	rgba[0]=r; rgba[1]=g; rgba[2]=b; rgba[3]=a;
}

void GLColor::set(const float _rgba[4])
{
	rgba[0]=_rgba[0]; rgba[1]=_rgba[1]; rgba[2]=_rgba[2]; rgba[3]=_rgba[3];
}

void GLColor::set(const GLColor& col)
{
	rgba[0]=col.rgba[0];
	rgba[1]=col.rgba[1];
	rgba[2]=col.rgba[2];
	rgba[3]=col.rgba[3];
}

void GLColor::setRandom()
{
	rgba[0]=Math::rng.randFloat();
	rgba[1]=Math::rng.randFloat();
	rgba[2]=Math::rng.randFloat();
	rgba[3]=Math::rng.randFloat();
}


void GLColor::setHSV(float h,const float s,const float v)
{
	if(s==0) { set(v,v,v); return; }
	Assert(h >= 0);
	h/=360.0;h-=floor(h);h*=6; //h is now in [0,6)
	if(h==6.0) h=5.99;
	int i=(int)floor(h);
	float f=h-i,p=v*(1-s),q=v*(1-s*f),t=v*(1-s*(1-f));
	switch(i){
	case 0:set(v,t,p);break;
	case 1:set(q,v,p);break;
	case 2:set(p,v,t);break;
	case 3:set(p,q,v);break;
	case 4:set(t,p,v);break;
	case 5:set(v,p,q);break;
	default:
	  FatalError("Input hue %g is incorrect, maybe negative?",h);
	  break;
	}
}

float GLColor::getLuminance() const
{
	return .299*rgba[0]+.587*rgba[1]+.114*rgba[2];
}

void GLColor::getHSV(float& h, float& s, float& v) const
{
	float r=rgba[0],g=rgba[1],b=rgba[2];
	float max_component=Max(r,g,b), min_component=Min(r,g,b), delta=max_component-min_component;
	v=max_component;
	s=(max_component!=0)?(delta/max_component):0;
	if(s==0) h=0;//an arbitrary but sensible value
	else{
		if(r==max_component) h=(g-b)/delta;
		else if(g==max_component) h=2.0f+(b-r)/delta;
		else h=4.0f+(r-g)/delta;
		h*=60; if(h<0)h+=360.0f;
	}
}	

void GLColor::setCurrentGL() const
{
	glColor4fv(rgba);
}


void GLColor::clamp(float min,float max)
{
  for(int i=0;i<4;i++) {
    if(rgba[i]<min) rgba[i]=min;
    if(rgba[i]>max) rgba[i]=max;
  }
}

void GLColor::compose(const GLColor& a,const GLColor& b)
{
  for(int i=0;i<4;i++)
    rgba[i]=a.rgba[i]*b.rgba[i];
}

void GLColor::scale(const GLColor& a,float c)
{
  for(int i=0;i<4;i++)
    rgba[i]=a.rgba[i]*c;
}

void GLColor::add(const GLColor& a,const GLColor& b)
{
  for(int i=0;i<4;i++)
    rgba[i]=a.rgba[i]+b.rgba[i];
}

void GLColor::blend(const GLColor& a,const GLColor& b,float t)
{
  for(int i=0;i<4;i++)
    rgba[i] = (1.0f-t)*a.rgba[i] + t*b.rgba[i];
}

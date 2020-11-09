#include "SphereWidget.h"
#include "drawextra.h"
#include "drawgeometry.h"
#include <math3d/geometry3d.h>
#include <math3d/basis.h>
#include <math3d/rotation.h>
#include <math/angle.h>
#include <typeinfo>
#include <iostream>

using namespace GLDraw;
using namespace std;


SphereWidget::SphereWidget(const Sphere3D& s)
{
  radius = s.radius;
  transformWidget.T.R.setIdentity();
  transformWidget.T.t = s.center;
  transformWidget.enableRotation = false;
  transformWidget.enableTranslation = true;
  sphereColor.set(1,0,0,0.25);
  ringColor.set(1,0,0);
  ringWidth = 0.005;
  scaleToScreen = true;
  hoverItem = -1;
}

void SphereWidget::Get(Sphere3D& out) const
{
  out.radius = radius;
  out.center = transformWidget.T.t;
}

bool SphereWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project(transformWidget.T.t,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }
  distance = Inf;
  int oldHoverItem = hoverItem;
  hoverItem = -1;
  Ray3D r;
  viewport.getClickSource((float)x, (float)y,r.source);
  viewport.getClickVector((float)x, (float)y,r.direction);
  
  //center overrides ring
  Real dtemp;
  if(transformWidget.Hover(x,y,viewport,dtemp)) {
    distance = dtemp;
    hoverItem = 0;
  }
  else {
    Circle3D c;
    c.center = transformWidget.T.t;
    viewport.getViewVector(c.axis);
    c.radius = radius+ringWidth*globalScale;
    Real t;
    if(c.intersects(r,&t) && t >= 0) {
      c.radius = radius;
      if(!c.intersects(r,NULL)) {
        if(t < distance) {
          distance = t;
          hoverItem = 1;
        }
      }
    }
  }

  if(transformWidget.requestRedraw) {
    Refresh();
    transformWidget.requestRedraw=false; 
  }
  if(hoverItem != oldHoverItem) {
    Refresh();
    if(hasHighlight) {
      if(hoverItem == 0)
        transformWidget.SetHighlight(true);
      else
        transformWidget.SetHighlight(false);
    }
    if(hasFocus) {
      if(hoverItem == 0)
        transformWidget.SetFocus(true);
      else
        transformWidget.SetFocus(false);
    }
  }
  r.eval(distance,hoverPos);
  return hoverItem != -1;
}

bool SphereWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Hover(x,y,viewport,distance);
  if(hoverItem < 0) return false;
  if(hoverItem == 0)
    transformWidget.BeginDrag(x,y,viewport,distance);
  //store start drag information
  clickX = dragX = x;
  clickY = dragY = y;
  clickPos = hoverPos;
  clickDistance = distance;
  clickStartRadius = radius;

  return true;
}

void SphereWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  dragX += dx;
  dragY += dy;
  if(hoverItem < 0) return;
  Ray3D r;
  viewport.getClickSource(dragX,dragY,r.source);
  viewport.getClickVector(dragX,dragY,r.direction);
  Vector3 v;
  viewport.getMovementVectorAtDistance((float)dx, (float)dy, (float)clickDistance, v);
  if(hoverItem == 0) {  //center
    transformWidget.Drag(dx,dy,viewport);
  }
  else { //ring change
    Real d=r.distance(transformWidget.T.t);
    radius = d - clickPos.distance(transformWidget.T.t) + clickStartRadius;
    radius = Max(0.0,radius);
  }
  if(transformWidget.requestRedraw) {
    transformWidget.requestRedraw=false; 
  }
  Refresh();
}

void SphereWidget::EndDrag()
{}

void SphereWidget::SetHighlight(bool value)
{
  Widget::SetHighlight(value);
  if(hoverItem == 0) {
    transformWidget.SetHighlight(value);
  }
  else
    transformWidget.SetHighlight(false);
}

void SphereWidget::SetFocus(bool value)
{
  Widget::SetFocus(value);
  if(hoverItem == 0)
    transformWidget.SetFocus(value);
  else
    transformWidget.SetFocus(false);
}

void SphereWidget::DrawGL(Camera::Viewport& viewport)
{
  transformWidget.DrawGL(viewport);
  glEnable(GL_LIGHTING);
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project(transformWidget.T.t,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }

  //draw ring
  Vector3 axis,x,y;
  viewport.getViewVector(axis);
  axis.inplaceNegative();
  GetCanonicalBasis(axis,x,y);
  Real innerWidth = 0;
  Real outerWidth = ringWidth;
  if(hasHighlight && !transformWidget.hasHighlight) {
    innerWidth = -0.5*ringWidth;
    outerWidth = 1.5*ringWidth;
  }
  Real r1 = radius+innerWidth*globalScale;
  Real r2 = radius+outerWidth*globalScale;
  glDisable(GL_CULL_FACE);
  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,ringColor.rgba); 
  glPushMatrix();
  glTranslate(transformWidget.T.t);
  drawArc((float)r1, (float)r2,axis,x,0,360);
  glPopMatrix();
  glEnable(GL_CULL_FACE);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,sphereColor.rgba); 
  Sphere3D s;
  Get(s);
  draw(GeometricPrimitive3D(s));
  glDisable(GL_BLEND);
}

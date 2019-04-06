#include <KrisLibrary/Logger.h>
#include "TransformWidget.h"
#include "drawextra.h"
#include <math3d/geometry3d.h>
#include <math3d/basis.h>
#include <math3d/rotation.h>
#include <math/angle.h>
#include <typeinfo>

using namespace GLDraw;
using namespace std;

Widget::Widget()
  :hasHighlight(false),hasFocus(false), requestRedraw(true)
{}

WidgetSet::WidgetSet()
  :activeWidget(NULL),closestWidget(NULL)
{}

void WidgetSet::Enable(int index,bool enabled)
{
  Assert(index >= 0 && index < (int)widgets.size());
  widgetEnabled.resize(widgets.size(),true);
  widgetEnabled[index] = enabled;
}

void WidgetSet::Enable(Widget* widget,bool enabled)
{
  widgetEnabled.resize(widgets.size(),true);
  for(size_t i=0;i<widgets.size();i++)
    if(widgets[i] == widget) widgetEnabled[i] = enabled;
}

bool WidgetSet::Hover(int x,int y,Camera::Viewport& viewport,double& closestDistance)
{
  widgetEnabled.resize(widgets.size(),true);
  closestDistance = Inf;
  closestWidget = NULL;
  for(size_t i=0;i<widgets.size();i++) {
    if(!widgetEnabled[i]) continue;
    Real d;
    if(widgets[i]->Hover(x,y,viewport,d)) {
      if(d < closestDistance) {
	closestDistance = d;
	closestWidget = widgets[i];
      }
    }
  }
  for(size_t i=0;i<widgets.size();i++)
    if(widgets[i]->requestRedraw) { Refresh(); widgets[i]->requestRedraw=false; }
  if(closestWidget) return true;
  return false;
}

void WidgetSet::SetHighlight(bool value)
{
  Widget::SetHighlight(value);
  if(value) {
    if(activeWidget != closestWidget) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Activate widget\n");
      if(activeWidget && activeWidget != closestWidget) activeWidget->SetHighlight(false);
      if(closestWidget) closestWidget->SetHighlight(true);
      if(closestWidget && closestWidget->requestRedraw) {
	Refresh();
	closestWidget->requestRedraw=false; 
      }
      if(activeWidget && activeWidget->requestRedraw) {
	Refresh();
	activeWidget->requestRedraw=false; 
      }
      activeWidget = closestWidget;
      closestWidget = NULL;
    }
  }
  else {
    if(activeWidget) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Deactivate widget\n");
      activeWidget->SetHighlight(false);
      if(activeWidget->requestRedraw) {
	Refresh();
	activeWidget->requestRedraw=false; 
      }
      activeWidget = NULL;
    }
  }
}

bool WidgetSet::BeginDrag(int x,int y,Camera::Viewport& viewport,double& closestDistance) 
{
  widgetEnabled.resize(widgets.size(),true);
  closestDistance = Inf;
  closestWidget = NULL;
  for(size_t i=0;i<widgets.size();i++) {
    if(!widgetEnabled[i]) continue;
    Real d;
    if(widgets[i]->BeginDrag(x,y,viewport,d)) {
      if(d < closestDistance) {
	closestDistance = d;
	closestWidget = widgets[i];
      }
    }
  }
  for(size_t i=0;i<widgets.size();i++)
    if(widgets[i]->requestRedraw) { Refresh(); widgets[i]->requestRedraw=false; }

  if(closestWidget) return true;
  return false;
}

void WidgetSet::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(activeWidget) {
    activeWidget->Drag(dx,dy,viewport);
    if(activeWidget->requestRedraw) { Refresh(); activeWidget->requestRedraw=false; }
  }
}

void WidgetSet::EndDrag()
{
  if(activeWidget) {
    activeWidget->EndDrag();
    if(activeWidget->requestRedraw) { Refresh(); activeWidget->requestRedraw=false; }
  }
}

void WidgetSet::SetFocus(bool value) 
{
  Widget::SetFocus(value);
  if(value) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Set focus on widget "<<typeid(*this).name()<<", sub-widget "<<(closestWidget?typeid(*closestWidget).name():"NULL"));
    if(activeWidget && activeWidget != closestWidget) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"... Removing focus on sub-widget "<<(activeWidget?typeid(*activeWidget).name():"NULL"));
      activeWidget->SetFocus(false);
    }
    if(closestWidget) closestWidget->SetFocus(true);  
    if(closestWidget && closestWidget->requestRedraw) {
      Refresh();
      closestWidget->requestRedraw=false; 
    }
    if(activeWidget && activeWidget->requestRedraw) {
      Refresh();
      activeWidget->requestRedraw=false; 
    }
    activeWidget = closestWidget;
    closestWidget = NULL;
  }
  else {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Remove focus on widget "<<typeid(*this).name()<<", sub-widget "<<(activeWidget?typeid(*activeWidget).name():"NULL"));
    if(activeWidget) activeWidget->SetFocus(false);
    activeWidget = NULL;
  }
}
void WidgetSet::Keypress(char c)
{
  if(activeWidget) {
    activeWidget->Keypress(c);
    if(activeWidget->requestRedraw) { Refresh(); activeWidget->requestRedraw=false; }
  }
}
void WidgetSet::DrawGL(Camera::Viewport& viewport)
{
  widgetEnabled.resize(widgets.size(),true);
  for(size_t i=0;i<widgets.size();i++) {
    if(widgetEnabled[i])
      widgets[i]->DrawGL(viewport);
  }
}

void WidgetSet::Idle()
{
  widgetEnabled.resize(widgets.size(),true);
  for(size_t i=0;i<widgets.size();i++) {
    if(widgetEnabled[i])
      widgets[i]->Idle();
  }
}

TransformWidget::TransformWidget()
{
  T.setIdentity();
  enableRotation=enableTranslation=true;
  enableOriginTranslation=enableOuterRingRotation=true;
  for(int i=0;i<3;i++) 
    enableRotationAxes[i]=enableTranslationAxes[i]=true;
  originColor.set(1,1,1);
  xAxisColor.set(1,0,0);
  yAxisColor.set(0,1,0);
  zAxisColor.set(0,0,1);
  hoverScale = 1.5;
  originRadius = 0.01;
  axisLength = 0.1;
  axisRadius = 0.005;
  arrowRadius = 0.01;
  arrowHeight = 0.02;
  ringOuterRadius = 0.1;
  ringInnerRadius = 0.095;
  scaleToScreen = true;

  hoverItem = -1;
}

bool TransformWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project(T.t,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }
  distance = Inf;
  int oldHoverItem = hoverItem;
  hoverItem = -1;
  Ray3D r;
  viewport.getClickSource((float)x, (float)y,r.source);
  viewport.getClickVector((float)x, (float)y,r.direction);
  //check origin
  if(enableTranslation && enableOriginTranslation) {
    Sphere3D s;
    s.center = T.t;
    s.radius = originRadius*globalScale;
    Real tmin,tmax;
    if(s.intersects(r,&tmin,&tmax)) {
      distance = tmin;
      hoverItem = 0;
    }
  }
  //check translation axes
  for(int i=0;i<3;i++) {
    if(!enableTranslation) break;
    if(!enableTranslationAxes[i]) continue;
    Line3D axisLine;
    axisLine.source = T.t;
    axisLine.direction = Vector3(T.R.col(i));
    Real t,u;
    axisLine.closestPoint(r,t,u);
    t = Clamp(t,0.0,axisLength*globalScale);
    u = Clamp(u,0.0,Inf);
    Vector3 paxis,pray;
    axisLine.eval(t,paxis);
    r.eval(u,pray);
    if(paxis.distanceSquared(pray) <= Sqr(axisRadius*globalScale)) {
      if(u < distance) {
	distance = u;
	hoverItem = 1+i;
      }
    }
  }
  if(enableRotation) {
    //check rotation rings
    Circle3D c;
    c.center = T.t;
    for(int i=0;i<3;i++) {
      if(!enableRotationAxes[i]) continue;
      c.axis = Vector3(T.R.col(i));
      c.radius = ringOuterRadius*globalScale;
      Real t;
      if(c.intersects(r,&t) && t >= 0) {
	c.radius = ringInnerRadius*globalScale;
	if(!c.intersects(r,NULL)) {
	  if(t < distance) {
	    distance = t;
	    hoverItem = i+4;
	  }
	}
      }
    }
  }
  if(enableRotation && enableOuterRingRotation) {
    //check outer ring
    Circle3D c;
    c.center = T.t;
    viewport.getViewVector(c.axis);
    c.radius = (ringOuterRadius+arrowHeight)*globalScale;
    Real t;
    if(c.intersects(r,&t) && t >= 0) {
      c.radius = (ringInnerRadius+arrowHeight)*globalScale;
      if(!c.intersects(r,NULL)) {
	if(t < distance) {
	  distance = t;
	  hoverItem = 7;
	}
      }
    }
    clickAxis = c.axis;
  }
  if(hoverItem != oldHoverItem) Refresh();
  r.eval(distance,hoverPos);
  return hoverItem != -1;
}

bool TransformWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Hover(x,y,viewport,distance);
  if(hoverItem < 0) return false;
  //store start drag information
  clickTransform = T;
  clickX = dragX = x;
  clickY = dragY = y;
  clickPos = hoverPos;
  clickDistance = distance;

  return true;
}

void TransformWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  dragX += dx;
  dragY += dy;
  Ray3D r;
  viewport.getClickSource(dragX,dragY,r.source);
  viewport.getClickVector(dragX,dragY,r.direction);
  if(hoverItem < 0) return;
  else if(hoverItem == 0) {
    Vector3 v;
    viewport.getMovementVectorAtDistance((float)dx, (float)dy, (float)clickDistance, v);
    T.t += v;
  }
  else if(hoverItem <= 3) { //translation
    Line3D axisLine;
    axisLine.source = clickPos;
    axisLine.direction = Vector3(T.R.col(hoverItem-1));
    Real t,u;
    axisLine.closestPoint(r,t,u);
    T.t = clickTransform.t + axisLine.direction*t;
  }
  else {
    Plane3D ringPlane;
    Vector3 axis;
    if(hoverItem <= 6) axis = Vector3(clickTransform.R.col(hoverItem-4));
    else axis = clickAxis;
    Vector3 x,y;
    GetCanonicalBasis(axis,x,y);
    //find rotation to minimize distance from clicked pos to drag ray
    Real cx = x.dot(clickPos - T.t);
    Real cy = y.dot(clickPos - T.t);
    ringPlane.setPointNormal(T.t,axis);
    Real t;
    bool res=ringPlane.intersectsRay(r,&t);
    //odd... no intersection
    if(res==false) return;
    Vector3 raypos = r.source + t*r.direction - T.t;
    Real rx = x.dot(raypos);
    Real ry = y.dot(raypos);
    if(Sqr(rx) + Sqr(ry) < 1e-5) return;
    Real theta = AngleDiff(Atan2(ry,rx),Atan2(cy,cx));
    AngleAxisRotation aa;
    aa.axis = axis;
    aa.angle = theta;
    QuaternionRotation qR,qT,qRes;
    qR.setAngleAxis(aa);
    qT.setMatrix(clickTransform.R);
    qRes.mul(qR,qT);
    qRes.getMatrix(T.R);
  }
  Refresh();
}


void TransformWidget::EndDrag()
{}

void TransformWidget::DrawGL(Camera::Viewport& viewport)
{
  glEnable(GL_LIGHTING);
  Real scale=1.0;
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project(T.t,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }

  glPushMatrix();
  glTranslate(T.t);
  //draw origin
  if(enableTranslation && enableOriginTranslation) {
    scale = (hasHighlight && hoverItem == 0 ? hoverScale : 1.0)*globalScale;
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,originColor.rgba); 
    drawSphere((float)originRadius*scale,16,8);
  }
  else {
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,originColor.rgba); 
    drawSphere((float)axisRadius*globalScale,16,8);
  }

  //draw axes
  if(enableTranslation) {
    Vector3 axis;
    if(enableTranslationAxes[0]) {
      axis.set(T.R.col1());
      scale = (hasHighlight && hoverItem == 1 ? hoverScale : 1.0)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,xAxisColor.rgba); 
      drawCylinder(axis*axisLength*globalScale,axisRadius*scale,8);
      glPushMatrix();
      glTranslate(axis*axisLength*globalScale);
      drawCone(axis*arrowHeight*scale, (float)arrowRadius*scale);
      glPopMatrix();
    }

    if(enableTranslationAxes[1]) {
      axis.set(T.R.col2());
      scale = (hasHighlight && hoverItem == 2 ? hoverScale : 1.0)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,yAxisColor.rgba); 
      drawCylinder(axis*axisLength*globalScale,axisRadius*scale,8);
      glPushMatrix();
      glTranslate(axis*axisLength*globalScale);
      drawCone(axis*arrowHeight*scale, (float)arrowRadius*scale);
      glPopMatrix();
    }

    if(enableTranslationAxes[2]) {
      axis.set(T.R.col3());
      scale = (hasHighlight && hoverItem == 3 ? hoverScale : 1.0)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,zAxisColor.rgba); 
      drawCylinder(axis*axisLength*globalScale,axisRadius*scale,8);
      glPushMatrix();
      glTranslate(axis*axisLength*globalScale);
      drawCone(axis*arrowHeight*scale,arrowRadius*scale);
      glPopMatrix();
    }
  }

  //TODO: indicate original matrix rotation

  //draw rings
  if(enableRotation) {
    glDisable(GL_CULL_FACE);
    Vector3 x,y;
    Vector3 axis;
    Real r1,r2;
    if (enableRotationAxes[0]) {
      axis = Vector3(T.R.col1()); 
      GetCanonicalBasis(axis,x,y);
      r1 = (hasHighlight && hoverItem == 4 ? ringInnerRadius - (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringInnerRadius)*globalScale;
      r2 = (hasHighlight && hoverItem == 4 ? ringOuterRadius + (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringOuterRadius)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,xAxisColor.rgba); 
      drawArc(r1,r2,axis,x,0,360);
    }
    
    if (enableRotationAxes[1]) {
      axis = Vector3(T.R.col2()); 
      GetCanonicalBasis(axis,x,y);
      r1 = (hasHighlight && hoverItem == 5 ? ringInnerRadius - (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringInnerRadius)*globalScale;
      r2 = (hasHighlight && hoverItem == 5 ? ringOuterRadius + (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringOuterRadius)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,yAxisColor.rgba); 
      drawArc(r1,r2,axis,x,0,360);
    }

    if (enableRotationAxes[2]) {
      axis = Vector3(T.R.col3()); 
      GetCanonicalBasis(axis,x,y);
      r1 = (hasHighlight && hoverItem == 6 ? ringInnerRadius - (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringInnerRadius)*globalScale;
      r2 = (hasHighlight && hoverItem == 6 ? ringOuterRadius + (ringOuterRadius - ringInnerRadius)*0.5*hoverScale : ringOuterRadius)*globalScale;
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,zAxisColor.rgba); 
      drawArc(r1,r2,axis,x,0,360);
    }

    if(enableOuterRingRotation && hoverItem == 7) {
      axis = clickAxis;
      GetCanonicalBasis(axis,x,y);
      r1 = (ringInnerRadius+arrowHeight)*globalScale;
      r2 = (ringOuterRadius+arrowHeight)*globalScale;
      float color[4] = {0,0,0,1};
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color); 
      drawArc((float)r1, (float)r2,axis,x,0,360);
    }
    glEnable(GL_CULL_FACE);
  }

  glPopMatrix();
}

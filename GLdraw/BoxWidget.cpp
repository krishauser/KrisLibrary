#include "BoxWidget.h"
#include "drawextra.h"
#include "drawgeometry.h"
#include <math3d/geometry3d.h>
#include <math3d/basis.h>
#include <math3d/rotation.h>
#include <math/angle.h>
#include <typeinfo>

using namespace GLDraw;
using namespace std;


BoxWidget::BoxWidget(const AABB3D& bb)
{
  this->bb = bb;
  T.setIdentity();
  transformWidget.T.R = T.R;
  transformWidget.T.t = T*(0.5*(bb.bmin+bb.bmax));
  transformWidget.enableRotation = false;
  transformWidget.enableTranslation = true;
  transformWidget.axisLength *= 0.5;
  transformWidget.axisRadius *= 0.5;
  transformWidget.arrowRadius *= 0.5;
  transformWidget.arrowHeight *= 0.5;
  cornerColor.set(1,0,0);
  boxColor.set(1,0,0,0.25);
  hoverColorBlend.set(1,1,0);
  cornerRadius = 0.01;
  scaleToScreen = true;
  hoverItem[0] = -1;
}

BoxWidget::BoxWidget(const Box3D& bb)
{
  this->bb.bmin.setZero();
  this->bb.bmax = bb.dims;
  T.R.set(bb.xbasis,bb.ybasis,bb.zbasis);
  T.t = bb.origin;
  transformWidget.T.R = T.R;
  transformWidget.T.t = T*(0.5*(this->bb.bmin+this->bb.bmax));
  transformWidget.enableRotation = true;
  transformWidget.enableTranslation = true;
  transformWidget.axisLength *= 0.5;
  transformWidget.axisRadius *= 0.5;
  transformWidget.arrowRadius *= 0.5;
  transformWidget.arrowHeight *= 0.5;
  cornerColor.set(1,0,0);
  boxColor.set(1,0,0,0.25);
  hoverColorBlend.set(1,1,0);
  cornerRadius = 0.01;
  scaleToScreen = true;
  hoverItem[0] = -1;
}

void BoxWidget::Get(AABB3D& out) const
{
  out = this->bb;
}

void BoxWidget::Get(Box3D& out) const
{
  out.origin = T*bb.bmin;
  out.dims = bb.bmax-bb.bmin;
  T.R.get(out.xbasis,out.ybasis,out.zbasis);
}

bool BoxWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project((bb.bmin+bb.bmax)*0.5,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }
  distance = Inf;
  IntTriple oldHoverItem = hoverItem;
  hoverItem[0] = -1;
  Ray3D r;
  Vector3 temp;
  viewport.getClickSource((float)x, (float)y,temp);
  T.mulInverse(temp,r.source);
  viewport.getClickVector((float)x, (float)y,temp);
  T.R.mulTranspose(temp,r.direction);

  //center overrides box 
  Real dtemp;
  if(transformWidget.Hover(x,y,viewport,dtemp)) {
    distance = dtemp;
    hoverItem.set(1,1,1);
  }
  else {
    //r is now a local point
    Real tmin=0,tmax=Inf;
    if(r.intersects(bb,tmin,tmax)) {
      //hover face
      Vector3 p;
      r.eval(tmin,p);
      Vector3 dedge;
      dedge.x = Min(Abs(p.x-bb.bmin.x),Abs(p.x-bb.bmax.x));
      dedge.y = Min(Abs(p.y-bb.bmin.y),Abs(p.y-bb.bmax.y));
      dedge.z = Min(Abs(p.z-bb.bmin.z),Abs(p.z-bb.bmax.z));
      hoverItem.set(1,1,1);
      Real dmin = Min(dedge.x,dedge.y,dedge.z);
      if(dmin < 1e-3) { //not inside?
        distance = tmin;  
        if(dedge.x == dmin)
          hoverItem[0] = (Abs(p.x-bb.bmin.x) < Abs(p.x-bb.bmax.x) ? 0: 2);
        else if(dedge.y == dmin)
          hoverItem[1] = (Abs(p.y-bb.bmin.y) < Abs(p.y-bb.bmax.y) ? 0: 2);
        else 
          hoverItem[2] = (Abs(p.z-bb.bmin.z) < Abs(p.z-bb.bmax.z) ? 0: 2);
      }
    }
  }
  if(cornerRadius > 0) {
    //check corners
    Sphere3D s;
    s.radius = cornerRadius*globalScale;
    Real tmin,tmax;
    s.center = bb.bmin;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(0,0,0);
      }
    }
    s.center.x = bb.bmax.x;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(2,0,0);
      }
    }
    s.center.x = bb.bmin.x;
    s.center.y = bb.bmax.y;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(0,2,0);
      }
    }
    s.center.x = bb.bmax.x;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(2,2,0);
      }
    }
    s.center.x = bb.bmin.x;
    s.center.y = bb.bmin.y;
    s.center.z = bb.bmax.z;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(0,0,2);
      }
    }
    s.center.x = bb.bmax.x;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(2,0,2);
      }
    }
    s.center.x = bb.bmin.x;
    s.center.y = bb.bmax.y;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(0,2,2);
      }
    }
    s.center.x = bb.bmax.x;
    if(s.intersects(r,&tmin,&tmax)) {
      if(tmin < distance) {
        distance = tmin;
        hoverItem.set(2,2,2);
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
      if(hoverItem == IntTriple(1,1,1))
        transformWidget.SetHighlight(true);
      else
        transformWidget.SetHighlight(false);
    }
    if(hasFocus) {
      if(hoverItem == IntTriple(1,1,1))
        transformWidget.SetFocus(true);
      else
        transformWidget.SetFocus(false);
    }
  }
  r.eval(distance,hoverPosLocal);
  hoverPos = T*hoverPosLocal; // convert back to world coordinates
  return hoverItem[0] != -1;
}

bool BoxWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  Hover(x,y,viewport,distance);
  if(hoverItem[0] < 0) return false;
  if(hoverItem == IntTriple(1,1,1))
    transformWidget.BeginDrag(x,y,viewport,distance);
  //store start drag information
  clickX = dragX = x;
  clickY = dragY = y;
  clickPos = hoverPos;
  clickPosLocal = hoverPosLocal;
  clickDistance = distance;

  return true;
}

void BoxWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  dragX += dx;
  dragY += dy;
  Ray3D r;
  Vector3 temp;
  viewport.getClickSource(dragX,dragY,temp);
  T.mulInverse(temp,r.source);
  viewport.getClickVector(dragX,dragY,temp);
  T.R.mulTranspose(temp,r.direction);
  Vector3 v;
  viewport.getMovementVectorAtDistance((float)dx, (float)dy, (float)clickDistance, temp);
  T.R.mulTranspose(temp,v);
  if(hoverItem[0] < 0) return;
  else if(hoverItem == IntTriple(1,1,1)) {  //center
    Vector3 c_rel;
    T.mulInverse(transformWidget.T.t,c_rel);
    Vector3 bmin_rel = bb.bmin - c_rel;
    Vector3 bmax_rel = bb.bmax - c_rel;
    transformWidget.Drag(dx,dy,viewport);
    T.R = transformWidget.T.R;
    T.mulInverse(transformWidget.T.t,c_rel);
    bb.bmin = bmin_rel + c_rel;
    bb.bmax = bmax_rel + c_rel;
  }
  else { //translation
    int numOnes = 0;
    int selectedDim = -1;
    if(hoverItem[0] == 1) numOnes += 1;
    else selectedDim = 0;
    if(hoverItem[1] == 1) numOnes += 1;
    else selectedDim = 1;
    if(hoverItem[2] == 1) numOnes += 1;
    else selectedDim = 2;
    if(numOnes == 2) {
      //plane
      Line3D axisLine;
      axisLine.source = clickPosLocal;
      axisLine.direction.setZero();
      axisLine.direction[selectedDim] = hoverItem[selectedDim]-1;
      Real t,u;
      axisLine.closestPoint(r,t,u);
      if(hoverItem[selectedDim] > 1) {
        bb.bmax[selectedDim] = clickPosLocal[selectedDim] + t;
        if(bb.bmax[selectedDim] < bb.bmin[selectedDim])
          bb.bmax[selectedDim] = bb.bmin[selectedDim];
      }
      else {
        bb.bmin[selectedDim] = clickPosLocal[selectedDim] - t;
        if(bb.bmin[selectedDim] > bb.bmax[selectedDim])
          bb.bmin[selectedDim] = bb.bmax[selectedDim];
      }
    }
    else {
      Assert(numOnes == 0);
      Vector3 pt=bb.bmin;
      Vector3 opp=bb.bmax;
      if(hoverItem[0]==2) { pt.x = bb.bmax.x; opp.x = bb.bmin.x; }
      if(hoverItem[1]==2) { pt.y = bb.bmax.y; opp.y = bb.bmin.y; }
      if(hoverItem[2]==2) { pt.z = bb.bmax.z; opp.z = bb.bmin.z; }
      pt += v;
      bb.bmin.x = Min(pt.x,opp.x);
      bb.bmin.y = Min(pt.y,opp.y);
      bb.bmin.z = Min(pt.z,opp.z);
      bb.bmax.x = Max(pt.x,opp.x);
      bb.bmax.y = Max(pt.y,opp.y);
      bb.bmax.z = Max(pt.z,opp.z);
      if(pt.x < opp.x && hoverItem[0] == 2) hoverItem[0] = 0;
      if(pt.x > opp.x && hoverItem[0] == 0) hoverItem[0] = 2;
      if(pt.y < opp.y && hoverItem[1] == 2) hoverItem[1] = 0;
      if(pt.y > opp.y && hoverItem[1] == 0) hoverItem[1] = 2;
      if(pt.z < opp.z && hoverItem[2] == 2) hoverItem[2] = 0;
      if(pt.z > opp.z && hoverItem[2] == 0) hoverItem[2] = 2;
    }
    transformWidget.T.t = T*(0.5*(bb.bmin+bb.bmax));
  }
  if(transformWidget.requestRedraw) {
    transformWidget.requestRedraw=false; 
  }
  Refresh();
}

void BoxWidget::EndDrag()
{}

void BoxWidget::SetHighlight(bool value)
{
  Widget::SetHighlight(value);
  if(hoverItem == IntTriple(1,1,1)) {
    transformWidget.SetHighlight(value);
  }
  else
    transformWidget.SetHighlight(false);
  if(!value)
    hoverItem[0] = -1;
}

void BoxWidget::SetFocus(bool value)
{
  Widget::SetFocus(value);
  if(hoverItem == IntTriple(1,1,1))
    transformWidget.SetFocus(value);
  else
    transformWidget.SetFocus(false);
}

void BoxWidget::DrawGL(Camera::Viewport& viewport)
{
  transformWidget.DrawGL(viewport);
  glEnable(GL_LIGHTING);
  Real globalScale = 1.0;
  if(scaleToScreen) {
    float sx,sy,sz;
    viewport.project((bb.bmin+bb.bmax)*0.5,sx,sy,sz);
    globalScale = sz/viewport.scale;
  }

  glPushMatrix();
  glMultMatrix(Matrix4(T));

  if(hasHighlight && !transformWidget.hasHighlight) {
    if(cornerRadius > 0 && (hoverItem[0] ==0 || hoverItem[0]==2)) {
      if(hoverItem[1] == 0 || hoverItem[1] == 2) {
        if(hoverItem[2] == 0 || hoverItem[2] == 2) {
          Vector3 pt = bb.bmin;
          if(hoverItem[0]==2) pt.x = bb.bmax.x;
          if(hoverItem[1]==2) pt.y = bb.bmax.y;
          if(hoverItem[2]==2) pt.z = bb.bmax.z;
          glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cornerColor.rgba); 
          glPushMatrix();
          glTranslate(pt);
          drawSphere((float)cornerRadius*globalScale,16,8);
          glPopMatrix();
        }
      }
    }
  }

  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  
  if(hasHighlight && !transformWidget.hasHighlight) {
    int numOnes = 0;
    int selectedDim = -1;
    if(hoverItem[0] == 1) numOnes += 1;
    else selectedDim = 0;
    if(hoverItem[1] == 1) numOnes += 1;
    else selectedDim = 1;
    if(hoverItem[2] == 1) numOnes += 1;
    else selectedDim = 2;
    if(numOnes==2) {
      Vector3 a,b,c,d;
      a = bb.bmin;
      b = bb.bmin;
      c = bb.bmax;
      d = bb.bmax;
      int dnext=(selectedDim+1)%3;
      b[dnext] = bb.bmax[dnext];
      d[dnext] = bb.bmin[dnext];
      GLColor hoverColor;
      hoverColor.blend(hoverColorBlend,boxColor,0.5);
      glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,hoverColor.rgba); 
      if(hoverItem[selectedDim] == 0)
        a[selectedDim] = b[selectedDim] = c[selectedDim] = d[selectedDim] = bb.bmin[selectedDim];
      else
        a[selectedDim] = b[selectedDim] = c[selectedDim] = d[selectedDim] = bb.bmax[selectedDim];
      if((selectedDim==0) || (selectedDim == 1 && hoverItem[selectedDim] == 0)) 
        drawQuad(a,b,c,d);
      else 
        drawQuad(a,d,c,b);
    }
  }
  glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,boxColor.rgba); 
  draw(GeometricPrimitive3D(bb));
  glEnable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  glPopMatrix();
}

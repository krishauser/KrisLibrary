#ifndef GLDRAW_SPHERE_WIDGET_H
#define GLDRAW_SPHERE_WIDGET_H

#include "Widget.h"
#include "TransformWidget.h"
#include <KrisLibrary/math3d/Sphere3D.h>

namespace GLDraw {

class SphereWidget : public Widget
{
 public:
  SphereWidget(const Sphere3D& s);
  void Get(Sphere3D& s) const;
  virtual ~SphereWidget() {}
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) override;
  virtual void EndDrag() override;
  virtual void DrawGL(Camera::Viewport& viewport) override;
  virtual void SetHighlight(bool value) override;
  virtual void SetFocus(bool value) override;

  Real radius;
  TransformWidget transformWidget;  //<this widget handles the center translation and rotation changes

  //editor settings
  GLColor sphereColor,ringColor;
  Real ringWidth;
  bool scaleToScreen;

  //state
  int hoverItem;            //< -1: nothing, 0: center, 1: ring
  Vector3 hoverPos;
  int clickX,clickY,dragX,dragY;
  Vector3 clickPos;
  Real clickDistance;
  Real clickStartRadius;
};

} //namespace GLDraw

#endif

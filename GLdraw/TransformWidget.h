#ifndef GLDRAW_TRANSFORM_WIDGET_H
#define GLDRAW_TRANSFORM_WIDGET_H

#include "Widget.h"

namespace GLDraw {

class TransformWidget : public Widget
{
 public:
  TransformWidget();
  virtual ~TransformWidget() {}
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void EndDrag();
  virtual void DrawGL(Camera::Viewport& viewport);

  //settings
  RigidTransform T;
  bool enableRotation,enableTranslation;
  bool enableOriginTranslation,enableOuterRingRotation;
  bool enableRotationAxes[3],enableTranslationAxes[3];
  GLColor originColor,xAxisColor,yAxisColor,zAxisColor;
  double hoverScale;
  double originRadius;
  double axisLength,axisRadius,arrowRadius,arrowHeight;
  double ringOuterRadius,ringInnerRadius;
  bool scaleToScreen;

  //state
  int hoverItem; // -1 = nothing, 0 = origin, 1,2,3 = x,y,z translation axes, 4,5,6 = x,y,z rotation axes, 7 = outer rotation ring
  Vector3 hoverPos;
  RigidTransform clickTransform;
  int clickX,clickY,dragX,dragY;
  Vector3 clickPos;
  Real clickDistance;
  Vector3 clickAxis;
};

} //namespace GLDraw

#endif

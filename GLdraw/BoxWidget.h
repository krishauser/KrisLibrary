#ifndef GLDRAW_BOX_WIDGET_H
#define GLDRAW_BOX_WIDGET_H

#include "Widget.h"
#include "TransformWidget.h"
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/Box3D.h>
#include <KrisLibrary/utils/IntTriple.h>

namespace GLDraw {

class BoxWidget : public Widget
{
 public:
  BoxWidget(const AABB3D& bb);
  BoxWidget(const Box3D& bb);
  void Get(AABB3D& bb) const;
  void Get(Box3D& bb) const;
  virtual ~BoxWidget() {}
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) override;
  virtual void EndDrag() override;
  virtual void DrawGL(Camera::Viewport& viewport) override;
  virtual void SetHighlight(bool value) override;
  virtual void SetFocus(bool value) override;

  RigidTransform T;  //< the transform of the origin. bb is expressed w.r.t. this frame
  AABB3D bb;
  TransformWidget transformWidget;  //<this widget handles the center translation and rotation changes

  //editor settings
  GLColor cornerColor,boxColor;
  GLColor hoverColorBlend;
  Real cornerRadius;
  bool scaleToScreen;

  //state
  IntTriple hoverItem;            //< 111: center, 101: lower y face, 121: upper y face, 000: -x -y -z corner, 002: -x -y +z corner, etc.
  Vector3 hoverPos,hoverPosLocal;
  int clickX,clickY,dragX,dragY;
  Vector3 clickPos,clickPosLocal;
  Real clickDistance;
  Vector3 clickAxis;
};

} //namespace GLDraw

#endif

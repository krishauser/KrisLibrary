#ifndef GL_WIDGET_H
#define GL_WIDGET_H

#include "GL.h"
#include "GLColor.h"
#include <KrisLibrary/camera/viewport.h>
#include <vector>

namespace GLDraw {

using namespace Math3D;

/** @ingroup GLDraw
 * @brief Base class for a visual widget
 */
class Widget
{
 public:
  Widget();
  virtual ~Widget() {}
  //returns true if it has been hovered over.  The widget should be given
  //highlight
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance) { return false; }
  virtual void SetHighlight(bool value) { hasHighlight=value; }
  //returns true if it was clicked.  The widget should be given focus, if
  //nothing was closer)
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) { return false; }
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) {}
  virtual void EndDrag() {}
  virtual void SetFocus(bool value) { hasFocus=value; }
  virtual void Keypress(char c) {}
  virtual void DrawGL(Camera::Viewport& viewport) {}
  virtual void Idle() {}

  //widgets call this internally
  void Refresh() { requestRedraw = true; }

  bool hasHighlight,hasFocus;
  bool requestRedraw;
};

/** @ingroup GLDraw
 * @brief A container of sub-widgets.
 *
 * Note that this does not manage the storage of widgets.
 */
class WidgetSet : public Widget
{
 public:
  WidgetSet();
  virtual ~WidgetSet() {}
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void SetHighlight(bool value);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void EndDrag();
  virtual void SetFocus(bool value);
  virtual void Keypress(char c);
  virtual void DrawGL(Camera::Viewport& viewport);
  virtual void Idle();
  virtual void Enable(int index,bool enabled);
  virtual void Enable(Widget* widget,bool enabled);

  std::vector<Widget*> widgets;
  std::vector<bool> widgetEnabled;
  Widget* activeWidget;
  Widget* closestWidget;
};

} //namespace GLDraw

#endif

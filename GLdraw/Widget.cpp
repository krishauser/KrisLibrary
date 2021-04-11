#include <KrisLibrary/Logger.h>
#include "Widget.h"
#include <KrisLibrary/errors.h>
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
  requestRedraw = false;
}

void WidgetSet::Idle()
{
  widgetEnabled.resize(widgets.size(),true);
  for(size_t i=0;i<widgets.size();i++) {
    if(widgetEnabled[i])
      widgets[i]->Idle();
  }
}

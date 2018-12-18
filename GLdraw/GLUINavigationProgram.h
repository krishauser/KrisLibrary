#ifndef GL_GLUI_NAVIGATION_PROGRAM_H
#define GL_GLUI_NAVIGATION_PROGRAM_H


#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/Timer.h>
#include "GLUIProgram.h"

/** @ingroup GLDraw
 * @brief A 3D navigation program based on GLUI.
 * @sa GLUTNavigationProgram
 */
class GLUINavigationProgram : public GLUIProgramBase
{
public:
  GLUINavigationProgram();
  virtual bool Initialize();
  virtual void Handle_Display();
  virtual void Handle_Reshape(int w,int h);
  virtual void Handle_Click(int button,int state,int x,int y);
  virtual void Handle_Drag(int x,int y);
  virtual void Handle_Keypress(unsigned char key,int x,int y);
  virtual void Handle_Idle();
  
  //overrideable
  virtual void SetWorldLights() {}
  virtual void RenderWorld() {}
  virtual void RenderScreen() {}
  
  virtual void BeginDrag(int x,int y,int button,int modifiers);
  virtual void DoDrag(int dx,int dy,int button,int modifiers);
  virtual void EndDrag(int x,int y,int button,int modifiers);
  virtual void DoFreeDrag(int dx,int dy,int button);
  virtual void DoCtrlDrag(int dx,int dy,int button);
  virtual void DoAltDrag(int dx,int dy,int button);
  virtual void DoShiftDrag(int dx,int dy,int button);

  void DragPan(int dx,int dy);
  void DragRotate(int dx,int dy);
  void DragZoom(int dx,int dy);
  void DragTruck(int dx,int dy);

  void Set2DMode(bool mode=true);
  void DisplayCameraTarget();
  void CenterCameraOn(const Math3D::AABB3D& bbox);

  void WriteDisplaySettings(std::ostream& out) const;
  void ReadDisplaySettings(std::istream& in);
  
  Camera::Viewport viewport;
  Camera::CameraController_Orbit camera;
  int oldmousex,oldmousey;
  int clickButton, clickModifiers;
  bool stereo_mode;
  float stereo_offset;
  Timer timer;
  int show_view_target; float t_hide_view_target;
  double last_frame_time;
  bool show_frames_per_second;
  int frames_rendered;
  bool mode_2d;
};

#endif

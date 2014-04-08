#ifndef GL_GLUT_PROGRAM_H
#define GL_GLUT_PROGRAM_H

#include <limits.h>

class GLUTProgramBase
{
public:
  GLUTProgramBase(int width=800,int height=600);
  virtual ~GLUTProgramBase() {}
  //if displayMode is non-zero, initializes glut with that display mode
  int Run(const char *window_title="OpenGL Viewer",unsigned int displayMode=0);

  //overrideable
  virtual bool Initialize();
  virtual void Handle_Display() {}
  virtual void Handle_Reshape(int w,int h) { width=w; height=h; }
  virtual void Handle_Keypress(unsigned char key,int x,int y){}
  virtual void Handle_KeypressUp(unsigned char key,int x,int y){}
  virtual void Handle_Special(int key,int x,int y) {}
  virtual void Handle_SpecialUp(int key,int x,int y) {}
  virtual void Handle_Click(int button,int state,int x,int y){}
  virtual void Handle_Drag(int x,int y){}
  virtual void Handle_Motion(int x,int y){}
  virtual void Handle_Idle(); // by default turns off idle callbacks to save cpu

  //helpers
  void Refresh();
  void SetFullscreen(bool fullscreen_on);
  void SleepIdleCallback(unsigned int time=INT_MAX);

  int main_window;
  int width,height; // window size
  bool fullscreen_mode;
  int saved_width,saved_height;

private:
  static GLUTProgramBase* current_program;
  static void DisplayFunc();
  static void ReshapeFunc(int w,int h);
  static void KeyboardFunc(unsigned char key,int x,int y);
  static void KeyboardUpFunc(unsigned char key,int x,int y);
  static void SpecialFunc(int key,int x,int y);
  static void SpecialUpFunc(int key,int x,int y);
  static void MouseFunc(int button,int state,int x,int y);
  static void MotionFunc(int x,int y);
  static void PassiveMotionFunc(int x,int y);
  static void IdleFunc();
};

#endif

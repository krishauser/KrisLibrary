#include <KrisLibrary/Logger.h>
#include "GLUTNavigationProgram.h"
#include "GL.h"
#include "GLView.h"
#include <iostream>
#include <string>
using namespace GLDraw;

#if HAVE_GLUT

#if defined (__APPLE__) || defined (MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif //Apple

using namespace std;

/*
  Code for doing wireframe drawings
  if(fill_mode == DRAW_FILLED)
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
  else if(fill_mode == DRAW_WIREFRAME){
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    if(!enable_lighting_for_wireframe){ 
      glDisable(GL_LIGHTING);wireframe_color.setCurrentGL();}}
  else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0,1.0);
    RenderWorld();
    glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    if(!enable_lighting_for_wireframe){ 
      glDisable(GL_LIGHTING); wireframe_color.setCurrentGL();}}
*/

#define SHOW_VIEW_TARGET(secs) {\
  show_view_target=1; \
  t_hide_view_target=timer.LastElapsedTime()+secs; \
  SleepIdleCallback(0);				   \
}


GLUTNavigationProgram::GLUTNavigationProgram()
  :oldmousex(0),oldmousey(0),
   clickButton(-1),clickModifiers(0),
   stereo_mode(false),stereo_offset(.02),
   show_view_target(0),t_hide_view_target(0),
   frames_per_second(0),show_frames_per_second(false),
   frames_rendered(0),
   mode_2d(false)
{
  //const int control=-'a'+1; // only works reliably for lowercase letters
  camera.tgt.setZero();
  camera.rot.setZero();
  camera.dist=100;
  camera.ori=Camera::Camera::XZnY;
}

bool GLUTNavigationProgram::Initialize()
{
  if(!GLUTProgramBase::Initialize()) return false;
  //glClearColor(0,0,0,0);
  glClearDepth(1);
  timer.Reset();
  return true;
}

void GLUTNavigationProgram::Handle_Display()
{
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //SetWorldLights();

  camera.toCamera(viewport);

  GLView view;
  view.setViewport(viewport);
  view.setCurrentGL();
  SetWorldLights();

  if(stereo_mode){
    glColorMask(GL_FALSE,GL_TRUE,GL_TRUE,GL_TRUE); // draw only to green, blue and alpha
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glTranslatef(-stereo_offset*camera.dist,0,0);
    RenderWorld();
    glPopMatrix();
    glClear(GL_DEPTH_BUFFER_BIT); // leave the blue image but clear Z (NOTE: may need to clear alpha as well for transparency effects!)
    glColorMask(GL_TRUE,GL_FALSE,GL_FALSE,GL_TRUE); // draw only to red and alpha
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glTranslatef(stereo_offset*camera.dist,0,0);
    RenderWorld();
    glPopMatrix();
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);}
  else {
    RenderWorld();
    if(show_view_target) DisplayCameraTarget();
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,viewport.w,viewport.h,0,-100,100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  RenderScreen();

  ++frames_rendered;
  glutSwapBuffers();
  SleepIdleCallback(0);
}

void GLUTNavigationProgram::Handle_Reshape(int w,int h)
{
  width=viewport.w=w;
  height=viewport.h=h;
  glViewport(0,0,(GLsizei)width,(GLsizei)height);
  Refresh();
}

void GLUTNavigationProgram::Handle_Keypress(unsigned char key,int x,int y)
{
}

void GLUTNavigationProgram::BeginDrag(int x,int y,int button,int modifiers)
{
  SHOW_VIEW_TARGET(1);
}

void GLUTNavigationProgram::EndDrag(int x,int y,int button,int modifiers)
{
  clickButton = -1;
}

void GLUTNavigationProgram::DoDrag(int dx,int dy,int button,int modifiers)
{
  if(button != -1) {
    int ctrl_pressed = modifiers&GLUT_ACTIVE_CTRL;
    int alt_pressed = modifiers&GLUT_ACTIVE_ALT;
    int shift_pressed = modifiers&GLUT_ACTIVE_SHIFT;

    if(ctrl_pressed) DoCtrlDrag(dx,dy,button);
    else if(alt_pressed) DoAltDrag(dx,dy,button);
    else if(shift_pressed) DoShiftDrag(dx,dy,button);
    else DoFreeDrag(dx,dy,clickButton);
    Refresh();    
  }
}

void GLUTNavigationProgram::DoFreeDrag(int dx,int dy,int button)
{
  switch(button){
  case GLUT_LEFT_BUTTON:
    if (mode_2d) DragPan(dx,dy);
    else  DragRotate(dx,dy);
    break;
  case GLUT_RIGHT_BUTTON:
    DragTruck(dx,dy);
    break;
  case GLUT_MIDDLE_BUTTON:
    DragPan(dx,dy);
    break;
  }
}

void GLUTNavigationProgram::DoCtrlDrag(int dx,int dy,int button)
{
  switch(button) {
  case GLUT_LEFT_BUTTON:
    DragPan(dx,dy);
    break;
  case GLUT_RIGHT_BUTTON:
    DragZoom(dx,dy);
    break;
  default:
    return;
  }
}

void GLUTNavigationProgram::DoAltDrag(int dx,int dy,int button)
{}

void GLUTNavigationProgram::DoShiftDrag(int dx,int dy,int button)
{}


void GLUTNavigationProgram::DragPan(int dx,int dy)
{
  Vector3 v;
  viewport.getMovementVectorAtDistance(-dx,dy,camera.dist,v);
  camera.tgt+=v;
  //viewport.scroll(dx,dy);
  SHOW_VIEW_TARGET(0.5);
}

void GLUTNavigationProgram::DragRotate(int dx,int dy)
{
  camera.rot.y-=DtoR((Real)dx);
  camera.rot.x-=DtoR((Real)dy);
  SHOW_VIEW_TARGET(0.5);
}

void GLUTNavigationProgram::DragZoom(int dx,int dy)
{
  viewport.scale *= (1+float(dy)*0.01);
  SHOW_VIEW_TARGET(0.5);
}

void GLUTNavigationProgram::DragTruck(int dx,int dy)
{
  Vector3 v(viewport.zDir());
  //camera.tgt.madd(v,Real(dy)/viewport.scale/**camera.dist*/);
  camera.dist *= (1+float(dy)*0.01);
  SHOW_VIEW_TARGET(0.5);
}


void GLUTNavigationProgram::Handle_Click(int button,int state,int x,int y)
{
  oldmousex=x;
  oldmousey=y;
  clickModifiers = glutGetModifiers();
  clickButton = button;
  if(state == GLUT_UP) {
    EndDrag(x,y,clickButton,clickModifiers);
    clickButton=-1;
  }
  else {
    BeginDrag(x,y,clickButton,clickModifiers);
  }
}

void GLUTNavigationProgram::Handle_Drag(int x,int y)
{
  int dx=x-oldmousex,dy=y-oldmousey;
  DoDrag(dx,dy,clickButton,clickModifiers);
  oldmousex=x;
  oldmousey=y;
}

void GLUTNavigationProgram::Handle_Idle()
{
  double old_time = timer.LastElapsedTime();
  double new_time = timer.ElapsedTime();
  double delta_time=new_time-old_time;
  if(delta_time>0)
    frames_per_second = 1.0/delta_time;
  else
    frames_per_second = 0.0;

  if(show_view_target && t_hide_view_target <= new_time) {
    show_view_target = 0;
    Refresh();
  }
  if(t_hide_view_target <= new_time) 
    SleepIdleCallback();
  else 
    SleepIdleCallback((int)((t_hide_view_target - new_time)*1000));
}

void GLUTNavigationProgram::DisplayCameraTarget()
{
  glMatrixMode (GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(camera.tgt.x,camera.tgt.y,camera.tgt.z);
  glDisable(GL_LIGHTING);
  GLfloat clear_color[4];
  glGetFloatv(GL_COLOR_CLEAR_VALUE,clear_color);
  glColor3f(1-clear_color[0],1-clear_color[1],1-clear_color[2]);
  float logcd=log(camera.dist)/log(20.);
  float smallsize=pow(20.0,floor(logcd-.5));
  glutWireCube(smallsize);
  glutWireCube(20*smallsize);
  glEnable(GL_LIGHTING); 
  glPopMatrix();
}
/*
  void GLUTNavigationProgram::
  Display_Strings() 
  {
  int i;
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  int g_Width=width, g_Height=height;
  gluOrtho2D(0,g_Width,0,g_Height);
  g_Height-=18;

  if (show_frames_per_second){
  glColor3f(1,1,0);
  char fps[10];sprintf(fps,"%dfps",frames_per_second);
  OpenGL_String(VECTOR_2D<float>(0,g_Height), fps, GLUT_BITMAP_HELVETICA_18);
  g_Height-=18;
  }
  glColor3f(1,1,1);
  for(i=1; i<= strings_to_print.m; i++){
  OpenGL_String(VECTOR_2D<float>(0,g_Height), strings_to_print(i), GLUT_BITMAP_HELVETICA_18);
  g_Height-=18;
  }
	
  // set openGL states back to the way they were
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  }*/


void GLUTNavigationProgram::Set2DMode(bool mode)
{
  mode_2d=mode;
}

void GLUTNavigationProgram::CenterCameraOn(const AABB3D& aabb)
{
  aabb.getMidpoint(camera.tgt);
}

void GLUTNavigationProgram::WriteDisplaySettings(ostream& out) const
{
  out<<viewport<<endl;
  out<<"ORBITDIST "<<camera.dist<<endl;
}

void GLUTNavigationProgram::ReadDisplaySettings(istream& in)
{
  in>>viewport;
  string str;
  in>>str;
  if(str != "ORBITDIST") { in.setstate(ios::badbit); return; }
  in>>camera.dist;

  camera.fromCamera(viewport,camera.dist);
  glutReshapeWindow(viewport.w,viewport.h);
}



#else //HAVE_GLUT

using namespace std;

GLUTNavigationProgram::GLUTNavigationProgram()
  :oldmousex(0),oldmousey(0),
   clickButton(-1),clickModifiers(0),
   stereo_mode(false),stereo_offset(.02f),
   show_view_target(0),t_hide_view_target(0),
   frames_per_second(0),show_frames_per_second(false),
   frames_rendered(0),
   mode_2d(false)
{
  //const int control=-'a'+1; // only works reliably for lowercase letters
  camera.tgt.setZero();
  camera.rot.setZero();
  camera.dist=100;
  camera.ori=Camera::Camera::XZnY;
}

bool GLUTNavigationProgram::Initialize()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
  return true;
}

void GLUTNavigationProgram::Handle_Display()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::Handle_Reshape(int w,int h)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::Handle_Keypress(unsigned char key,int x,int y)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::BeginDrag(int x,int y,int button,int modifiers)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::EndDrag(int x,int y,int button,int modifiers)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DoDrag(int dx,int dy,int button,int modifiers)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DoFreeDrag(int dx,int dy,int button)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DoCtrlDrag(int dx,int dy,int button)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DoAltDrag(int dx,int dy,int button)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DoShiftDrag(int dx,int dy,int button)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}


void GLUTNavigationProgram::DragPan(int dx,int dy)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DragRotate(int dx,int dy)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DragZoom(int dx,int dy)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DragTruck(int dx,int dy)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}


void GLUTNavigationProgram::Handle_Click(int button,int state,int x,int y)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::Handle_Drag(int x,int y)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::Handle_Idle()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::DisplayCameraTarget()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::Set2DMode(bool mode)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::CenterCameraOn(const AABB3D& aabb)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::WriteDisplaySettings(ostream& out) const
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}

void GLUTNavigationProgram::ReadDisplaySettings(istream& in)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUT not defined");
}


#endif // HAVE_GLUT

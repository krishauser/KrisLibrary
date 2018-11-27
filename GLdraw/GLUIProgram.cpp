#include <KrisLibrary/Logger.h>
#include "GLUIProgram.h"

#if HAVE_GLUI

#include "GL.h"
#include "GLError.h"

#if defined (__APPLE__) || defined (MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif // Apple

#if FREEGLUT
#include <GL/freeglut_ext.h>
#endif //FREEGLUT


#include <GL/glui.h>

#include <assert.h>
#include <stdio.h>

#define DEBUG(x) { x; }
/*
#define DEBUG(x) { \
    LOG4CXX_ERROR(KrisLibrary::logger(),"Debug begin "<<#x); \
  x; \
    LOG4CXX_ERROR(KrisLibrary::logger(),"Debug end "<<#x); \
}
*/

GLUIProgramBase::GLUIProgramBase(int w,int h)
:main_window(0),width(w),height(h),fullscreen_mode(false)
{}

GLUIProgramBase* GLUIProgramBase::current_program=NULL;
void GLUIProgramBase::DisplayFunc() {
  DEBUG(current_program->Handle_Display());
  CheckGLErrors("DisplayFunc",false);
}
void GLUIProgramBase::ReshapeFunc(int w,int h) { DEBUG(current_program->Handle_Reshape(w,h)); }
void GLUIProgramBase::KeyboardFunc(unsigned char c,int x,int y) {
  DEBUG(current_program->Handle_Keypress(c,x,y));
  if(GLUT_API_VERSION < 4)
    DEBUG(current_program->Handle_KeypressUp(c,x,y));
}
void GLUIProgramBase::KeyboardUpFunc(unsigned char c,int x,int y) { DEBUG(current_program->Handle_KeypressUp(c,x,y)); }
void GLUIProgramBase::SpecialFunc(int key,int x,int y) {
  DEBUG(current_program->Handle_Special(key,x,y));
  if(GLUT_API_VERSION < 4)
    DEBUG(current_program->Handle_SpecialUp(key,x,y));
}
void GLUIProgramBase::SpecialUpFunc(int key,int x,int y) { DEBUG(current_program->Handle_SpecialUp(key,x,y)); }
void GLUIProgramBase::MouseFunc(int button,int state,int x,int y) { DEBUG(current_program->Handle_Click(button,state,x,y)); }
void GLUIProgramBase::MotionFunc(int x,int y) { DEBUG(current_program->Handle_Drag(x,y)); }
void GLUIProgramBase::PassiveMotionFunc(int x,int y) { DEBUG(current_program->Handle_Motion(x,y)); }
void GLUIProgramBase::IdleFunc() 
{ 
  glutSetWindow(current_program->main_window);
  DEBUG(current_program->Handle_Idle());
}
void GLUIProgramBase::ControlFunc (int id) { current_program->Handle_Control(id); }

int GLUIProgramBase::Run(const char *window_title,unsigned int mode)
{
	current_program=this;
	int argc=1;char *(argv[1]);argv[0]="Program";
	glutInit(&argc,(char**)argv);
	if(mode == 0) {
	  mode=GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH;
#if (defined GL_MULTISAMPLE || defined GL_MULTISAMPLE_ARB)
 	  mode|=GLUT_MULTISAMPLE;
#if FREEGLUT
	  glutSetOption(GLUT_MULTISAMPLE, 4);
#endif // FREEGLUT
#endif //GL_MULTISAMPLE_ARB
	}
	glutInitDisplayMode(mode);
	glutInitWindowSize(width,height);
	main_window = glutCreateWindow(window_title);
	GLUI_Master.set_glutDisplayFunc(DisplayFunc);
	GLUI_Master.set_glutReshapeFunc(ReshapeFunc);
	GLUI_Master.set_glutKeyboardFunc(KeyboardFunc);
	GLUI_Master.set_glutSpecialFunc(SpecialFunc);
	GLUI_Master.set_glutMouseFunc(MouseFunc);
	GLUI_Master.set_glutIdleFunc(IdleFunc);

	glutMotionFunc(MotionFunc);
	glutPassiveMotionFunc(PassiveMotionFunc);
#if GLUT_API_VERSION >= 4
	glutKeyboardUpFunc(KeyboardUpFunc);
	glutSpecialUpFunc(SpecialUpFunc);
#endif //GLUT_API_VERSION

	if(!Initialize()) return -1;

	glutMainLoop();
	return 0;
}

bool GLUIProgramBase::Initialize()
{
	glEnable (GL_DEPTH_TEST);
	glEnable (GL_CULL_FACE);

#ifdef GL_MULTISAMPLE
	glEnable(GL_MULTISAMPLE);
#elif defined GL_MULTISAMPLE_ARB
	glEnable(GL_MULTISAMPLE_ARB);
#endif //GL_MULTISAMPLE
	return true;
}

void GLUIProgramBase::Refresh()
{
	glutPostRedisplay();
}

void GLUIProgramBase::SetFullscreen(bool fullscreen_on)
{
	if(fullscreen_mode != fullscreen_on) {
		fullscreen_mode=fullscreen_on;
		if(fullscreen_mode) { glutFullScreen(); saved_width=width; saved_height=height; }
		else glutReshapeWindow(saved_width,saved_height);
	}
}

void GLUIProgramBase::Handle_Idle()
{
  SleepIdleCallback();
}

typedef void (*VOIDFUNC)();
extern VOIDFUNC oldIdleFunc;
void enable_idle_func_glui(int)
{
  GLUI_Master.set_glutIdleFunc(oldIdleFunc); 
}

void GLUIProgramBase::SleepIdleCallback(unsigned int time)
{
  if(time==0) GLUI_Master.set_glutIdleFunc(IdleFunc); 
  else {
    GLUI_Master.set_glutIdleFunc(NULL); 
    oldIdleFunc=GLUIProgramBase::IdleFunc;
    glutTimerFunc(time,enable_idle_func_glui,0);
  }
}

#else  //HAVE_GLUI

#include <iostream>
using namespace std;

GLUIProgramBase::GLUIProgramBase(int w,int h)
:main_window(0),width(w),height(h),fullscreen_mode(false)
{
}

void GLUIProgramBase::DisplayFunc() {  }
void GLUIProgramBase::ReshapeFunc(int w,int h) {  }
void GLUIProgramBase::KeyboardFunc(unsigned char c,int x,int h) {  }
void GLUIProgramBase::SpecialFunc(int key,int x,int h) {  }
void GLUIProgramBase::MouseFunc(int button,int state,int x,int y) {  }
void GLUIProgramBase::MotionFunc(int x,int y) { }
void GLUIProgramBase::PassiveMotionFunc(int x,int y) {  }
void GLUIProgramBase::IdleFunc() { }

void GLUIProgramBase::ControlFunc (int id) { 
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
}

int GLUIProgramBase::Run(const char *window_title,unsigned int mode)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
  return -1;
}

bool GLUIProgramBase::Initialize()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
	return false;
}

void GLUIProgramBase::Refresh()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
}

void GLUIProgramBase::SetFullscreen(bool fullscreen_on)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
}

void GLUIProgramBase::Handle_Idle()
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
}

void GLUIProgramBase::SleepIdleCallback(unsigned int time)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, GLUI not defined");
}


#endif //HAVE_GLUI=0

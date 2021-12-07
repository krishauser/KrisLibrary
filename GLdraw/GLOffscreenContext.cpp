#include "GLOffscreenContext.h"

using namespace GLDraw;

#if __linux__

#include <stdio.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/glew.h>
#include <GL/gl.h>
//#include <GL/glxew.h>
#include <GL/glx.h>

#define OPENGL2_HACK 1

typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
static glXMakeContextCurrentARBProc glXMakeContextCurrentARB = 0;

struct GLOffscreenContextXData
{
    Display* dpy;
    GLXContext ctx;
    GLXPbuffer pbuf;
    GLXContext priorCtx;
};

GLOffscreenContext::GLOffscreenContext()
{
    auto xdata = new GLOffscreenContextXData();
    data = xdata; 
    xdata->dpy = NULL;
    xdata->ctx = 0;
    xdata->pbuf = 0;
}

GLOffscreenContext::~GLOffscreenContext()
{
    destroy();
    GLOffscreenContextXData* xdata = (GLOffscreenContextXData*)data;
    delete xdata;
}


void GLOffscreenContext::destroy()
{
    GLOffscreenContextXData* xdata = (GLOffscreenContextXData*)data;
    if(xdata->pbuf) 
        glXDestroyPbuffer(xdata->dpy,xdata->pbuf);
    if(xdata->ctx)
        glXDestroyContext(xdata->dpy,xdata->ctx);
    xdata->ctx = 0;
    xdata->pbuf = 0;
}

bool GLOffscreenContext::setup()
{
    GLOffscreenContextXData* xdata = (GLOffscreenContextXData*)data;
    Display*& dpy = xdata->dpy;
    GLXContext& ctx = xdata->ctx;
    GLXPbuffer& pbuf = xdata->pbuf;
    xdata->priorCtx = glXGetCurrentContext();

    //what do these do?
    int pbuffer_width = 32;
    int pbuffer_height = 32;

    static int visual_attribs[] = {
        GLX_RENDER_TYPE, GLX_RGBA_BIT,
        GLX_RED_SIZE, 8,
        GLX_GREEN_SIZE, 8,
        GLX_BLUE_SIZE, 8,
        GLX_ALPHA_SIZE, 8,
        GLX_DEPTH_SIZE, 24,
        GLX_STENCIL_SIZE, 8,
        None
    };

    dpy = XOpenDisplay(0);
    int fbcount = 0;
    GLXFBConfig* fbc = NULL;
    
    /* open display */
    if ( ! (dpy = XOpenDisplay(0)) ){
        fprintf(stderr, "GLOffscreenContext: Failed to open display\n");
        return false;
    }

    /* get framebuffer configs, any is usable (might want to add proper attribs) */
    if ( !(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount) ) ){
        fprintf(stderr, "GLOffscreenContext: Failed to get FBConfig\n");
        return false;
    }
    
    /* get the required extensions */
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB");
    glXMakeContextCurrentARB = (glXMakeContextCurrentARBProc)glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent");
    if ( !(glXCreateContextAttribsARB && glXMakeContextCurrentARB) ){
        fprintf(stderr, "GLOffscreenContext: missing support for GLX_ARB_create_context\n");
        XFree(fbc);
        return false;
    }

    #if OPENGL2_HACK
        /*
        static int  sngBuf[] = {    GLX_RGBA,
                    GLX_RED_SIZE, 1,
                    GLX_GREEN_SIZE, 1,
                    GLX_BLUE_SIZE, 1,
                    GLX_DEPTH_SIZE, 12,
                    None };
        XVisualInfo* vi;
        if(!(vi = glXChooseVisual(dpy, DefaultScreen(dpy), sngBuf))) {
            fprintf(stderr, "Failed to get visual info\n");
            XFree(fbc);
            return false;
        }
        */
        XVisualInfo *vi = glXGetVisualFromFBConfig( dpy, fbc[0] );
        if ( !vi )
        {
            fprintf(stderr, "GLOffscreenContext: Failed to get visual info\n");
            XFree(fbc);
            return false;
        }
        if ( !( ctx = glXCreateContext(dpy, vi, 0, True)) ){
            fprintf(stderr, "GLOffscreenContext: Failed to create opengl context\n");
            XFree(fbc);
            XFree( vi ); 
            return false;
        }
        XFree( vi ); 
    #else
        int context_attribs[] = {
            GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
            GLX_CONTEXT_MINOR_VERSION_ARB, 2,
            None
        };
        /* create a context using glXCreateContextAttribsARB */
        if ( !( ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs)) ){
            fprintf(stderr, "Failed to create opengl context\n");
            XFree(fbc);
            return false;
        }
    #endif // OPENGL2_HACK

    /* create temporary pbuffer */
    int pbuffer_attribs[] = {
        GLX_PBUFFER_WIDTH, pbuffer_width,
        GLX_PBUFFER_HEIGHT, pbuffer_height,
        None
    };
    pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);

    XFree(fbc);
    XSync(dpy, False);

    /* try to make it the current context */
    if ( !glXMakeContextCurrent(dpy, pbuf, pbuf, ctx) ){
        /* some drivers does not support context without default framebuffer, so fallback on
        * using the default window.
        */
        if ( !glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx) ){
            fprintf(stderr, "GLOffscreenContext: failed to make current\n");
            return false;
        }
    } else {
        //printf("Init completed.\n");
    }

    printf("OpenGL vendor: %s\n", (const char*)glGetString(GL_VENDOR));
    //Works in OpenGL 2.0 but not 3.0
    //printf("Extensions: %s\n", (const char*)glGetString(GL_EXTENSIONS));
    /*
    GLint n;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n);
    if (n > 0)
    {
        printf("%d extensions\n",n);
        GLint i;
        for (i = 0; i < n; i++)
        {
            printf("%d\n",(int)i);
            printf("  %s\n",(const char*)glGetStringi(GL_EXTENSIONS, i));
        }
    }
    */
    glewInit();
    //printf("%p\n",glGenRenderbuffers);
    return true;
}

bool GLOffscreenContext::makeCurrent()
{
    GLOffscreenContextXData* xdata = (GLOffscreenContextXData*)data;
    Display*& dpy = xdata->dpy;
    GLXContext& ctx = xdata->ctx;
    GLXPbuffer& pbuf = xdata->pbuf;
    if(!pbuf) return false;
    if(!ctx) return false;
    /* try to make it the current context */
    if ( !glXMakeContextCurrent(dpy, pbuf, pbuf, ctx) ){
        /* some drivers does not support context without default framebuffer, so fallback on
        * using the default window.
        */
        if ( !glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx) ){
            return false;
        }
    } else {
    }
    return true;
}

bool GLOffscreenContext::hasGLContext()
{
    return glXGetCurrentContext() != NULL;
}

#else

#include "GLUTProgram.h"

GLOffscreenContext::GLOffscreenContext()
{
}

GLOffscreenContext::~GLOffscreenContext()
{
}


void GLOffscreenContext::destroy()
{
}

bool GLOffscreenContext::setup()
{
    return false;
}

bool GLOffscreenContext::makeCurrent()
{
    return false;
}

bool GLOffscreenContext::hasGLContext()
{
    return false;
}

#endif //__linux__
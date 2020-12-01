#ifndef _GLDRAW_OFFSCREEN_CONTEXT_H
#define _GLDRAW_OFFSCREEN_CONTEXT_H

namespace GLDraw {

/** @ingroup GLDraw
 * @brief A way to draw to an offscreen buffer.  Currently only works with XWindows.
 *
 * To play nicely with other methods of setting up a context, you can first call the
 * hasGLContext static function to check whether there's another context available, 
 * and possibly use that instead. If you really want to use multiple contexts, you
 * should restore your other context after you are done rendering.  (This is highly
 * platform dependent, and may not be available with GLUT).
 */
class GLOffscreenContext
{
public:
    GLOffscreenContext();
    ~GLOffscreenContext();
    bool setup();
    void destroy();
    bool makeCurrent();

    static bool hasGLContext();

    void* data; //opaque pointer
};

} //namespace GLDraw

#endif


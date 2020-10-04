#ifndef _GLDRAW_OFFSCREEN_CONTEXT_H
#define _GLDRAW_OFFSCREEN_CONTEXT_H

namespace GLDraw {

/** @ingroup GLDraw
 * @brief A way to draw to an offscreen buffer.  Currently only works with XWindows.
 */
class GLOffscreenContext
{
public:
    GLOffscreenContext();
    ~GLOffscreenContext();
    bool setup();
    void destroy();
    bool makeCurrent();

    void* data; //opaque pointer
};

} //namespace GLDraw

#endif


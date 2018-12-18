#ifndef GL_RENDER_IMAGE_H
#define GL_RENDER_IMAGE_H

#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/image/image.h>
#include <vector>

namespace GLDraw {

/** @ingroup GLDraw
 * @brief Allows for GL calls to draw to an image
 *
 * Usage:
 * glrender = GLRenderToImage();
 * if(!glrender.Setup(640,480)) 
 *    error...
 *
 * ... do more setup of the viewport ...
 * Viewport vp; 
 * ... set up the viewport ...
 * glrender.Begin(vp)
 * ... do openGL calls ...
 * glrender.End();
 * Image image;
 * glrender.GetRGBA(image);
 * ... now image has the rendered image data.
 **/
class GLRenderToImage
{
public:
  GLRenderToImage();
  ~GLRenderToImage();

  ///Call this first to see if render to image is supported
  bool Setup(int w,int h);
  ///Call Begin before rendering
  void Begin(const Camera::Viewport& vp);
  ///Call Begin before rendering
  void Begin();
  ///Call End after rendering
  void End();

  ///scan line order, top left to bottom right
  void GetRGBA(std::vector<unsigned int>& image);
  ///pixels in scan line order in RGBA format, top left to bottom right
  void GetRGBA(std::vector<unsigned char>& image);
  ///pixels in matrix order, top left to bottom right
  void GetRGBA(std::vector<std::vector<unsigned int> >& image); 
  ///converts to KrisLibrary Image
  void GetRGBA(Image& image); 

  ///scan line order, top left to bottom right
  void GetDepth(const Camera::Viewport& vp,std::vector<float>& image);
  ///pixels in matrix order, top left to bottom right
  void GetDepth(const Camera::Viewport& vp,std::vector<std::vector<float> >& image);
  ///converts to KrisLibrary Image
  void GetDepth(const Camera::Viewport& vp,Image& image);

  ///this is the raw value in the z buffer
  void GetZBuffer(std::vector<float>& image);

  int width,height;
  unsigned int color_tex,fb,depth_rb;
};

} 

#endif //GL_RENDER_IMAGE_H
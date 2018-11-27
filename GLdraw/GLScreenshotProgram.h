#ifndef GL_SCREENSHOT_PROGRAM_H
#define GL_SCREENSHOT_PROGRAM_H

#include <KrisLibrary/Logger.h>
#include "GLScreenshot.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/Timer.h>
#include <string>

namespace GLDraw {

/** @ingroup GLDraw
 * @brief A plugin class that "automatically" saves a movie to disk in the form of
 * PPM screenshots.
 *
 * To save the screenshots, set #saveMovie to true and call MovieUpdate() during your
 * idle loop.
 *
 * By default, the frame rate is 30fps and the screenshot file format is image[xxxx].ppm
 * where [xxxx] is the frame number.  To change the fps, set #frameTime to 1.0/fps.
 * To use another file format, set #screenshotFile to the desired name of the initial
 * frame.  The program will automatically increment the digits in #screenshotFile,
 * keeping the same number of leading zeroes if possible.
 */
template <class BaseGUI>
class GLScreenshotProgram : public BaseGUI
{
public:
  std::string screenshotFile;
  bool saveMovie;
  double lastScreenshotTime;
  double frameTime;
  Timer timer;
  int verbose;

  GLScreenshotProgram()
  {
    saveMovie = false;
    lastScreenshotTime = 0;
    frameTime = 1.0/30.0;
    screenshotFile = "image0000.ppm";
    verbose = 1;
  }

  virtual ~GLScreenshotProgram() {}

  void SaveScreenshot()
  {
    GLSaveScreenshotPPM(screenshotFile.c_str());
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Screenshot saved to "<<screenshotFile.c_str());
  }

  void StartMovie()
  {
    saveMovie = true;
    lastScreenshotTime = 0;
    timer.Reset();
  }

  void StopMovie()
  {
    saveMovie = false;
  }

  void ToggleMovie()
  {
    if(saveMovie) StopMovie();
    else StartMovie();
  }

  //call this if you want a real-time movie
  void MovieUpdate()
  {
    this->MovieUpdate(timer.ElapsedTime());
  }

  //call this if you want to control movie time
  void MovieUpdate(double t)
  {
    if(saveMovie) {
      if(t >= lastScreenshotTime + frameTime) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Time "<<t<<" last "<<lastScreenshotTime<<", Saving "<<(int)Floor((t-lastScreenshotTime)/frameTime));
	while(lastScreenshotTime+frameTime < t) {
	  SaveScreenshot();
	  IncrementStringDigits(screenshotFile);
	  lastScreenshotTime += frameTime;
	}
      }
    }
  }
};

} //namespace GLDraw

#endif

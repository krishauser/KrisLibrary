#ifndef GL_SCREENSHOT_H
#define GL_SCREENSHOT_H

/** @ingroup GLDraw
 * @brief Saves the current back buffer to filename. 
 *
 * If GDI is available (windows) saves to whatever file type is 
 * given by filename.  Otherwise this saves in PPM format.
 */
bool GLSaveScreenshot(const char *filename);
bool GLSaveScreenshotPPM(const char *filename);

#endif

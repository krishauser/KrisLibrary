#ifndef IMAGE_IMPORT_H
#define IMAGE_IMPORT_H

#include "image.h"

bool ImportImage(const char* fn, Image& img);
bool ImportImagePPM(const char* fn, Image& img);
bool ImportImageBMP(const char* fn, Image& img);
bool ImportImageTGA(const char* fn, Image& img);
bool ImportImageGDIPlus(const char* fn, Image& img);
///Returns a semicolon-separated list of supported image types.
///This may not be exhaustive.
const char* ImageImportTypes();

#endif

#ifndef IMAGE_IMPORT_H
#define IMAGE_IMPORT_H

#include "image.h"

bool ImportImage(const char* fn, Image& img);
bool ImportImageBMP(const char* fn, Image& img);
bool ImportImageTGA(const char* fn, Image& img);
bool ImportImageGDIPlus(const char* fn, Image& img);
const char* ImageImportTypes();

#endif

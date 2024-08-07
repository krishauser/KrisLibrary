#ifndef IMAGE_IMPORT_EXPORT_H
#define IMAGE_IMPORT_EXPORT_H

#include "image.h"

bool ImportImage(const char* fn, Image& img);
bool ImportImagePPM(const char* fn, Image& img);
bool ImportImageBMP(const char* fn, Image& img);
bool ImportImageTGA(const char* fn, Image& img);
bool ImportImageGDIPlus(const char* fn, Image& img);
///Returns a semicolon-separated list of supported image types.
///This may not be exhaustive.
const char* ImageImportTypes();

bool ExportImage(const char* fn, const Image& img);
bool ExportImagePPM(const char* fn, const Image& img);
bool ExportImageGDIPlus(const char* fn, const Image& img);
///Returns a semicolon-separated list of supported image types.
///This may not be exhaustive.
const char* ImageExportTypes();

#endif

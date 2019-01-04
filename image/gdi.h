#ifndef IMAGE_GDI_H
#define IMAGE_GDI_H

#ifdef NOMINMAX
#undef NOMINMAX
#endif // NOMINMAX
#include <windows.h>
#include <GdiPlus.h>
#include "image.h"

//fmtOut gets the closest possible GDI+ format to be written to the image
Image::PixelFormat GdiToImagePixelFormat(Gdiplus::PixelFormat fmt, Gdiplus::PixelFormat& fmtOut);
Gdiplus::PixelFormat ImageGdiPixelFormat(Image::PixelFormat fmt);
void GdiBitmapToImage(Gdiplus::Bitmap& bit, Image& img);
Gdiplus::Bitmap* ImageToGdiBitmap(const Image& img);

bool ExportImageGDIPlus(const char* fn, Image& img);

#endif
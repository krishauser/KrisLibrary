#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif //WIN32
#include "image.h"

#ifdef WIN32
bool LoadImageFromBitmap(HDC hdc, HBITMAP hbit, Image& image);
#endif //WIN32
bool IsSquare(const Image& tex);
void StretchSquare(Image& tex);
void ExpandSquare(Image& tex);


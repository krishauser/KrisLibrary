#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "image.h"

bool LoadImageFromBitmap(HDC hdc, HBITMAP hbit, Image& image);
bool IsSquare(const Image& tex);
void StretchSquare(Image& tex);
void ExpandSquare(Image& tex);
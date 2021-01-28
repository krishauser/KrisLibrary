#include <KrisLibrary/Logger.h>
#include "gdi.h"
#include <errors.h>
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/utils/stringutils.h>

class GdiplusStartupHelper
{
public:
  GdiplusStartupHelper() {
    gdiplusToken = 0;
    tried_startup = false;
  }
  ~GdiplusStartupHelper() {
    if (gdiplusToken != 0) {
      Gdiplus::GdiplusShutdown(gdiplusToken);
    }
  }
  bool tried() {
    return tried_startup;
  }
  bool startup() {
    tried_startup = true;
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    Gdiplus::Status res = Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    return res == Gdiplus::Ok;
  }
  ULONG_PTR gdiplusToken;
  bool tried_startup;
};

GdiplusStartupHelper gdiplusStartupHelper;

Image::PixelFormat GdiToImagePixelFormat(Gdiplus::PixelFormat fmt, Gdiplus::PixelFormat& fmtOut)
{
	fmtOut = fmt;  //unless we need a conversion
	switch(fmt)
	{
	case PixelFormat1bppIndexed:
	case PixelFormat4bppIndexed:
	case PixelFormat8bppIndexed:
		fmtOut = PixelFormat32bppARGB;
		return Image::B8G8R8A8;
	case PixelFormat16bppARGB1555:
		return Image::R5G5B5X1;
	case PixelFormat16bppGrayScale:
		fmtOut = PixelFormat32bppARGB;
		return Image::B8G8R8A8;
	case PixelFormat16bppRGB555:
		return Image::R5G5B5X1;
	case PixelFormat16bppRGB565:
		return Image::R5G6B5;
	case PixelFormat24bppRGB:
		return Image::B8G8R8;
	case PixelFormat32bppARGB:
	case PixelFormat32bppPARGB: 
	case PixelFormat48bppRGB:
	case PixelFormat32bppRGB:
	case PixelFormat64bppARGB: 
	case PixelFormat64bppPARGB:
		fmtOut = PixelFormat32bppARGB;
		return Image::B8G8R8A8;
	default:
		fmtOut = PixelFormat32bppARGB;
		return Image::B8G8R8A8;
	}
}

Gdiplus::PixelFormat ImageToGdiPixelFormat(Image::PixelFormat fmt)
{
	switch(fmt) {
	case Image::None: return PixelFormat1bppIndexed;	
	case Image::R8G8B8: return PixelFormat24bppRGB;
  case Image::B8G8R8: return PixelFormat24bppRGB;
	case Image::R8G8B8A8: return PixelFormat32bppARGB;
  case Image::B8G8R8A8: return PixelFormat32bppARGB;
	case Image::R5G6B5: return PixelFormat16bppRGB565;
	case Image::R5G5B5X1: return PixelFormat16bppRGB555;
	case Image::A8: return PixelFormat8bppIndexed;
	default:
		FatalError("Thats an invalid GDI format");
		return PixelFormat1bppIndexed;	
	}
}

void GdiBitmapToImage(Gdiplus::Bitmap& bit, Image& img)
{
	Gdiplus::PixelFormat bmpFormat;
	Image::PixelFormat imageFormat;
	imageFormat=GdiToImagePixelFormat(bit.GetPixelFormat(),bmpFormat);

	Gdiplus::Rect rect;
	Gdiplus::RectF rectf;
	Gdiplus::Unit units;
	Gdiplus::BitmapData bitdata;
	bit.GetBounds(&rectf,&units);
	rect.X = (int)rectf.X;
	rect.Y = (int)rectf.Y;
	rect.Width = (int)rectf.Width;
	rect.Height = (int)rectf.Height;
	bit.LockBits(&rect,Gdiplus::ImageLockModeRead,bmpFormat,&bitdata);

	img.initialize(bitdata.Width,bitdata.Height,imageFormat);
	unsigned char* data = (unsigned char*)bitdata.Scan0;
	int scanline_size = bitdata.Width*img.pixelSize();
	int pixelBytes = img.pixelSize();
	for(UINT i=0; i<bitdata.Height;i++)
	{
		if(imageFormat == Image::None || imageFormat == Image::A8) {
			memcpy(img.getData(0,i),data,scanline_size);
		}
    else if (imageFormat == Image::B8G8R8 || imageFormat == Image::B8G8R8A8) {
      memcpy(img.getData(0,i), data, scanline_size);
    }
    else {//swap order for little endian pixels
			unsigned char* dataRow  = data;
			unsigned char* imgRow = img.getData(0,i);
			for(UINT j=0;j<bitdata.Width;j++,dataRow+=pixelBytes,imgRow+=pixelBytes)
				for(int k=0;k<pixelBytes;k++)
					imgRow[k] = dataRow[pixelBytes-1-k];
		}
		data += bitdata.Stride;
	}
	bit.UnlockBits(&bitdata);
}

Gdiplus::Bitmap* ImageToGdiBitmap(const Image& image)
{
	Gdiplus::PixelFormat bmpFormat=ImageToGdiPixelFormat(image.format);
	Gdiplus::Bitmap* bit = new Gdiplus::Bitmap(image.w,image.h,bmpFormat);
	Gdiplus::Rect rect;
	Gdiplus::RectF rectf;
	Gdiplus::Unit units;
	Gdiplus::BitmapData bitdata;
	bit->GetBounds(&rectf,&units);
	rect.X = (int)rectf.X;
	rect.Y = (int)rectf.Y;
	rect.Width = (int)rectf.Width;
	rect.Height = (int)rectf.Height;
	bit->LockBits(&rect,Gdiplus::ImageLockModeRead,bmpFormat,&bitdata);
	unsigned char* data = (unsigned char*)bitdata.Scan0;
	int scanline_size = bitdata.Width*image.pixelSize();
	int pixelBytes = image.pixelSize();
	for(UINT i=0; i<bitdata.Height;i++)
	{
		if(image.format == Image::None || image.format == Image::A8) {
			memcpy(data,image.getData(0,i),scanline_size);
		}
    else if (image.format == Image::B8G8R8 || image.format == Image::B8G8R8A8) {
      memcpy(data, image.getData(0,i), scanline_size);
    }
    else if(image.format == Image::R8G8B8) { //convert from RGB to BGR
			unsigned char* dataRow  = data;
			const unsigned char* imgRow = image.getData(0,i);
			for(UINT j=0;j<bitdata.Width;j++,dataRow+=pixelBytes,imgRow+=pixelBytes)
				for(int k=0;k<pixelBytes;k++)
					dataRow[pixelBytes-1-k] = imgRow[k];
		}
    else if (image.format == Image::R8G8B8A8) { //convert from RGBA to BGRA
      unsigned char* dataRow = data;
      const unsigned char* imgRow = image.getData(0,i);
      for (UINT j = 0; j < bitdata.Width; j++, dataRow += pixelBytes, imgRow += pixelBytes) {
        dataRow[0] = imgRow[2];
        dataRow[1] = imgRow[1];
        dataRow[2] = imgRow[0];
        dataRow[3] = imgRow[3];
      }
    }
    else {
      //ehhh... whatever
      memcpy(data, image.getData(0,i), scanline_size);
    }
		data += bitdata.Stride;
	}
	bit->UnlockBits(&bitdata);
	return bit;
}

bool ImportImageGDIPlus(const char* fn, Image& img)
{
	//load the bitmap (requires wchars)
	int length = strlen(fn)+5;
	WCHAR* wfn = new WCHAR[length];
	ToWideChar(fn,wfn,length);
	Gdiplus::Bitmap bit(wfn);
	delete [] wfn;

  if (bit.GetWidth() == 0 || bit.GetHeight() == 0) {
    if (!FileUtils::Exists(fn)) {
      fprintf(stderr, "ImportImageGDIPlus: file %s doesn't exist\n", fn);
    }
    else {
      if (gdiplusStartupHelper.tried())
        return false;
      if (!gdiplusStartupHelper.startup())
        fprintf(stderr, "ImportImageGDIPlus: Couldn't start Gdiplus\n");
      else {
        //try again
        return ImportImageGDIPlus(fn, img);
      }
      return false;
    }
    return false;
  }
	GdiBitmapToImage(bit,img);
	return true;
}

int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
   UINT  num = 0;          // number of image encoders
   UINT  size = 0;         // size of the image encoder array in bytes

   Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

   Gdiplus::GetImageEncodersSize(&num, &size);
   if(size == 0)
      return -1;  // Failure

   pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
   if(pImageCodecInfo == NULL)
      return -1;  // Failure

   Gdiplus::GetImageEncoders(num, size, pImageCodecInfo);

   for(UINT j = 0; j < num; ++j)
   {
      if( wcscmp(pImageCodecInfo[j].MimeType, format) == 0 )
      {
         *pClsid = pImageCodecInfo[j].Clsid;
         free(pImageCodecInfo);
         return j;  // Success
      }    
   }

   free(pImageCodecInfo);
   return -1;  // Failure
}

bool ExportImageGDIPlus(const char* fn, Image& img)
{
	//get the proper encoder
	const char* ext=FileExtension(fn);
	if(!ext || strlen(ext) > 4) ext="bmp";
	if(0==strcmp(ext,"tif")) ext="tiff";
	char typebuf[32];
#ifndef _WIN32
	snprintf(typebuf,32,"image/"<< ext);
#else
	sprintf_s<32>(typebuf,"image/%s", ext);
#endif //_WIN32
	WCHAR wtypebuf[32];
	ToWideChar(typebuf,wtypebuf,32);
	CLSID clsid;
	if(GetEncoderClsid(wtypebuf, &clsid)==-1)
		return false;

	Gdiplus::Bitmap* bmp=ImageToGdiBitmap(img);

	//save the bitmap (requires wchars)
	int length = strlen(fn)+5;
	WCHAR* wfn = new WCHAR[length];
	ToWideChar(fn,wfn,length);
	Gdiplus::Status res = bmp->Save(wfn, &clsid, NULL);
  if (res == Gdiplus::GdiplusNotInitialized) {
    if (gdiplusStartupHelper.tried()) {
      delete[] wfn;
      delete bmp;
      return false;
    }
    if (!gdiplusStartupHelper.startup()) {
      fprintf(stderr, "ExportImageGDIPlus: Couldn't start Gdiplus\n");
      delete[] wfn;
      delete bmp;
      return false;
    }
    bmp->Save(wfn, &clsid, NULL);
  }
	delete [] wfn;
	delete bmp;
	return true;
}

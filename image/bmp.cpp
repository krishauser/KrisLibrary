#include <KrisLibrary/Logger.h>
#include "image.h"
#ifdef _WIN32
#include <windows.h>
#else
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef long LONG;
struct __attribute__((__packed__)) BITMAPFILEHEADER {
  WORD  bfType;
  DWORD bfSize;
  WORD  bfReserved1;
  WORD  bfReserved2;
  DWORD bfOffBits;
};
struct __attribute__((__packed__)) BITMAPINFOHEADER {
  DWORD biSize;
  LONG  biWidth;
  LONG  biHeight;
  WORD  biPlanes;
  WORD  biBitCount;
  DWORD biCompression;
  DWORD biSizeImage;
  LONG  biXPelsPerMeter;
  LONG  biYPelsPerMeter;
  DWORD biClrUsed;
  DWORD biClrImportant;
};
struct __attribute__((__packed__)) RGBQUAD {
  BYTE rgbBlue;
  BYTE rgbGreen;
  BYTE rgbRed;
  BYTE rgbReserved;
};
#define BI_RGB 0
#include <stdlib.h>
#include <memory.h>
#endif // _WIN32
#include "errors.h"
#include <stdio.h>
#include <assert.h>

bool ImportImageBMP(const char* fn, Image& image)
{
	//todo:
	//RLE compression decoding?

	FILE* f = fopen(fn, "rb");
	if(!f) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  return false;
	}

	BITMAPFILEHEADER bfh;
	if(fread(&bfh, sizeof(bfh), 1, f) != 1)
	{
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  fclose(f);
	  return false;
	}

	if(memcmp(&bfh.bfType, "BM", 2) != 0) 
	{
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  fclose(f);
	  return false;
	}

	BITMAPINFOHEADER bi;
	if(fread(&bi, sizeof(bi), 1, f) != 1)
	{
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  fclose(f);
	  return false;
	}

	image.w = (unsigned short)bi.biWidth;
	image.h = (unsigned short)bi.biHeight;
	image.num_bytes = bi.biSizeImage;


	switch(bi.biCompression)
	{
	case BI_RGB:
		if(image.num_bytes == 0)
			image.num_bytes = image.w * image.h * (bi.biBitCount>>3);
		break;
	default:
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn<<": Unsupported compression type 0x"<<bi.biCompression);
		fclose(f);
		return false;
	}

	int size_to_read = image.num_bytes;

	RGBQUAD palette [256];
	unsigned int palette_size = 256;
	bool decode_8 = false;

	switch(bi.biBitCount)
	{
	case 8:
		{
		if(bi.biClrUsed != 0)
			palette_size = bi.biClrUsed;
		decode_8 = true;
		if(fread(&palette, sizeof(RGBQUAD), palette_size, f) != palette_size)
		{
		  		  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
		  fclose(f);
		  return false;
		}
		image.format = Image::R8G8B8;
		image.num_bytes = image.w * image.h * 3;
		}
		break;
	case 16:
		image.format = Image::R5G6B5;
		break;
	case 24:
		image.format = Image::R8G8B8;
		break;
	default:
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  fclose(f);
	  return false;
	}
	
	fseek(f, bfh.bfOffBits, SEEK_SET);

	unsigned char* bits = new unsigned char [size_to_read];
	if(fread(bits, 1, size_to_read, f) != size_to_read)
	{
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ImportImage "<<fn);
	  fclose(f);
		delete [] bits;
		return false;
	}

	if(ftell(f) != bfh.bfSize)
	{
		int cur = ftell(f);
		fseek(f, 0, SEEK_END);
		FatalError("Um, there was some stuff missed: offset is %d, size is %d, to read is %d, current is %d, end is %d\n",
			bfh.bfOffBits, bfh.bfSize, size_to_read, cur, ftell(f));
	}

	if(image.num_bytes < image.w*image.h*image.pixelSize()) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Strange, the number of bytes is not correct for the format?\n");
	  LOG4CXX_INFO(KrisLibrary::logger(),"Bytes "<<image.num_bytes<<" should equal "<<image.w<<" x "<<image.h<<" x "<<image.pixelSize()<<" = "<<image.w*image.h*image.pixelSize());
	}
	assert(image.num_bytes >= image.w*image.h*image.pixelSize());
	image.initialize(image.w,image.h,image.format);
	if(decode_8)
	{
		unsigned char* bit = bits;
		unsigned char* pixel = image.data;
		for(int i=0; i<size_to_read; i++)
		{
			if(*bit >= palette_size)
			{
				FatalError("ImportImage BMP: out of palette range\n");
				abort();
			}
			pixel[0] = palette[*bit].rgbBlue;
			pixel[1] = palette[*bit].rgbGreen;
			pixel[2] = palette[*bit].rgbRed;
			pixel+=3;
			bit++;
		}
	}
	else
	{
		assert(image.pixelBPP() == bi.biBitCount);
		memcpy(image.data, bits, image.num_bytes);
	}


	delete [] bits;
	fclose(f);
	return true;
}

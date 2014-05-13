#ifdef WIN32
#include "image.h"
#include <windows.h>
#include "../errors.h"
#include <stdio.h>
#include <assert.h>

bool ImportImageBMP(const char* fn, Image& image)
{
	//todo:
	//RLE compression decoding?

	FILE* f = fopen(fn, "rb");
	if(!f)
		return false;

	BITMAPFILEHEADER bfh;
	if(fread(&bfh, sizeof(bfh), 1, f) != 1)
	{
		FatalError("Couldn't load file header");
		fclose(f);
		return false;
	}

	if(memcmp(&bfh.bfType, "BM", 2) != 0) 
	{
		FatalError("This isn't a bitmap file");
		return false;
	}

	BITMAPINFOHEADER bi;
	if(fread(&bi, sizeof(bi), 1, f) != 1)
	{
		FatalError("Couldn't load info header");
		fclose(f);
		return false;
	}

	image.w = bi.biWidth;
	image.h = bi.biHeight;
	image.num_bytes = bi.biSizeImage;


	switch(bi.biCompression)
	{
	case BI_RGB:
		if(image.num_bytes == 0)
			image.num_bytes = image.w * image.h * (bi.biBitCount>>3);
		break;
	default:
		FatalError("Unsupported compression type");
		fclose(f);
		return false;
	}

	int size_to_read = image.num_bytes;

	RGBQUAD palette [256];
	int palette_size = 256;
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
			FatalError("Couldnt read palette");
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
		FatalError("Unsupported bit count");
		fclose(f);
		return false;
	}
	
	fseek(f, bfh.bfOffBits, SEEK_SET);

	unsigned char* bits = new unsigned char [size_to_read];
	if(fread(bits, 1, size_to_read, f) != size_to_read)
	{
		FatalError("Couldnt read bits\n");
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

	assert(image.num_bytes == image.w*image.h*image.pixelSize());
	image.initialize(image.w,image.h,image.format);
	if(decode_8)
	{
		unsigned char* bit = bits;
		unsigned char* pixel = image.data;
		for(int i=0; i<size_to_read; i++)
		{
			if(*bit >= palette_size)
			{
				FatalError("out of palette range\n");
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
#endif //WIN32

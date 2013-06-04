#include "import.h"
#include <errors.h>
#include <utils/stringutils.h>

const char* ImageImportTypes()
{
	return "*.bit;*.bmp;*.tga;";
}

bool ImportImage(const char* fn, Image& img)
{
	const char* ext = FileExtension(fn);
	if(!ext) {
		FatalError("Couldnt detect an extension on image import file %s", fn);
		return false;
	}
	if(strlen(ext) > 8) {
		FatalError("Unknown extension \"%s\" on image import file %s", ext, fn);
		return false;
	}
	char extbuf[8];
	strcpy(extbuf, ext);
	Lowercase(extbuf);
	if(0 == strcmp(extbuf, "bit")) {
		return img.Read(fn);
	}
	else if(0 == strcmp(extbuf, "bmp")) {
		return ImportImageBMP(fn, img);
	}
	else if(0 == strcmp(extbuf, "tga")) {
		return ImportImageTGA(fn, img);
	}
	else {
		return ImportImageGDIPlus(fn, img);
	}
	/*
	{
		ReportError("Unknown extension \"%s\" on image import file %s", ext, fn);
		return false;
	}*/
}


#if 0
/******************************* BMP Import ************************************/

bool ImportImageBMP(const char* fn, Image& img)
{
	//todo:
	//RLE compression decoding?

	FILE* f = fopen(fn, "rb");
	if(!f)
		return false;

	BITMAPFILEHEADER bfh;
	if(fread(&bfh, sizeof(bfh), 1, f) != 1)
	{
		printf("Couldn't load file header\n");
		fclose(f);
		return false;
	}

	if(memcmp(&bfh.bfType, "BM", 2) != 0) 
	{
		printf("This isn't a bitmap file\n");
		return false;
	}

	BITMAPINFOHEADER bi;
	if(fread(&bi, sizeof(bi), 1, f) != 1)
	{
		printf("Couldn't load info header\n");
		fclose(f);
		return false;
	}

	int unpackedsize = bi.biSize;

	switch(bi.biCompression)
	{
	case BI_RGB:
		if(unpackedsize == 0)
			unpackedsize = bi.biWidth * bi.biHeight * (bi.biBitCount>>3);
		break;
	default:
		printf("Unsupported compression type\n");
		fclose(f);
		return false;
	}

	int size_to_read = unpackedsize;

	RGBQUAD palette [256];
	int palette_size = 256;
	bool decode_8 = false;

	Image::PixelFormat fmt;
	switch(bi.biBitCount)
	{
	case 8:
		{
		if(bi.biClrUsed != 0)
			palette_size = bi.biClrUsed;
		printf("8 bit\n");
		decode_8 = true;
		if(fread(&palette, sizeof(RGBQUAD), palette_size, f) != palette_size)
		{
			printf("Couldnt read palette\n");
			fclose(f);
			return false;
		}
		fmt = Image::R8G8B8;
		}
		break;
	case 16:
		printf("16 bit\n");
		fmt = Image::R5G6B5;
		break;
	case 24:
		printf("24 bit\n");
		fmt = Image::R8G8B8;
		break;
	default:
		printf("Unsupported bit count\n");
		fclose(f);
		return false;
	}

	printf("currently %d\n", ftell(f));
	
	fseek(f, bfh.bfOffBits, SEEK_SET);

	unsigned char* bits = new unsigned char [size_to_read];
	if(fread(bits, 1, size_to_read, f) != size_to_read)
	{
		printf("Couldnt read bits\n");
		fclose(f);
		delete [] bits;
		return false;
	}

	if(ftell(f) != bfh.bfSize)
	{
		int cur = ftell(f);
		fseek(f, 0, SEEK_END);
		printf("Um, there was some stuff missed: offset is %d, size is %d, to read is %d, current is %d, end is %d\n",
			bfh.bfOffBits, bfh.bfSize, size_to_read, cur, ftell(f));
	}

	img.initialize(bi.biWidth, bi.biHeight, fmt);
	assert(image.pixelBPP() == bi.biBitCount);
	assert(image.num_bytes == image.w*image.h*image.pixelSize());
	if(decode_8)
	{
		unsigned char* bit = bits;
		unsigned char* pixel = img.data;
		for(int i=0; i<size_to_read; i++)
		{
			if(*bit >= palette_size)
			{
				printf("out of palette range\n");
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
		memcpy(img.data, bits, img.num_bytes);
	}


	delete [] bits;
	fclose(f);
	return true;
}


/******************************* TGA Import ************************************/



#define IT_COLOR_MAPPED		1
#define IT_RGB				2
#define IT_BW				3
#define IT_RLE_COLOR_MAPPED 9
#define IT_RLE_RGB			10

typedef unsigned char BYTE;
typedef unsigned short HALFWORD;

#pragma pack()

struct tga_file_header
{
	BYTE size_image_identification;
	BYTE color_map_type;
	BYTE image_type;

	//color map specification
	HALFWORD color_map_origin;
	HALFWORD color_map_size;
	BYTE color_map_bpp;

	//image specification
	HALFWORD x_origin;
	HALFWORD y_origin;
	HALFWORD width;
	HALFWORD height;
	BYTE bpp;
	BYTE image_descriptor;
};

void print_header(tga_file_header& h)
{
	printf("size ident: %d\n", h.size_image_identification);
	printf("color map type: %d\n", h.color_map_type);
	printf("image type: %d\n\n", h.image_type);

	printf("color map origin: %d\n", h.color_map_origin);
	printf("color map size: %d\n", h.color_map_size);
	printf("color map bpp: %d\n\n", h.color_map_bpp);

	printf("x origin: %d\n", h.x_origin);
	printf("y origin: %d\n", h.y_origin);
	printf("width: %d\n", h.width);
	printf("height: %d\n", h.height);
	printf("bpp: %d\n", h.bpp);
	printf("color map bpp: %d\n\n", h.color_map_bpp);
}

#include <memory.h>
void copy_header_data(BYTE hdr[], tga_file_header& header)
{
	header.size_image_identification = hdr[0];
	header.color_map_type = hdr[1];
	header.image_type = hdr[2];

	//color map specification
	memcpy(&header.color_map_origin,  &hdr[3], 2);
	memcpy(&header.color_map_size,  &hdr[5], 2);
	header.color_map_bpp = hdr[7];

	//image specification
	memcpy(&header.x_origin, &hdr[8], 2);
	memcpy(&header.y_origin, &hdr[10], 2);
	memcpy(&header.width, &hdr[12], 2);
	memcpy(&header.height, &hdr[14], 2);
	header.bpp = hdr[16];
	header.image_descriptor = hdr[17];
}

bool ImportImageTGA(const char* fn, Image& img)
{
	FILE* f = fopen(fn, "rb");
	if(!f)
		return false;

	tga_file_header header;
	BYTE* color_map = NULL;
	int pixel_size;

	BYTE hdr[18];
	if(fread(&hdr, 1, 18, f) != 18)
		return false;

	copy_header_data(hdr, header);

//	if(fread(&header, sizeof(tga_file_header), 1, f) != 1)
//		return false;

	printf("size of header %d\n", sizeof(tga_file_header));

	print_header(header);

	Image::PixelFormat fmt;

	pixel_size = header.bpp >> 3;

	if(header.image_type == IT_RGB)
	{
		switch(header.bpp)
		{
		case 16:
			fmt = Image::X1R5G5B5;
			break;
		case 24:
			fmt = Image::R8G8B8;
			break;
		case 32:
			fmt = Image::A8R8G8B8;
			break;
		default:
			ReportError("dont know how to read %d bit targas\n", header.bpp);
			return false;
		}
	}
	else if(header.image_type == IT_COLOR_MAPPED)
	{
		switch(header.color_map_bpp)
		{
		case 16:
			fmt = Image::X1R5G5B5;
			break;
		case 24:
			fmt = Image::R8G8B8;
			break;
		case 32:
			fmt = Image::A8R8G8B8;
			break;
		default:
			ReportError("dont know how to read %d bit targas\n", header.bpp);
			return false;
		}
	}
	else
	{
		ReportError("dont know how to read that format targa\n");
		return false;
	}

//	if(header.image_descriptor != 0)
//		ReportError("odd image descriptor, %x", header.image_descriptor);

	//skip over image identification
	if(header.size_image_identification!= 0)
		fseek(f, header.size_image_identification, SEEK_CUR);

	//color map
	if(header.color_map_type != 0)
	{
		int size_of_color_map = header.color_map_size * header.color_map_bpp>>3;
		color_map = (BYTE*)malloc(size_of_color_map);

		if(fread(color_map, 1, size_of_color_map, f) != size_of_color_map)
		{
			free(color_map);
			return false;
		}
	}

	img.initialize(header.width, header.height, fmt);

	//image data
	if(header.image_type == IT_RGB)
	{
		if(fread(img.data, 1, img.num_bytes, f) != img.num_bytes)
		{
			ReportError("error reading image, %d bytes\n", img.num_bytes);
			return false;
		}
	}
	else if(header.image_type == IT_COLOR_MAPPED)
	{
		int index;
		BYTE* dest = img.data;
		int color_map_bytes = header.color_map_bpp>>3;
		for(int i=0; i<header.width * header.height; i++)
		{
			if(fread(&index, pixel_size, 1, f) != 1)
				return false;

			for(int j=0; j<color_map_bytes; j++);
			{
				dest[color_map_bytes-j-1] = color_map[index*color_map_bytes + j];
			}
			*dest += color_map_bytes;
		}
	}

	if(color_map)
		free(color_map);
	fclose(f);
	return true;
}
#endif
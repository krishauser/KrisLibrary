#include "image.h"
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
using namespace std;

#define IT_COLOR_MAPPED		1
#define IT_RGB				2
#define IT_BW				3
#define IT_RLE_COLOR_MAPPED 9
#define IT_RLE_RGB			10

typedef unsigned char BYTE;
typedef unsigned short HALFWORD;

// HACK: GCC Darwin (Mac) doesn't like pragma pack for some reason
// inexplicable "too many #pragma options align=reset" error
// (mw54, Jun 2010)
#if !defined (__APPLE__) && !defined (MACOSX)
#pragma pack()
#endif

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

bool ImportImageTGA(const char* fn, Image& image)
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

	image.w = header.width;
	image.h = header.height;

	pixel_size = header.bpp >> 3;

	if(header.image_type == IT_RGB)
	{
		switch(header.bpp)
		{
		case 16:
			image.format = Image::X1R5G5B5;
			break;
		case 24:
			image.format = Image::R8G8B8;
			break;
		case 32:
			image.format = Image::A8R8G8B8;
			break;
		default:
		  cerr<<"Dont know how to read "<<header.bpp<<" bit targas"<<endl;
		  return false;
		}
	}
	else if(header.image_type == IT_COLOR_MAPPED)
	{
		switch(header.color_map_bpp)
		{
		case 16:
			image.format = Image::X1R5G5B5;
			break;
		case 24:
			image.format = Image::R8G8B8;
			break;
		case 32:
			image.format = Image::A8R8G8B8;
			break;
		default:
		  cerr<<"Dont know how to read "<<header.color_map_bpp<<" bit color-mapped targas"<<endl;
		  return false;
		}
	}
	else
	{
	  cerr<<"Dont know how to read that format targa"<<endl;
	  return false;
	}

	
	if(image.pixelBPP() != header.bpp)
	{
	  cerr<<"Invalid bpp in targa file"<<endl;
	  return false;
	}

//	if(header.image_descriptor != 0)
//		cerr<<"odd image descriptor, "<<hex<<header.image_descriptor<<endl;

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

	image.initialize(image.w, image.h, image.format);

	//image data
	if(header.image_type == IT_RGB)
	{
		if(fread(image.data, 1, image.num_bytes, f) != image.num_bytes)
		{
		  cerr<<"Error reading image, "<<image.num_bytes<<" bytes"<<endl;
		  return false;
		}
	}
	else if(header.image_type == IT_COLOR_MAPPED)
	{
		int index;
		BYTE* dest = image.data;
		int color_map_bytes = header.color_map_bpp>>3;
		for(int i=0; i<header.width * header.height; i++)
		{
			if(fread(&index, pixel_size, 1, f) != 1)
				return false;

			for(int j=0; j<color_map_bytes; j++)
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


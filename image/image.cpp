#include <KrisLibrary/Logger.h>
#include <KrisLibrary/File.h>
#include <KrisLibrary/utils.h>
#include <KrisLibrary/errors.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils.h>
#include <memory.h>
#include <math.h>
#include <iostream>
using namespace std;

#include "image.h"
#include <math.h>


static int IMAGEHEADER = 0xd0d0beef;

#include "formats.h"

PIXELGETPROC pixel_get_proc(Image::PixelFormat format)
{
	switch(format)
	{
	case Image::A8R8G8B8:
		return argb_get;
	case Image::R8G8B8:
		return rgb8_get;
	case Image::R5G6B5:
		return r5g6b5_get;
	case Image::X1R5G5B5:
		return x1r5g5b5_get;
	case Image::A8:
		return a8_get;
	case Image::FloatRGB:
		return frgb_get;
	case Image::FloatRGBA:
		return frgba_get;
	case Image::FloatA:
		return fa_get;
	default:
		FatalError("Unknown Image pixel format");
	}
	return NULL;
}

PIXELSETPROC pixel_set_proc(Image::PixelFormat format)
{
	switch(format)
	{
	case Image::A8R8G8B8:
		return argb_set;
	case Image::R8G8B8:
		return rgb8_set;
	case Image::R5G6B5:
		return r5g6b5_set;
	case Image::X1R5G5B5:
		return x1r5g5b5_set;
	case Image::A8:
		return a8_set;
	case Image::FloatRGB:
		return frgb_set;
	case Image::FloatRGBA:
		return frgba_set;
	case Image::FloatA:
		return fa_set;
	default:
		FatalError("Unknown Image pixel format");
	}
	return NULL;
}


static unsigned int pfsizes [] = 
{
	0,
	3,
	4,
	2,
	2,
	1,
	12,
	16,
	4
};

bool Image::isValidFormat(PixelFormat f)
{
	return (f < ArraySize(pfsizes) && f >= 0);
}

unsigned int Image::pixelFormatSize(PixelFormat f)
{
	if(f >= ArraySize(pfsizes)) return false;
	return pfsizes[f];
}




Image::Image()
:format(None), w(0),h(0), data(NULL), num_bytes(0) 
{}

Image::Image(const Image& other)
:format(None), w(0),h(0), data(NULL), num_bytes(0) 
{
	operator = (other);
}

Image::~Image()
{
	unload();
}

int Image::initialize(int _w, int _h, PixelFormat _fmt)
{
	if(_w < 0 || _h < 0 || !isValidFormat(_fmt))
		return -1;
	unload();
	w = _w;
	h = _h;
	format = _fmt;
	num_bytes = w*h*pixelSize();
	if(num_bytes != 0) {
		data = (unsigned char*)malloc(num_bytes);
		if(!data)
		{
		  LOG4CXX_ERROR(KrisLibrary::logger(),"Error allocating "<<num_bytes<<" bytes");
		  return -1;
		}
	}
	else
		data = NULL;
	return num_bytes;
}

void Image::unload()
{
	if(data)
	{
		free(data);
		data = NULL;
	}
	w = h = 0;
	format = None;
	num_bytes = 0;
}

const Image& Image::operator = (const Image& other)
{
	unload();
	initialize(other.w, other.h, other.format);
	assert(num_bytes >= other.num_bytes);
	memcpy(data, other.data, other.num_bytes);
	return *this;
}

bool Image::Read(const char* str)
{
	File f;
	if(!f.Open(str, FILEREAD)) return false;
	return Read(f);
}


bool Image::Read(File& f)
{
	unload();

	int hdr,tmp;
	if(!ReadFile(f, hdr))
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Could not read header");
	  return false;
	}
	if(hdr != IMAGEHEADER)
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid header "<<hdr);
	  return false;
	}

	if(!ReadFile(f, w)) return false;
	if(!ReadFile(f, h)) return false;
	if(!ReadFile(f, tmp)) return false;
	format = (PixelFormat)tmp;
	if(initialize(w,h,format) < 0)
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error initializing image");
	  return false;
	}

	if(!f.ReadData(data, num_bytes))
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Could not read texture");
	  unload();
	  return false;
	}

	return true;
}

void Image::clear(int dat)
{
	memset(data,dat,num_bytes);
}

void Image::blit(Image& dest, int sx, int sy, int bw, int bh, int dx, int dy) const
{
	assert(data != NULL);
	assert(dest.data != NULL);

	//crop args
	if(sx < 0)
	{
		dx = dx - sx;
		bw = bw + sx;
		sx = 0;
	}
	if(sy < 0)
	{
		dy = dy - sy;
		bh = bh + sy;
		sy = 0;
	}
	if(dx < 0)
	{
		sx = sx - dx;
		bw = bw + dx;
		dx = 0;
	}
	if(dy < 0)
	{
		sy = sy - dy;
		bh = bh + dy;
		dy = 0;
	}

	if (bw < 0)		//default width
		bw = w;
	if (bh < 0)		//default height
		bh = h;

	if(sx + bw > w)
	{
		bw = w - sx;
	}
	if(sy + bh > h)
	{
		bh = h - sy;
	}
	if(dx + bw > dest.w)
	{
		bw = dest.w - dx;
	}
	if(dy + bh > dest.h)
	{
		bh = dest.h - dy;
	}

	assert(sx >= 0);
	assert(sy >= 0);
	assert(dx >= 0);
	assert(dy >= 0);
	assert(bw >= 0);
	assert(bh >= 0);
	assert(sx+bw <= w);
	assert(sy+bh <= h);
	assert(dx+bw <= dest.w);
	assert(dy+bh <= dest.h);

	PIXELGETPROC get;
	PIXELSETPROC set;
	get = pixel_get_proc(format);
	set = pixel_set_proc(dest.format);

	//now do the blit
	unsigned char* src = getData(sx, sy);
	unsigned char* dst = dest.getData(dx, dy);
	int ss = pixelSize(), ds = dest.pixelSize();
	int sp = pitch(), dp = dest.pitch();
	COLOROPTYPE col;
	for(int j=0; j<bh; j++, src+=sp, dst+=dp)
	{
		unsigned char* sl = src;
		unsigned char* dl = dst;
		for(int i=0; i<bw; i++, sl+=ss, dl+=ds)
		{
			assert(sl >= data);
			assert(sl < (data + num_bytes));
			assert(dl >= dest.data);
			assert(dl < (dest.data + dest.num_bytes));
			get(sl, col);
			set(dl, col);
		}
	}
}


bool Image::Write(const char* str) const
{
	File f;
	if(!f.Open(str, FILEWRITE)) return false;
	return Write(f);
}

bool Image::Write(File& f) const
{
	int hdr = IMAGEHEADER, tmp;
	if(!WriteFile(f, hdr)) return false;
	if(!WriteFile(f, w)) return false;
	if(!WriteFile(f, h)) return false;
	tmp = format;
	if(!WriteFile(f, tmp)) return false;
	if(!f.WriteData(data, num_bytes)) return false;
	return true;
}

unsigned int Image::pitch() const
{
	return pixelSize() * w;
}

unsigned char* Image::getData(int x, int y) const
{
	if(x >= w || x < 0)
		return NULL;
	if(y >= h || y < 0)
		return NULL;
	int size = pixelSize();
	return data + ((x+y*w)*size);
}



ImageMipmapped::ImageMipmapped()
:Image(), mipmap_data(NULL), num_mipmap_levels(0)
{}

ImageMipmapped::ImageMipmapped(const Image& i)
:Image(i), mipmap_data(NULL), num_mipmap_levels(0)
{
	createMipmaps();
}

ImageMipmapped::~ImageMipmapped()
{
	//destructor for Image will call unload
}

const ImageMipmapped& ImageMipmapped::operator = (const Image& img)
{
	Image::operator =(img);
	createMipmaps();
	return *this;
}

const ImageMipmapped& ImageMipmapped::operator = (const ImageMipmapped& img)
{
	Image::operator =(img);
	createMipmaps();
	return *this;
}

void ImageMipmapped::unload()
{
	if(mipmap_data)
	{
		for(unsigned int i=0; i<num_mipmap_levels; i++)
			delete [] mipmap_data[i];
		delete [] mipmap_data;
		mipmap_data = NULL;
		num_mipmap_levels = 0;
	}
	Image::unload();
}

bool ImageMipmapped::Read(File& f)
{
	if(!Image::Read(f)) return false;
	createMipmaps();
	return true;
}



int countMipLevels(unsigned int w, unsigned int h)
{
	unsigned int minsize = Min(w, h);
	unsigned int mask=0xffffffff;
	int i;
	for(i=0; i<32; i++) {
		mask=mask<<1;
		if(!(minsize&mask)) return i;
	}
	LOG4CXX_ERROR(KrisLibrary::logger(),"Cant have more than 32 mip levels... something's fishy");
	abort();
	return i;
}


//dest dims are w/2, h/2
unsigned char* shrink_texture_boxfilter(const unsigned char* src, int w, int h, Image::PixelFormat format)
{
	int stride, pixelsize;
	pixelsize = Image::pixelFormatSize(format);
	stride = w * pixelsize;

	PIXELGETPROC get = pixel_get_proc(format);
	PIXELSETPROC set = pixel_set_proc(format);

	int w2 = w>>1, h2 = h>>1;
	assert(w2 > 0);
	assert(h2 > 0);
	int size=w2*h2*pixelsize;
	unsigned char* shrunken = new unsigned char[size];
	if(!shrunken) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Not enough memory to allocate shrunken bitmap?");
	  abort();
	}
	unsigned char* dest = shrunken;

	int h_ofs = 0;
	for(int p=0; p<h2; p++)
	{
		int w_ofs = 0;
		for(int q=0; q<w2; q++)
		{
			const unsigned char* a = &src[h_ofs + w_ofs];

			COLOROPTYPE col;
			col[0] = col[1] = col[2] = col[3] = 0;
			COLOROPTYPE tcol;
			get(a, tcol);
			add_color(col,tcol);
			get(a+pixelsize, tcol);
			add_color(col,tcol);
			get(a+stride, tcol);
			add_color(col,tcol);
			get(a+pixelsize+stride, tcol);
			add_color(col,tcol);
			shift_right_color(col,2);		//div by four
			set(dest, col);

			dest += pixelsize;
			w_ofs += pixelsize+pixelsize;  //move 2 pixels of source
		}
		h_ofs += (stride*2);				//move 2 lines of source
	}
	return shrunken;
}


void ImageMipmapped::createMipmaps()
{
	if(mipmap_data)
	{
		for(unsigned int i=0; i<num_mipmap_levels; i++)
			delete [] mipmap_data[i];
		delete [] mipmap_data;
	}

	//set up the mipmap chain
	num_mipmap_levels = countMipLevels(w,h);	//count possible mip levels (1st is the root data)
	if(num_mipmap_levels <= 0)	//possible if size is 1 by 1
	{
		mipmap_data = NULL;
		return;
	}

	mipmap_data = new unsigned char* [num_mipmap_levels];
	if(!mipmap_data)
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Not enough memory to create mipmap bits?");
	  mipmap_data = 0;
	  return;
	}

	int size = (w>>1)*(h>>1)*pixelSize();
	mipmap_data [0]=shrink_texture_boxfilter(data, w, h, format);
	for(unsigned int i=1; i<num_mipmap_levels; i++)
	{
		mipmap_data[i] = shrink_texture_boxfilter(mipmap_data[i-1], w>>i, h>>i, format);
	}
}



void Pixel::mix(const Pixel& p, const Pixel& q, float u)
{
	float temp;
	temp = (1.0f-u)*p.r + u*q.r;
	r = (color_channel_t)temp;
	temp = (1.0f-u)*p.g + u*q.g;
	g = (color_channel_t)temp;
	temp = (1.0f-u)*p.b + u*q.b;
	b = (color_channel_t)temp;
	temp = (1.0f-u)*p.a + u*q.a;
	a = (color_channel_t)temp;
}

inline int clamp(int x, int a, int b)
{
	return (x < a ? a : (x > b ? b : x));
}

ImageOperator::ImageOperator()
:pixels(NULL),w(0),h(0),num_pixels(0)
{}

ImageOperator::ImageOperator(const Image& i)
:pixels(NULL),w(0),h(0),num_pixels(0)
{
	operator = (i);
}

ImageOperator::~ImageOperator()
{
	unload();
}

void ImageOperator::initialize(int _w, int _h)
{
	unload();
	w = _w;
	h = _h;
	num_pixels = w*h;
	pixels = new Pixel[num_pixels];
}

void ImageOperator::resize(int _w, int _h)
{
	if(w != _w || h != _h)
	{
		initialize(_w,_h);
	}
}

void ImageOperator::unload()
{
	SafeArrayDelete(pixels);
	w = h = num_pixels = 0;
}

const ImageOperator& ImageOperator::operator = (const Image& img)
{
	resize(img.w,img.h);
	int psize = img.pixelSize();
	PIXELGETPROC get;
	COLOROPTYPE op;
	get = pixel_get_proc(img.format);
	assert(get);
	unsigned char* dat = img.data;
	for(unsigned int i=0; i<num_pixels; i++, dat+=psize)
	{
		get(dat, op);
		pixels[i].r = op[0];
		pixels[i].g = op[1];
		pixels[i].b = op[2];
		pixels[i].a = op[3];
	}
  return *this;
}

void ImageOperator::output (Image& img, Image::PixelFormat fmt) const
{
	img.initialize(w,h,fmt);
	int psize = img.pixelSize();
	PIXELSETPROC set;
	COLOROPTYPE op;
	set = pixel_set_proc(img.format);
	assert(set);
	unsigned char* dat = img.data;
	for(unsigned int i=0; i<num_pixels; i++, dat+=psize)
	{
		op[0] = pixels[i].r;
		op[1] = pixels[i].g;
		op[2] = pixels[i].b;
		op[3] = pixels[i].a;
		set(dat, op);
	}
}

const Pixel& ImageOperator::getPixel(int x, int y) const
{
	x = clamp(x, 0, w);
	y = clamp(y, 0, h);
	return pixels[x+y*w];
}


Pixel& ImageOperator::getPixel(int x, int y)
{
	clamp(x, 0, w);
	clamp(y, 0, h);
	return pixels[x+y*w];
}

void ImageOperator::sampleNearest(float x, float y, Pixel& out) const
{
	int bx = (int)floorf(x);
	int by = (int)floorf(y);
	out = getPixel(bx,by);
}


void ImageOperator::sampleLinear(float x, float y, Pixel& out) const
{
	float fx = floorf(x);
	float fy = floorf(y);
	int bx = int(fx);
	int by = int(fy);
	float ux = x - fx;
	float uy = y - fy;

	Pixel temp;
	temp.mix(getPixel(bx+1,by), getPixel(bx,by), ux);
	out.mix(getPixel(bx+1,by+1), getPixel(bx,by+1), ux);
	out.mix(out,temp,uy);
}



void ImageOperator::stretchBlit(ImageOperator& dest) const
{
	float x_rate = (float)w / dest.w;
	float y_rate = (float)h / dest.h;
	int is,js;
	float fis,fjs;
	//float a,b;
	fis = 0;
	Pixel* s=pixels;
	Pixel* d=dest.pixels;
	for(int i=0; i<dest.h; i++, fis += y_rate)
	{
		is = (int)fis;
		int src_row = w * is;
		fjs = 0;
		for(int j=0; j<dest.w; j++, fjs += x_rate)			//copy horzontal line
		{
			js = (int)fjs;
			s = &pixels[js + src_row];
			*d = *s;
			d++;
		}
	}
}

void ImageOperator::stretchBlitBilinear(ImageOperator& dest) const
{
	float x_rate = (float)w / dest.w;
	float y_rate = (float)h / dest.h;
	Pixel pixel;
	float fis,fjs;
	//float a,b;
	fis = 0;
	Pixel* s=pixels;
	Pixel* d=dest.pixels;
	for(int i=0; i<dest.h; i++, fis += y_rate)
	{
		fjs = 0;
		for(int j=0; j<dest.w; j++, fjs += x_rate)			//copy horzontal line
		{
			sampleLinear(fjs,fis,pixel);
			*d = pixel;
			d++;
		}
	}
}



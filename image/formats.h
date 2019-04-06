
typedef unsigned int COLOROPTYPE [4];
typedef void (*PIXELGETPROC) (const unsigned char* bits, COLOROPTYPE col);
typedef void (*PIXELSETPROC) (unsigned char* bits, const COLOROPTYPE col);

#include <memory.h>

struct r5g6b5
{
	unsigned int r :5;
	unsigned int g :6;
	unsigned int b :5;
};

struct x1r5g5b5
{
	unsigned int x :1;
	unsigned int r :5;
	unsigned int g :5;
	unsigned int b :5;
};


void argb_get (const unsigned char* bits, COLOROPTYPE col)
{
	col[0] = *bits;
	col[1] = *(bits+1);
	col[2] = *(bits+2);
	col[3] = *(bits+3);
}

void argb_set (unsigned char* bits, const COLOROPTYPE col)
{
	*bits     = col[0];
	*(bits+1) = col[1];
	*(bits+2) = col[2];
	*(bits+3) = col[3];
}

void rgb8_get (const unsigned char* bits, COLOROPTYPE col)
{
	col[0] = *bits;
	col[1] = *(bits+1);
	col[2] = *(bits+2);
}

void rgb8_set (unsigned char* bits, const COLOROPTYPE col)
{
	*bits     = col[0];
	*(bits+1) = col[1];
	*(bits+2) = col[2];
}

void r5g6b5_get (const unsigned char* bits, COLOROPTYPE col)
{
	r5g6b5 b;
	memcpy(&b, bits, sizeof(short));
	col[0] = b.r;
	col[1] = b.g;
	col[2] = b.b;
}

void r5g6b5_set (unsigned char* bits, const COLOROPTYPE col)
{
	r5g6b5 c;
	c.r = col[0];
	c.g = col[1];
	c.b = col[2];
	memcpy(bits, &c, sizeof(short));
}

void x1r5g5b5_get (const unsigned char* bits, COLOROPTYPE col)
{
	x1r5g5b5 c;
	memcpy(&c, bits, sizeof(short));
	col[0] = c.r;
	col[1] = c.g;
	col[2] = c.b;
}

void x1r5g5b5_set (unsigned char* bits, const COLOROPTYPE col)
{
	x1r5g5b5 c;
	c.r = col[0];
	c.g = col[1];
	c.b = col[2];
	memcpy(bits, &c, sizeof(short));
}

void a8_get (const unsigned char* bits, COLOROPTYPE col)
{
	col[0] = *bits;
	col[1] = col[2] = col[0];
}

void a8_set (unsigned char* bits, const COLOROPTYPE col)
{
	*bits     = col[0];
}


const static float one_over_255 = 1.0f/255.0f;

void frgb_get (const unsigned char* bits, COLOROPTYPE col)
{
	const float* fbits = (const float*)bits;
	col[0] = (unsigned int)(fbits[0]*255.0);
	col[1] = (unsigned int)(fbits[1]*255.0);
	col[2] = (unsigned int)(fbits[2]*255.0);
}

void frgb_set (unsigned char* bits, const COLOROPTYPE col)
{
	float* fbits = (float*)bits;
	fbits[0] = float(col[0])*one_over_255;
	fbits[1] = float(col[1])*one_over_255;
	fbits[2] = float(col[2])*one_over_255;
}

void frgba_get (const unsigned char* bits, COLOROPTYPE col)
{
	const float* fbits = (const float*)bits;
	col[0] = (unsigned int)(fbits[0]*255.0);
	col[1] = (unsigned int)(fbits[1]*255.0);
	col[2] = (unsigned int)(fbits[2]*255.0);
	col[3] = (unsigned int)(fbits[3]*255.0);
}

void frgba_set (unsigned char* bits, const COLOROPTYPE col)
{
	float* fbits = (float*)bits;
	fbits[0] = float(col[0])*one_over_255;
	fbits[1] = float(col[1])*one_over_255;
	fbits[2] = float(col[2])*one_over_255;
	fbits[3] = float(col[3])*one_over_255;
}


void fa_get (const unsigned char* bits, COLOROPTYPE col)
{
	const float* fbits = (const float*)bits;
	col[0] = (unsigned int)(fbits[0]*255.0);
	col[1] = col[2] = col[0];
}

void fa_set (unsigned char* bits, const COLOROPTYPE col)
{
	float* fbits = (float*)bits;
	fbits[0] = float(col[0])*one_over_255;
}


inline void zero_color(COLOROPTYPE x)
{
	x[0] = x[1] = x[2] = x[3] = 0;
}

inline void add_color(COLOROPTYPE x, const COLOROPTYPE a)
{
	x[0]+=a[0];
	x[1]+=a[1];
	x[2]+=a[2];
	x[3]+=a[3];
}

inline void shift_right_color(COLOROPTYPE x, unsigned int shift)
{
	x[0] = x[0] >> shift;
	x[1] = x[1] >> shift;
	x[2] = x[2] >> shift;
	x[3] = x[3] >> shift;
}

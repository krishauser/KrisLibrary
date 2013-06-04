#ifndef MATH_CAST_H
#define MATH_CAST_H

#include <math.h>

const static unsigned int OneFloat = 0x3f800000;
const static unsigned int FSignMask = 0x80000000;
const static unsigned int FExponentMask = 0x7f800000;
const static unsigned int FMantissaMask = 0x007fffff;

#ifdef WIN32
inline long int iRound(float f) { return long int(f); }
inline long int iRound(double f) { return long int(f); }
#else
inline long int iRound(float f) { return lrintf(f); }
inline long int iRound(double f) { return lrint(f); }
#endif

inline long int iFloor(float f) { return iRound(floorf(f)); }
inline long int iFloor(double f) { return iRound(floor(f)); }

inline long int iCeil(float f) { return  iRound(ceilf(f)); }
inline long int iCeil(double f) { return iRound(ceil(f)); }

inline long int iTrunc(float f) { return (long int)f; }
inline long int iTrunc(double f) { return (long int)f; }


//converts a fixed-point unsigned char to a float (unsigned char in the range [0,1])
inline float FixedByteToFloat(unsigned char x)
{
	static const float scale = 1.0f/255.0f;
	return float(x)*scale;
}

//converts a float in the range [0,1] to a fixed-point unsigned char
inline unsigned char FloatToFixedByte(float x)
{
	if(x > 255.0f)
		return 0xff;
	if(x < 0.0f)
		return 0;

	int i = int(x*255.0f);
	return i&0xff;
}

//converts a fixed-point unsigned char to a float (unsigned char in the range [0,1) )
inline float FixedByteToFloat_Exclusive(unsigned char x)
{
	unsigned int tmp = OneFloat | (x << 15);
	return (*(float*)&tmp) - 1.0f;
}

//converts a fixed-point int to a float (int in the range [0,1) )
inline float FixedIntToFloat_Exclusive(unsigned long i)
{
	unsigned int tmp = OneFloat | (i >> 9);
	return (*(float*)&tmp) - 1.0f;
}

#endif

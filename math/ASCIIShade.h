#ifndef MATH_ASCII_SHADE_H
#define MATH_ASCII_SHADE_H

#include "vector.h"
#include "matrix.h"

namespace Math {

char ASCIIShade(double x);
void OutputASCIIShade(std::ostream& out,double x);
void OutputASCIIShade(std::ostream& out,const fVector& x,float scale=0);
void OutputASCIIShade(std::ostream& out,const fMatrix& A,float scale=0,int indent=0);
void OutputASCIIShade(std::ostream& out,const dVector& x,double scale=0);
void OutputASCIIShade(std::ostream& out,const dMatrix& A,double scale=0,int indent=0);

} //namespace Math

#endif

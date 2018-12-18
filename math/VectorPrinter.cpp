#include <KrisLibrary/Logger.h>
#include "VectorPrinter.h"
#include "complex.h"
#include <utils/stringutils.h>
#include <iostream>
#include "ASCIIShade.h"
using namespace std;

namespace Math {

VectorPrinter::VectorPrinter(const fVector& v,Mode _mode)
  :fv(&v),dv(NULL),cv(NULL),delim(' '),bracket('['),mode(_mode)
{}

VectorPrinter::VectorPrinter(const dVector& v,Mode _mode)
  :fv(NULL),dv(&v),cv(NULL),delim(' '),bracket('['),mode(_mode)
{}

VectorPrinter::VectorPrinter(const cVector& v,Mode _mode)
  :fv(NULL),dv(NULL),cv(&v),delim(' '),bracket('['),mode(_mode)
{}

template<class T>
void PrintVector(const VectorTemplate<T>& x,ostream& out,char delim,char bracket)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  VectorIterator<T> v=x.begin();
  for(int i=0; i<x.n; i++,v++)
    out<<*v<<delim;
  if(bracket) out<<closebracket;
}

template <class T>
void OutputPlusMinus(ostream& out,const VectorTemplate<T>& x,T zeroTolerance=Epsilon)
{
  for(int i=0;i<x.n;i++) {
    if(x(i) < -zeroTolerance) out<<'-';
    else if(x(i) > zeroTolerance) out<<'+';
    else out<<'0';
  }
}

void VectorPrinter::Print(ostream& out) const
{
  switch(mode) {
  case Normal:
    if(fv) PrintVector(*fv,out,delim,bracket);
    else if(dv) PrintVector(*dv,out,delim,bracket);
    else if(cv) PrintVector(*cv,out,delim,bracket);
    break;
  case AsciiShade:
    if(fv) OutputASCIIShade(out,*fv);
    else if(dv) OutputASCIIShade(out,*dv);
    else if(cv) { LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to output an ASCII-shaded complex matrix"); }
    break;
  case PlusMinus:
    if(fv) OutputPlusMinus(out,*fv);
    else if(dv) OutputPlusMinus(out,*dv);
    else if(cv) { LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to output an +/- shaded complex matrix"); }
    break;
  }
}

ostream& operator << (ostream& out,const VectorPrinter& vp)
{
  vp.Print(out);
  return out;
}

} //namespace Math

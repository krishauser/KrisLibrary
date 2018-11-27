#include <KrisLibrary/Logger.h>
#include "MatrixPrinter.h"
#include "complex.h"
#include "ASCIIShade.h"
#include <utils/stringutils.h>
#include <iostream>
using namespace std;

namespace Math {

MatrixPrinter::MatrixPrinter(const fMatrix& m,Mode _mode)
  :fm(&m),dm(NULL),cm(NULL),delim(' '),bracket('['),mode(_mode)
{}

MatrixPrinter::MatrixPrinter(const dMatrix& m,Mode _mode)
  :fm(NULL),dm(&m),cm(NULL),delim(' '),bracket('['),mode(_mode)
{}

MatrixPrinter::MatrixPrinter(const cMatrix& m,Mode _mode)
  :fm(NULL),dm(NULL),cm(&m),delim(' '),bracket('['),mode(_mode)
{}

template<class T>
void PrintMatrix(const MatrixTemplate<T>& x,ostream& out,char delim,char bracket,int indent=0)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  MatrixIterator<T> v=x.begin();
  for(int i=0;i<x.m;i++,v.nextRow()) {
    if(indent) out<<string(indent,' ');
    if(bracket) out<<bracket;
    for(int j=0;j<x.n;j++,v.nextCol())
      out<<*v<<delim;
    if(bracket) out<<closebracket;
    if(i+1 != x.m) out<<endl;
  }
  if(bracket) out<<closebracket;
}

template<class T>
void OutputPlusMinus(ostream& out,const MatrixTemplate<T>& x,int indent,T zeroTolerance=Epsilon)
{
  MatrixIterator<T> v=x.begin();
  for(int i=0;i<x.m;i++,v.nextRow()) {
    if(indent) out<<string(indent,' ');
    for(int j=0;j<x.n;j++,v.nextCol()) {
      if(*v < -zeroTolerance) out<<'-';
      else if(*v > zeroTolerance) out<<'+';
      else out<<'0';
    }
    if(i+1 != x.m) out<<endl;
  }
}

void MatrixPrinter::Print(ostream& out,int indent) const
{
  switch(mode) {
  case Normal:
    if(fm) PrintMatrix(*fm,out,delim,bracket,indent);
    else if(dm) PrintMatrix(*dm,out,delim,bracket,indent);
    else if(cm) PrintMatrix(*cm,out,delim,bracket,indent);
    break;
  case AsciiShade:
    if(fm) OutputASCIIShade(out,*fm,0,indent);
    else if(dm) OutputASCIIShade(out,*dm,0,indent);
    else if(cm) { LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to output an ASCII-shaded complex matrix"); }
    break;
  case PlusMinus:
    if(fm) OutputPlusMinus(out,*fm,indent);
    else if(dm) OutputPlusMinus(out,*dm,indent);
    else if(cm) { LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to output an +/- shaded complex matrix"); }
    break;
  }
}

ostream& operator << (ostream& out,const MatrixPrinter& mp)
{
  mp.Print(out);
  return out;
}

} //namespace Math

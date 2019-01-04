#include "BlockPrinter.h"
#include "utils/stringutils.h"
#include <iostream>
#include <errors.h>
using namespace std;

namespace Math {

void PrintBlockVector(const BlockVector& v,ostream& out,char delim,char bracket)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  for(size_t i=0;i<v.size();i++) {
    for(int j=0;j<v[i].n;j++)
      out<<v[i](j)<<delim;
    if(i+1 != v.size())
      out<<'|';
  }
  if(closebracket) out<<closebracket;
}

void PrintBlockMatrix(const BlockTridiagonalMatrix& m,ostream& out,char delim,char bracket)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  for(size_t k=0;k<m.diagonal.size();k++) {
    for(int i=0;i<m.diagonal[k].m;i++) {
      //prepadding
      if(bracket) out<<bracket;
      for(size_t l=0;l+1<k;l++)
	out<<"  0  |";
      if(k != 0) {
	Assert(m.lowerDiagonal[k-1].m == m.diagonal[k].m);
	for(int j=0;j<m.lowerDiagonal[k-1].n;j++)
	  out<<m.lowerDiagonal[k-1](i,j)<<delim;
	out<<'|';
      }
      //diag
      for(int j=0;j<m.diagonal[k].n;j++)
	out<<m.diagonal[k](i,j)<<delim;
      if(k+1<m.diagonal.size()) {
	Assert(m.upperDiagonal[k].m == m.diagonal[k].m);
	out<<'|';
	for(int j=0;j<m.upperDiagonal[k].n;j++)
	  out<<m.upperDiagonal[k](i,j)<<delim;
      }
      //postpadding
      for(size_t l=k+2;l<m.diagonal.size();l++)
	out<<"|  0  ";
      if(closebracket) out<<closebracket;
      out<<endl;
    }
  }
}

BlockPrinter::BlockPrinter(const BlockVector& _v)
  :v(&_v),m(NULL),delim(' '),bracket('[')
{}

BlockPrinter::BlockPrinter(const BlockTridiagonalMatrix& _m)
  :v(NULL),m(&_m),delim(' '),bracket('[')
{}

void BlockPrinter::Print(ostream& out) const
{
  if(v) PrintBlockVector(*v,out,delim,bracket);
  else if(m) PrintBlockMatrix(*m,out,delim,bracket);
}

ostream& operator << (ostream& out,const BlockPrinter& bp)
{
  bp.Print(out);
  return out;
}

} //namespace Math

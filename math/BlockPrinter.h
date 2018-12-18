#ifndef MATH_BLOCK_PRINTER_H
#define MATH_BLOCK_PRINTER_H

#include "BlockVector.h"
#include "BlockTridiagonalMatrix.h"

namespace Math {

struct BlockPrinter
{
  BlockPrinter(const BlockVector& v);
  BlockPrinter(const BlockTridiagonalMatrix& m);
  void Print(std::ostream& out) const;

  const BlockVector* v;
  const BlockTridiagonalMatrix* m;
  char delim,bracket;
};

std::ostream& operator <<(std::ostream& out,const BlockPrinter& bp);

} //namespace Math

#endif

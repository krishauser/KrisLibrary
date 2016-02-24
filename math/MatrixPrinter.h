#ifndef MATH_MATRIX_PRINTER_H
#define MATH_MATRIX_PRINTER_H

#include "MatrixTemplate.h"

namespace Math {

struct MatrixPrinter
{
  enum Mode { Normal, AsciiShade, PlusMinus };

  MatrixPrinter(const fMatrix& m,Mode mode=Normal);
  MatrixPrinter(const dMatrix& m,Mode mode=Normal);
  MatrixPrinter(const cMatrix& m,Mode mode=Normal);
  void Print(std::ostream& out,int indent=0) const;

  const fMatrix* fm;
  const dMatrix* dm;
  const cMatrix* cm;
  char delim,bracket;
  Mode mode;
};

std::ostream& operator <<(std::ostream& out,const MatrixPrinter& mp);

} //namespace Math

#endif

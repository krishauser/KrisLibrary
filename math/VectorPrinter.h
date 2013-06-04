#ifndef MATH_VECTOR_PRINTER_H
#define MATH_VECTOR_PRINTER_H

#include "VectorTemplate.h"

namespace Math {

struct VectorPrinter
{
  enum Mode { Normal, AsciiShade, PlusMinus };

  VectorPrinter(const fVector& v,Mode mode=Normal);
  VectorPrinter(const dVector& v,Mode mode=Normal);
  VectorPrinter(const cVector& v,Mode mode=Normal);
  void Print(std::ostream& out) const;

  const fVector* fv;
  const dVector* dv;
  const cVector* cv;
  char delim,bracket;
  Mode mode;
};

std::ostream& operator <<(std::ostream& out,const VectorPrinter& vp);

} //namespace Math

#endif


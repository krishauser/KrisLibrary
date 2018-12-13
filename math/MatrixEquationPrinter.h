#ifndef MATH_MATRIX_EQUATION_PRINTER_H
#define MATH_MATRIX_EQUATION_PRINTER_H

#include "matrix.h"
#include <list>

namespace Math
{

struct EquationTerm
{
  EquationTerm();
  int Height() const;
  int Width() const;
  void PrintLine(int line,std::ostream& out) const;

  const Matrix* matrix;
  const Vector* vector;
  const char* text;
  Real scalar;
  bool transpose;
  bool ASCIIshade;
};

class MatrixEquationPrinter
{
public:
  void PushMatrix(const Matrix&);
  void PushMatrixTranspose(const Matrix&);
  void PushVector(const Vector&);
  void PushText(const char*);
  void PushScalar(Real);
  inline void PushAdd() { PushText("+"); }
  inline void PushSub() { PushText("-"); }
  inline void PushTimes() { PushText("*"); }
  inline void PushDot()  { PushText("."); }
  inline void PushEquals()  { PushText("="); }
  void Print(std::ostream& out) const;
  void Clear();

  std::list<EquationTerm> terms;
};


} //namespace Math

#endif

#include "IntPair.h"
#include <iostream>

std::ostream& operator << (std::ostream& out,const IntPair& t)
{
  out<<t.a<<" "<<t.b;
  return out;
}

std::istream& operator >> (std::istream& in,IntPair& t)
{
  in >> t.a >> t.b ;
  return in;
}

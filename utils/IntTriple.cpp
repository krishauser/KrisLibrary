#include "IntTriple.h"
#include <iostream>

std::ostream& operator << (std::ostream& out,const IntTriple& t)
{
  out<<t.a<<" "<<t.b<<" "<<t.c;
  return out;
}

std::istream& operator >> (std::istream& in,IntTriple& t)
{
  in >> t.a >> t.b >> t.c;
  return in;
}

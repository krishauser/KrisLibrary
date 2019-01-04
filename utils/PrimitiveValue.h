#ifndef PRIMITIVE_VALUE_H
#define PRIMITIVE_VALUE_H

#include <string>
#include <iosfwd>
using namespace std;

/** @brief A basic primitive value type, including integers, floats, and
 * strings.
 */
class PrimitiveValue
{
 public:
  enum { None, Integer, Double, String };

  PrimitiveValue();
  PrimitiveValue(int i);
  PrimitiveValue(double v);
  PrimitiveValue(const string& str);
  bool HasType(int _type) const { return type == _type; }
  bool CanCast(int _type) const;
  bool IsNumeric() const;
  operator const int& () const;
  operator int& ();
  operator const double& () const;
  operator double& ();
  operator const string& () const;
  operator string& ();
  int AsInteger() const;
  double AsDouble() const;
  string AsString() const;
  bool operator == (int v) const;
  bool operator == (double v) const;
  bool operator == (const string& v) const;
  bool operator == (const PrimitiveValue& v) const;
  bool operator < (int v) const;
  bool operator < (double v) const;
  bool operator < (const string& v) const;
  bool operator < (const PrimitiveValue& v) const;
  bool operator <= (int v) const;
  bool operator <= (double v) const;
  bool operator <= (const string& v) const;
  bool operator <= (const PrimitiveValue& v) const;

  int type;
  string sValue;
  double dValue;
  int iValue;
};

ostream& operator << (ostream& out,const PrimitiveValue& val);
istream& operator >> (istream& in,PrimitiveValue& val);

#endif

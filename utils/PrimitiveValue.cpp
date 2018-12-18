#include <KrisLibrary/Logger.h>
#include "PrimitiveValue.h"
#include <iostream>
#include <utils/ioutils.h>
#include <utils/stringutils.h>
#include <fstream>
#include <sstream>
#include <limits>
#include <errors.h>

const char* kNoneTypeString = "##NONE##";

PrimitiveValue::PrimitiveValue()
  :type(None)
{}

PrimitiveValue::PrimitiveValue(int val)
{
  type = Integer;
  iValue = val;
}

PrimitiveValue::PrimitiveValue(double val)
{
  type = Double;
  dValue = val;
}

PrimitiveValue::PrimitiveValue(const string& val)
{
  type = String;
  sValue = val;
}

PrimitiveValue::operator const int& () const
{
  Assert(type == Integer);
  return iValue;
}

PrimitiveValue::operator int& ()
{
  Assert(type == Integer);
  return iValue;
}

PrimitiveValue::operator const double& () const
{
  Assert(type == Double);
  return dValue;
}

PrimitiveValue::operator double& ()
{
  Assert(type == Double);
  return dValue;
}

PrimitiveValue::operator const string& () const
{
  Assert(type == String);
  return sValue;
}

PrimitiveValue::operator string& ()
{
  Assert(type == String);
  return sValue;
}

bool PrimitiveValue::IsNumeric() const
{
  return (type == Integer || type == Double);
}

bool PrimitiveValue::CanCast(int _type) const
{
  switch(_type) {
  case None: return true;
  case Integer: return type==Integer || type==Double;
  case Double:  return type==Integer || type==Double;
  case String: return type!=None;
  default: return false;
  }
}

int PrimitiveValue::AsInteger() const
{
  Assert(type == Integer || type == Double);
  if(type == Integer) return iValue;
  else return (int)dValue;
}

double PrimitiveValue::AsDouble() const
{
  Assert(type == Integer || type == Double);
  if(type == Integer) return (double)iValue;
  else return dValue;
}

string PrimitiveValue::AsString() const
{
  Assert(type != None);
  stringstream ss;
  switch(type) {
  case Integer:
    ss << iValue;
    break;
  case Double:
    ss << dValue;
    break;
  case String:
    ss << sValue;
    break;
  }
  return ss.str();
}

bool PrimitiveValue::operator == (int v) const
{
  if(type != Integer) return false;
  return iValue==v; 
}

bool PrimitiveValue::operator == (double v) const
{
  if(type != Double) return false;
  return dValue==v; 
}

bool PrimitiveValue::operator == (const string& v) const
{
  if(type != String) return false;
  return sValue==v; 
}

bool PrimitiveValue::operator == (const PrimitiveValue& v) const
{
  if(type != v.type) return false;
  switch(type) {
  case Integer: return iValue==v.iValue; 
  case Double: return dValue==v.dValue; 
  case String: return sValue==v.sValue; 
  default:  return false;
  }
}

bool PrimitiveValue::operator < (int v) const
{
  if(type != Integer) return false;
  return iValue<v; 
}

bool PrimitiveValue::operator < (double v) const
{
  if(type != Double) return false;
  return dValue<v; 
}

bool PrimitiveValue::operator < (const string& v) const
{
  if(type != String) return false;
  return sValue<v; 
}

bool PrimitiveValue::operator < (const PrimitiveValue& v) const
{
  if(type != v.type) return false;
  switch(type) {
  case Integer: return iValue<v.iValue; 
  case Double: return dValue<v.dValue; 
  case String: return sValue<v.sValue; 
  default:  return false;
  }
}

bool PrimitiveValue::operator <= (int v) const
{
  if(type != Integer) return false;
  return iValue<=v; 
}

bool PrimitiveValue::operator <= (double v) const
{
  if(type != Double) return false;
  return dValue<=v; 
}

bool PrimitiveValue::operator <= (const string& v) const
{
  if(type != String) return false;
  return sValue<=v; 
}

bool PrimitiveValue::operator <= (const PrimitiveValue& v) const
{
  if(type != v.type) return false;
  switch(type) {
  case Integer: return iValue<=v.iValue; 
  case Double: return dValue<=v.dValue; 
  case String: return sValue<=v.sValue; 
  default:  return false;
  }
}

ostream& operator << (ostream& out,const PrimitiveValue& v)
{
  switch(v.type) {
  case PrimitiveValue::None: out<<kNoneTypeString; break;
  case PrimitiveValue::Integer: out<<v.iValue; break;
  case PrimitiveValue::Double: out<<v.dValue; break;
  case PrimitiveValue::String: SafeOutputString(out,v.sValue); break;
  }
  return out;
}

istream& operator >> (istream& in,PrimitiveValue& v)
{
  EatWhitespace(in);
  int c=in.peek();
  if(c=='\"') {
    if(!InputQuotedString(in,v.sValue)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading a quoted string");
      in.setstate(ios::badbit);
      return in;
    }
    v.sValue = TranslateEscapes(v.sValue);
    v.type = PrimitiveValue::String;
  }
  else {
    v.sValue.erase();
    while(!isspace(c)) {
      v.sValue += c;
      c=in.get();
      c=in.peek();
      if(in.eof()) break;
      else if(!in) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"End of file read while reading a value");
	in.setstate(ios::badbit);
	return in;
      }
    }
    
    //test to see if it can be converted into a value
    if(IsValidInteger(v.sValue.c_str())) {
      v.type = PrimitiveValue::Integer;
      stringstream ss;
      ss.str(v.sValue);
      ss >> v.iValue;
    }
    else if(IsValidFloat(v.sValue.c_str())) {
      v.type = PrimitiveValue::Double;
      stringstream ss;
      ss.str(v.sValue);
      ss >> v.dValue;
    }
    else {
      string temp = v.sValue;
      Lowercase(temp);
      if(temp == "nan") {
	v.type = PrimitiveValue::Double;
	v.dValue = std::numeric_limits<double>::quiet_NaN();
      }
      else if(temp == "inf") {
	v.type = PrimitiveValue::Double;
	v.dValue = std::numeric_limits<double>::infinity();
      }
      else if(temp == "-inf") {
	v.type = PrimitiveValue::Double;
	v.dValue = -std::numeric_limits<double>::infinity();
      }
      else {
	if(v.sValue == kNoneTypeString || v.sValue == "")
	  v.type = PrimitiveValue::None;
	else
	  v.type = PrimitiveValue::String;
      }
    }
  }
  return in;
}

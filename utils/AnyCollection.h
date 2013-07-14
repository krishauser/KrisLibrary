#ifndef ANY_COLLECTION_H
#define ANY_COLLECTION_H

#include "AnyValue.h"
#include "SmartPointer.h"
#include <errors.h>
#include <vector>
#include <map>
#ifdef _MSC_VER
//MSVC doesn't put TR1 files in the tr1/ folder
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif //_MSC_VER

/** @brief Any primitive value (bool, char, unsigned char, int,
 * unsigned int, float, double, string) that we know how to make into a
 * hashable object
 */
struct AnyKeyable
{
  AnyKeyable();
  AnyKeyable(const AnyKeyable& rhs);
  AnyKeyable(bool value);
  AnyKeyable(char value);
  AnyKeyable(unsigned char value);
  AnyKeyable(int value);
  AnyKeyable(unsigned int value);
  AnyKeyable(float value);
  AnyKeyable(double value);
  AnyKeyable(const std::string& value);
  size_t hash() const;
  bool operator == (const AnyKeyable& rhs) const;

  AnyValue value;
};

namespace std {
#ifndef _MSC_VER
//MSVC doesn't put tr1 stuff in the tr1 namespace
namespace tr1 {
#endif //_MSC_VER

template <>
struct hash<AnyKeyable> : public unary_function<AnyKeyable,size_t>
{
  size_t operator()(const AnyKeyable& key) const { return key.hash(); }
};

#ifndef _MSC_VER
//MSVC doesn't put tr1 stuff in the tr1 namespace
} //namespace tr1 
#endif //_MSC_VER
}//namespace std



/** @brief A flexible hierarchical collection of AnyValues, which can
 * be easily initialized to contain primitive values, arrays, or maps.
 * Arrays and maps can be nested as well.
 */
class AnyCollection
{
 public:
  AnyCollection();
  AnyCollection(AnyValue value);
  template <class T>
  AnyCollection(const std::vector<T>& array);
  template <class T,class T2>
  AnyCollection(const std::map<T,T2>& map);
  template <class T,class T2>
  AnyCollection(const std::tr1::unordered_map<T,T2>& map);

  ///returns the number of sub-elements in the collection, or 1 if it is a
  ///primitive data type
  size_t size() const;
  ///returns true if it is a non-primitive data type
  bool collection() const;
  ///depth of nesting: 0 for a primitive data type, 1 for vector or map etc
  size_t depth() const;
  ///cast to AnyValue, if this is a primitive data type
  operator const AnyValue& () const;
  operator AnyValue& ();
  //coerce cast to plain data type
  template<class T>
  operator T() const;
  //coerce cast a flat array type to a vector
  template<class T>
  void asvector(std::vector<T>& values) const;
  //retrieve elements of a flat array type
  void asvector(std::vector<AnyValue>& values) const;
  //test equality with value
  template <class T>
  bool operator == (const T& value) const;
  //test inequality with value
  template <class T>
  bool operator != (const T& value) const { return !operator == (value); }
  //make into an array
  void resize(size_t n);
  //clears to an empty type
  void clear();
  SmartPointer<AnyCollection> find(int i) const;
  SmartPointer<AnyCollection> find(const char* str) const;
  SmartPointer<AnyCollection> find(AnyKeyable key) const;
  AnyCollection& operator[](int i);
  const AnyCollection& operator[](int i) const;
  AnyCollection& operator[](const char* str);
  const AnyCollection& operator[](const char* str) const;
  AnyCollection& operator[](AnyKeyable key);
  const AnyCollection& operator[](AnyKeyable key) const;

  ///shallow copy
  void shallow_copy(const AnyCollection& rhs);
  ///deep copy
  void deep_copy(const AnyCollection& rhs);

  ///shallow copy
  AnyCollection& operator = (const AnyCollection& rhs);
  ///set to a value
  AnyCollection& operator = (AnyValue value);
  ///set to an array
  template <class T>
  AnyCollection& operator = (const std::vector<T>& array);
  ///set to a map
  template <class T,class T2>
  AnyCollection& operator = (const std::map<T,T2>& map);
  ///set to a map
  template <class T,class T2>
  AnyCollection& operator = (const std::tr1::unordered_map<T,T2>& map);

  ///Looks up a reference string, which can be several nested references
  ///deep.  A reference string takes the form S = epsilon|XS' where X is
  ///either ".K" where K is a map key reference or "[K]" where K can be an
  ///integer string, as long as this is an array type.  Otherwise array
  ///references of the form "[K]" are treated as string keys into a map.
  AnyCollection* lookup(const std::string& reference,char delim='.',char lbracket='[',char rbracket=']');

  ///Looks up a composite reference string where array references of the
  ///form [K1:K2] are treated as slices and [K1,K2,...,Kn] are multiple
  ///references.  Constructs a new AnyCollection containing nested references
  ///to the items
  SmartPointer<AnyCollection> slice(const std::string& reference,const char* delims=".[]:,");

  ///if this is a collection, returns the list of sub-collections
  void enumerate(std::vector<SmartPointer<AnyCollection> >& collections) const;
  ///returns an enumerated list of keys contained within
  void enumerate_keys(std::vector<AnyKeyable>& elements) const;
  ///returns an enumerated list of primitive values contained within
  void enumerate_values(std::vector<AnyValue>& elements) const;
  ///returns an enumerated list of elements contained within
  void enumerate_values_dfs(std::vector<AnyValue>& elements) const;

  ///For two collection types, merges the entries of other into this one.
  ///If there are index clashes, the elements of other are taken.
  ///Merging arrays into maps is done by casting indices to int keys.
  void merge(const AnyCollection& other);
  ///Same as merge, but when indices clash, the collection elements of other
  ///are also merged
  void deepmerge(const AnyCollection& other);

  bool read(std::istream& in);
  void write(std::ostream& out) const;

 private:
  enum Type { None, Value, Array, Map };
  typedef AnyValue ValueType;
  typedef std::vector<SmartPointer<AnyCollection> > ArrayType;
  typedef std::tr1::unordered_map<AnyKeyable,SmartPointer<AnyCollection> > MapType;

  Type type;
  ValueType value;
  ArrayType array;
  MapType map;
};

inline std::istream& operator >> (std::istream& in,AnyCollection& c)
{
  bool res=c.read(in);
  if(!res) in.setstate(std::ios::failbit);
  return in;
}


inline std::ostream& operator << (std::ostream& out,const AnyCollection& c)
{
  c.write(out);
  return out;
}

template <class T>
AnyCollection::AnyCollection(const std::vector<T>& array)
  :type(None)
{
  operator = (array);
}

template <class T,class T2>
AnyCollection::AnyCollection(const std::map<T,T2>& map)
  :type(None)
{
  operator = (map);
}

template <class T,class T2>
AnyCollection::AnyCollection(const std::tr1::unordered_map<T,T2>& map)
  :type(None)
{
  operator = (map);
}


template<class T>
AnyCollection::operator T() const
{
  if(type == Value) {
    T result;
    bool res=CoerceCast<T>(&value,result);
    if(!res)
      FatalError("AnyCollection: coercion from %s to %s failed\n",value.type().name(),typeid(T).name());
    return result;
  }
  else {
    FatalError("AnyCollection: not of basic value type\n");
    return T();
  }
}

template <class T>
bool AnyCollection::operator == (const T& v) const
{
  if(type != Value) return false;
  return value == v;
}

template<class T>
void AnyCollection::asvector(std::vector<T>& values) const
{
  std::vector<AnyValue> anyvalues;
  asvector(anyvalues);
  values.resize(anyvalues);
  for(size_t i=0;i<values.size();i++) {
    bool res = CoerceCast<T>(&anyvalues[i],values[i]);
    if(!res)
      FatalError("AnyCollection: coercion from %s to %s failed\n",anyvalues[i].type().name(),typeid(T).name());  
  }
}


template <class T>
AnyCollection& AnyCollection::operator = (const std::vector<T>& _array)
{
  type = Array;
  array.resize(_array.size());
  for(size_t i=0;i<array.size();i++)
    array[i] = new AnyCollection(_array[i]);
  return *this;
}

template <class T,class T2>
AnyCollection& AnyCollection::operator = (const std::map<T,T2>& _map)
{
  type = Map;
  for(typename std::map<T,T2>::const_iterator i=_map.begin();i!=_map.end();i++)
    map[i->first] = new AnyCollection(i->second);
  return *this;
}

template <class T,class T2>
AnyCollection& AnyCollection::operator = (const std::tr1::unordered_map<T,T2>& _map)
{
  type = Map;
  for(typename std::tr1::unordered_map<T,T2>::const_iterator i=_map.begin();i!=_map.end();i++)
    map[i->first] = new AnyCollection(i->second);
  return *this;
}

#endif

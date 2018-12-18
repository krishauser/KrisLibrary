#ifndef ANY_COLLECTION_H
#define ANY_COLLECTION_H

#include <KrisLibrary/Logger.h>
#include "AnyValue.h"
#include <KrisLibrary/errors.h>
#include <vector>
#include <map>
#include <memory>
#include "stl_tr1.h"

/** @brief Any primitive value (bool, char, unsigned char, int,
 * unsigned int, float, double, string) that we know how to make into a
 * hashable object.
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
  AnyKeyable(const char* value);
  size_t hash() const;
  bool operator == (const AnyKeyable& rhs) const;

  AnyValue value;
};

BEGIN_TR1_NAMESPACE

template <>
struct hash<AnyKeyable> : public unary_function<AnyKeyable,size_t>
{
  size_t operator()(const AnyKeyable& key) const { return key.hash(); }
};

END_TR1_NAMESPACE



/** @brief A flexible hierarchical collection of AnyValues, which can
 * be easily initialized to contain primitive values, arrays, or maps.
 * Arrays and maps can be nested as well.
 * 
 * AnyCollection c;
 * c["firstName"] = "John";  //automatically sets c to a map
 * c["lastName"] = "Smith";
 * c["age"] = 25;
 * c["phoneNumbers"][0] = "123-4567";  //automatically sets c["phoneNumbers"] to an array
 * c["phoneNumbers"][1] = "890-1234";
 * Assert(int(c["age"]) == 25);
 * Assert(string(c["age"]) == "25");
 *
 * An AnyCollection v representing a primitive value can be cast to a given
 * type T using T(v). This will be cast appropriately to the type of value.
 * Or, if value is of type T, v.as(T) will perform error checking and return
 * false if v is not of the right type.
 *
 * AnyCollections are written to / read from JSON format.
 *
 * Notes on cross-DLL passing of AnyCollections: as of now, this is not 
 * functional because C++ RTTI is done via comparison of typeinfo *pointers*,
 * which are different between the DLL and main program.  Look into the
 * #defines at the top of AnyCollection.cpp if you want to be able to pass
 * these objects between DLLs.
 */
class AnyCollection
{
 public:
  typedef std::shared_ptr<AnyCollection> AnyCollectionPtr;

  AnyCollection();
  AnyCollection(AnyValue value);
  template <class T>
  AnyCollection(const std::vector<T>& array);
  template <class T,class T2>
  AnyCollection(const std::map<T,T2>& map);
  template <class T,class T2>
  AnyCollection(const UNORDERED_MAP_TEMPLATE<T,T2>& map);

  ///returns the number of sub-elements in the collection, or 1 if it is a
  ///primitive data type
  size_t size() const;
  ///returns true if this is a null 
  bool null() const;
  ///returns true if it is a non-primitive data type
  bool collection() const;
  ///returns true if this is a primitive data type
  bool isvalue() const;
  ///returns true if this is an array data type
  bool isarray() const;
  ///returns true if this is a map data type
  bool ismap() const;
  ///depth of nesting: 0 for a primitive data type, 1 for vector or map etc
  size_t depth() const;
  ///cast to AnyValue, if this is a primitive data type
  operator const AnyValue& () const;
  operator AnyValue& ();
  //coerce cast to plain data type
  template<class T>
  operator T() const;
  //coerce cast to plain data type
  template<class T>
  bool as(T& value) const;
  //coerce cast a flat array type to a vector
  template<class T>
  bool asvector(std::vector<T>& values) const;
  //retrieve elements of a flat array type
  bool asvector(std::vector<AnyValue>& values) const;
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
  //accessors
  AnyCollectionPtr find(int i) const;
  AnyCollectionPtr find(const char* str) const;
  AnyCollectionPtr find(AnyKeyable key) const;
  AnyCollectionPtr insert(int i);
  AnyCollectionPtr insert(const char* str);
  AnyCollectionPtr insert(AnyKeyable key);
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
  ///set to a primitive data type that can accidentally be cast to a char*
  AnyCollection& operator = (bool v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (char v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (unsigned char v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (int v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (unsigned int v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (float v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (double v) { return operator = (AnyValue(v)); }
  AnyCollection& operator = (const std::string& str) { return operator = (AnyValue(str)); }
  ///set to a C-string
  AnyCollection& operator = (const char* str) { return operator = (std::string(str)); }
  ///set to an array
  template <class T>
  AnyCollection& operator = (const std::vector<T>& array);
  ///set to a map
  template <class T,class T2>
  AnyCollection& operator = (const std::map<T,T2>& map);
  ///set to a map
  template <class T,class T2>
  AnyCollection& operator = (const UNORDERED_MAP_TEMPLATE<T,T2>& map);

  ///Looks up a reference string, which can be several nested references
  ///deep.  A reference string takes the form S = epsilon|XS' where X is
  ///either ".K" where K is a map key reference or "[K]" where K can be an
  ///integer string, as long as this is an array type.  Otherwise array
  ///references of the form "[K]" are treated as string keys into a map.
  AnyCollectionPtr lookup(const std::string& reference,bool insert=false,char delim='.',char lbracket='[',char rbracket=']');
  AnyCollectionPtr lookup(const std::vector<std::string>& path,bool insert=false);
  AnyCollectionPtr lookup(const std::vector<AnyKeyable>& path,bool insert=false);
  ///parse a reference string into a path
  static bool parse_reference(const std::string& reference,std::vector<std::string>& path,char delim='.',char lbracket='[',char rbracket=']');
  ///converts a string path to a key path matching the array/map structure of
  ///this object.  will be smart in converting string keys to integer keys
  ///if this object contains an integer key matching the string
  bool match_path(const std::vector<std::string>& path,std::vector<AnyKeyable>& key_path) const;

  ///Looks up a composite reference string where array references of the
  ///form [K1:K2] are treated as slices and [K1,K2,...,Kn] are multiple
  ///references.  Constructs a new AnyCollection containing nested references
  ///to the items
  AnyCollectionPtr slice(const std::string& reference,const char* delims=".[]:,");

  ///Adds all elements referenced by the given paths into a sub-collection,
  ///copying the nesting structure of this item.
  ///
  ///TODO: slice references are not done yet. 
  ///
  ///Example:
  /// {a:[1,2,3,4],b:[5,6],c:{foo:7,bar:8}}.subcollection(["a[1]","a[2]","c[foo]"]) 
  /// => {a:{1:2,2:3},c:{foo:7}}
  bool subcollection(const std::vector<std::string>& paths,AnyCollection& subset,const char* delims=".[]:,");

  ///if this is a collection, returns the list of sub-collections
  void enumerate(std::vector<AnyCollectionPtr >& collections) const;
  ///returns an enumerated list of keys contained within
  void enumerate_keys(std::vector<AnyKeyable>& elements) const;
  ///returns an enumerated list of primitive values contained within
  void enumerate_values(std::vector<AnyValue>& elements) const;
  ///returns an enumerated list of keys contained within
  void enumerate_keys_dfs(std::vector<std::vector<AnyKeyable> >& paths) const;
  ///returns an enumerated list of elements contained within
  void enumerate_values_dfs(std::vector<AnyValue>& elements) const;

  ///For two collection types, merges the entries of other into this one.
  ///If there are index clashes, the elements of other are taken.
  ///Merging arrays into maps is done by casting indices to int keys.
  void merge(const AnyCollection& other);
  ///Same as merge, but when indices clash, the collection elements of other
  ///are also merged
  void deepmerge(const AnyCollection& other);
  ///recursively fills the primitive entries of this collection with the
  ///corresponding items in the universe.  If checkSuperset = true,
  ///then this will check to see if the universe contains a superset of keys.
  ///false is returned if the universe does not match the structure of this
  ///collection
  bool fill(AnyCollection& universe,bool checkSuperset=false);

  ///Reads in JSON format
  bool read(std::istream& in);
  bool read(const char* data);
  ///Writes in JSON format
  void write(std::ostream& out,int indent=0) const;
  ///Same as write, but puts everything onto one line
  void write_inline(std::ostream& out) const;

 private:
  enum Type { None, Value, Array, Map };
  typedef AnyValue ValueType;
  typedef std::vector<AnyCollectionPtr > ArrayType;
  typedef UNORDERED_MAP_TEMPLATE<AnyKeyable,AnyCollectionPtr > MapType;

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
AnyCollection::AnyCollection(const UNORDERED_MAP_TEMPLATE<T,T2>& map)
  :type(None)
{
  operator = (map);
}


template<class T>
AnyCollection::operator T() const
{
  T res;
  if(!as(res)) {
    if(type == Value)
      FatalError("AnyCollection: coercion from %s to %s failed\n",value.type().name(),typeid(T).name());
    else if(type == None)
      FatalError("AnyCollection: coersion to %s failed, item does not exist\n",typeid(T).name());
    else
      FatalError("AnyCollection: coersion to %s failed, not of value type\n",typeid(T).name());
  }
  return res;
}

template<class T>
bool AnyCollection::as(T& result) const
{
  if(type == Value) {
    bool res=CoerceCast<T>(value,result);
    return res;
  }
  else {
    return false;
  }
}

template <class T>
bool AnyCollection::operator == (const T& v) const
{
  if(type != Value) return false;
  return value == v;
}

template<class T>
bool AnyCollection::asvector(std::vector<T>& values) const
{
  std::vector<AnyValue> anyvalues;
  if(!asvector(anyvalues)) return false;
  values.resize(anyvalues.size());
  for(size_t i=0;i<values.size();i++) {
    bool res = CoerceCast<T>(anyvalues[i],values[i]);
    if(!res) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Coerce cast "<<anyvalues[i].type().name()<<" to "<<typeid(T).name()<<" failed for element "<<(int)i);
      return false;
    }
  }
  return true;
}


template <class T>
AnyCollection& AnyCollection::operator = (const std::vector<T>& _array)
{
  type = Array;
  array.resize(_array.size());
  for(size_t i=0;i<array.size();i++)
    array[i] = std::make_shared<AnyCollection>(_array[i]);
  return *this;
}

template <class T,class T2>
AnyCollection& AnyCollection::operator = (const std::map<T,T2>& _map)
{
  type = Map;
  for(typename std::map<T,T2>::const_iterator i=_map.begin();i!=_map.end();i++)
    map[i->first] = std::make_shared<AnyCollection>(i->second);
  return *this;
}

template <class T,class T2>
AnyCollection& AnyCollection::operator = (const UNORDERED_MAP_TEMPLATE<T,T2>& _map)
{
  type = Map;
  for(typename UNORDERED_MAP_TEMPLATE<T,T2>::const_iterator i=_map.begin();i!=_map.end();i++)
    map[i->first] = std::make_shared<AnyCollection>(i->second);
  return *this;
}

#endif

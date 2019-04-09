#ifndef UTILS_PROPERTY_MAP_H
#define UTILS_PROPERTY_MAP_H


#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
class TiXmlElement;

/** @brief A simple map from keys to values.
 * 
 * Values are easily converted to basic types using the as<T>(key) method.
 * This method will convert the string into a stream and use the >> operator
 * to convert to the desired type.
 *
 * Basic types are conveted into values using the set(key,value) method.
 * This will convert the value to a string using the << operator.
 *
 * Property maps also support basic whitespace-separated arrays. getArray,
 * setArray, and asArray can be used to set/get those arrays.
 *
 * Property maps can be easily converted to/from JSON objects and XML 
 * node attributes.  Loading/saving to disk is done through JSON serialization.
 */
class PropertyMap : public std::map<std::string,std::string>
{
 public:
  PropertyMap() {}
  PropertyMap(const std::map<std::string,std::string>& );
  template <class T>
  PropertyMap(const std::map<std::string,T>& );
  template <class T>
  PropertyMap(const std::vector<std::string>& keys,const std::vector<T>& values);
  inline bool contains(const std::string& key) const { return count(key) != 0; }
  bool remove(const std::string& key);
  void set(const std::string& key,const std::string& value);
  bool get(const std::string& key,std::string& value) const;
  std::string as(const std::string& key) const;
  void setArray(const std::string& key,const std::vector<std::string>& values);
  bool getArray(const std::string& key,std::vector<std::string>& values) const;
  std::vector<std::string> asArray(const std::string& key) const;
  template <class T>
  void set(const std::string& key,const T& value);
  template <class T>
  bool get(const std::string& key,T& value) const;
  template <class T>
  T as(const std::string& key) const;
  template <class T>
  T getDefault(const std::string& key,T defaultValue=T()) const;
  template <class T>
  void setArray(const std::string& key,const std::vector<T>& value);
  template <class T>
  bool getArray(const std::string& key,std::vector<T>& values) const;
  template <class T>
  std::vector<T> asArray(const std::string& key) const;

  bool Load(TiXmlElement* node);
  bool Save(TiXmlElement* node) const;
  bool LoadJSON(std::istream& in);
  bool SaveJSON(std::ostream& out) const;
  bool Load(const char* fn);
  bool Save(const char* fn) const;
  void Print(std::ostream& out) const;
};

inline std::ostream& operator << (std::ostream& out,const PropertyMap& pmap)
{
  pmap.SaveJSON(out);
  return out;
}

inline std::istream& operator >> (std::istream& in,PropertyMap& pmap)
{
  if(!pmap.LoadJSON(in))
    in.setstate(std::ios::failbit);
  return in;
}


template <class T>
PropertyMap::PropertyMap(const std::map<std::string,T>& rhs)
{
  for(const auto& i=rhs.begin();i!=rhs.end();++i)
    set(i->first,i->second);
}

template <class T>
PropertyMap::PropertyMap(const std::vector<std::string>& keys,const std::vector<T>& values)
{
  for(size_t i=0;i<keys.size();i++)
    set(keys[i],values[i]);
}

template <class T>
T PropertyMap::as(const std::string& key) const
{
  T res;
  get(key,res);
  return res;
}

template <class T>
T PropertyMap::getDefault(const std::string& key,T defaultValue) const
{
  T res;
  if(!get(key,res))
    return defaultValue;
  return res;
}

template <class T>
bool PropertyMap::get(const std::string& key,T& value) const
{
  const_iterator i=find(key);
  if(i==end()) return false;
  std::stringstream ss(i->second);
  ss>>value;
  if(!ss) return false;
  return true;
}

template <class T>
void PropertyMap::set(const std::string& key,const T& value)
{
  std::stringstream ss;
  ss<<value;
  (*this)[key] = ss.str();
}

template <class T>
bool PropertyMap::getArray(const std::string& key,std::vector<T>& values) const
{
  const_iterator i=find(key);
  if(i==end()) return false;
  std::stringstream ss(i->second);
  T temp;
  values.resize(0);
  while(ss) {
    ss>>temp;
    if(ss) 
      values.push_back(temp);
  }
  return true;
}


template <class T>
void PropertyMap::setArray(const std::string& key,const std::vector<T>& values)
{
  std::stringstream ss;
  for(size_t i=0;i<values.size();i++) {
    if(i > 0) ss<<" ";
    ss<<values[i];
  }
  set(key,ss.str());
}



template <class T>
std::vector<T> PropertyMap::asArray(const std::string& key) const
{
  std::vector<T> res;
  getArray(key,res);
  return res;
}


#endif

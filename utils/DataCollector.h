#ifndef UTILS_DATA_COLLECTOR_H
#define UTILS_DATA_COLLECTOR_H

#include <string>
#include <map>
#include <vector>
#include <typeinfo>
#include "SmartPointer.h"

/** @ingroup Utils
 * @brief A helper class that points to any kind of data.
 * 
 * The type is named, and the user is responsible for passing in the
 * appropriate name if RTTI is not enabled.
 *
 * Upon instantiation, data is copied using the type's copy constructor.
 * The AnyData copy constructor doesn't actually copy data because it
 * stores data in a reference-counted SmartPointer.
 */
struct AnyData
{
  ///Instantiates the data as empty
  AnyData() {}

  ///Instantiates the data with just a value, determining its type using RTTI.
  template <class T>
  AnyData(const T& _value)
  {
    Set(_value);
  }


  ///Instantiates the data with the name of T specified.
  ///Use this when RTTI is not enabled.
  template <class T>
  AnyData(const std::string& _type,const T& _value)
  {
    Set(_type,_value);
  }

  void Clear()
  {
    type.erase();
    value = NULL;
  }

  ///Instantiates the data with the name of T specified.
  ///Use this when RTTI is not enabled.
  template <class T>
  void Set(const std::string& _type,const T& _value)
  {
    type=_type;
    value = reinterpret_cast<int*>(new T(_value));
  }

  ///Instantiates the data with just a value, determining its type using RTTI.
  template <class T>
  void Set(const T& _value) { Set(std::string(typeid(_value).name()),_value); }

  ///If the string is the right type, returns the value pointer, cast to a T*.
  ///Otherwise, returns NULL.
  template <class T>
  T* Get(const std::string& _type) {
    if(type == _type) return reinterpret_cast<T*>((int*)value);
    return NULL;
  }

  ///Const version of the above
  template <class T>
  const T* Get(const std::string& _type) const {
    if(type == _type) return reinterpret_cast<const T*>((const int*)value);
    return NULL;
  }

  ///Uses RTTI to obtain the type string for T
  template <class T>
  T* Get() { return Get<T>(std::string(typeid(T).name())); }
  ///Const version of the above
  template <class T>
  const T* Get() const { return Get<T>(std::string(typeid(T).name())); }


  std::string type;
  SmartPointer<int> value;
};


/** @ingroup Utils
 * @brief A heirarchical database of arbitrary values.
 *
 * Each entry is indexed by a string of the form "str1:str2:str3:...:strn"
 * that denotes a path in the tree.
 *
 * Insert<T>(name,type,object)
 *   Inserts a new object into the database indexed by name,
 *   overwriting any old value that was there.
 * Find<T>(name) 
 * Find<T>(name,type) 
 *   Returns a pointer to the object indexed by name, if one exists.
 *   Otherwise, returns NULL.
 * Get<T>(name) 
 * Get<T>(name,type) 
 *   If the object indexed by name does not exist, inserts a new object
 *   with no arguments.  Returns a pointer to the object indexed by name.
 */
struct DataDatabase
{
  struct Entry : public AnyData
  {
    Entry() {}

    std::map<std::string,Entry> children;
  };

  DataDatabase();
  void Clear();
  Entry& Insert(const std::string& name);
  const Entry* Find(const std::string& name) const;
  Entry* Find(const std::string& name);
  void Erase(const std::string& name);
  void Rename(const std::string& name,const std::string& newName);

  //convenience functions
  template <class T>
  void Insert(const std::string& name,const std::string& type,const T& data) { Insert(name).Set(type,data); }
  template <class T>
  void Insert(const std::string& name,const std::string& child,const std::string& type,const T& data) { return Insert(Concat(name,child),type,data); }
  template <class T>
  void Insert(const std::string& name,const std::string& child1,const std::string& child2,const std::string& type,const T& data) { Insert(Concat(name,child1,child2),type,data); }
  template <class T>
  void Insert(const std::string& n,const std::string& c1,const std::string& c2,const std::string& c3,const std::string& type,const T& data) { Insert(Concat(n,c1,c2,c3),type,data); }
  template <class T>
  const T* Find(const std::string& name,const std::string& type) const {
    const Entry* e= Find(name);
    if(e) return e->Get<T>(type);
    return NULL;
  }
  template <class T>
  const T* Find(const std::string& name) const {
    const Entry* e= Find(name);
    if(e) return e->Get<T>();
    return NULL;
  }
  template <class T>
  T* Find(const std::string& name,const std::string& type) {
    Entry* e= Find(name);
    if(e) return e->Get<T>(type);
    return NULL;
  }
  template <class T>
  T* Find(const std::string& name) {
    Entry* e= Find(name);
    if(e) return e->Get<T>();
    return NULL;
  }
  template <class T>
  T* Get(const std::string& name) {
    return Get<T>(name,typeid(T).name());
  }
  template <class T>
  T* Get(const std::string& name,const std::string& type) {
    Entry& e=Insert(name);
    if(e.type != type) 
      e.Set<T>(type,T());
    return e.Get<T>(type);
  }

  //helpers
  void NameToPath(const std::string& name,std::vector<std::string>& path) const;
  std::string Concat(const std::string& s1,const std::string& s2) const;
  std::string Concat(const std::string& s1,const std::string& s2,const std::string& s3) const;
  std::string Concat(const std::string& s1,const std::string& s2,const std::string& s3,const std::string& s4) const;
  std::string Concat(const std::vector<std::string>& s) const;

  unsigned char delim;
  Entry root;
};

#endif

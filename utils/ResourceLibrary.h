#ifndef RESOURCE_LIBRARY_H
#define RESOURCE_LIBRARY_H

#include "SmartPointer.h"
#include <string>
#include <iostream>
#include <typeinfo>
#include <map>
#include <vector>

class TiXmlElement;

/**@brief A generic "resource" that can be saved/loaded to disk.
 *
 * The subclass must support saving through one or more of the
 * following methods
 * - to a file on disk: Load/Save(string)
 * - to iostreams: Load(istream)/Save(ostream)
 * - to xml nodes: Load/Save(TiXmlElement*)
 *
 * If the resource library is saved to a directory, then each entry
 * is stored to a separate file.  
 *
 * If the resource library is saved to an xml file, then each entry
 * that supports saving to xml is saved directly in the xml file.
 * If it supports iostreams, then it is outputted directly as a text
 * node in the xml file.  If it only supports saving to file, then 
 * the filename is stored in the xml file and the resource is dumped
 * to disk.
 */
class ResourceBase
{
 public:
  ResourceBase();
  ResourceBase(const std::string& name);
  ResourceBase(const std::string& name,const std::string& fn);
  virtual ~ResourceBase() {}
  virtual bool Load(const std::string& fn);
  virtual bool Load();
  virtual bool Save(const std::string& fn);
  virtual bool Save();
  virtual bool Load(std::istream& in) { return false; }
  virtual bool Save(std::ostream& out) { return false; }
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  //A unique type string used for type indexing and xml output, only 
  //alphanumeric characters allowed.
  virtual const char* Type() const { return typeid(*this).name(); }
  //ResourceBase's are their own factories
  virtual ResourceBase* Make() { return new ResourceBase; }

  std::string name,fileName;
};

typedef SmartPointer<ResourceBase> ResourcePtr;

/** @brief A basic data type.
 *
 * Assumes the data type has std::ostream << and std::istream >> overloads.
 */
template <class T>
class BasicResource : public ResourceBase
{
 public:
  BasicResource() {}
  BasicResource(const T& val) : data(val) {}
  BasicResource(const T& val,const std::string& name) : ResourceBase(name),data(val) {}
  virtual ~BasicResource() {}
  virtual bool Load(std::istream& in) {
    in>>data;
    if(in.bad()) {
      return false;
    }
    return true;
  }
  virtual bool Save(std::ostream& out) {
    out<<data<<std::endl;
    return true;
  }
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save(const std::string& fn) { return ResourceBase::Save(fn); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual const char* Type() const { return className; }
  virtual ResourceBase* Make() { return new BasicResource<T>; }

  T data;
  static const char* className;
};

typedef BasicResource<int> IntResource;
typedef BasicResource<double> FloatResource;
typedef BasicResource<std::string> StringResource;

/** @brief A collection of resources, which may be loaded/saved as separate
 * files in a directory or inlined into a single XML file.  Helps with
 * managing subdirectories of data.
 *
 * Inlined XML files are structured:
 * <resource_library>
 *    <{resource.Type()} name={resource.name} file={resource.fileName}> data </{resource.Type()}>
 *  
 * </resource_library>
 * where data is given if the item supports inline reading/writing to 
 * iostreams, the xml sub-tree if it supports saving to xml,
 * and otherwise it is omitted.  In the latter case, the resource
 * is saved to its fileName.
 *
 * Resources are accessed either by name (Get()) or type (GetByType()).
 */
class ResourceLibrary
{
 public:
  ///Adds a new type not associated with a given filename
  template <class T>
  void AddType();
  ///Adds a new type associated with the given file extension ext
  template <class T>
  void AddLoader(const std::string& ext);

  ///Erases all elements (maintains loaders)
  void Clear();
  ///Saves to an XML file
  bool SaveXml(const std::string& fn);
  ///Loads from an XML file
  bool LoadXml(const std::string& fn);
  ///Saves to an XML element
  bool Save(TiXmlElement* e);
  ///Loads from an XML element
  bool Load(TiXmlElement* e);
  ///Loads from an XML file without loading external files
  bool LazyLoadXml(const std::string& fn);
  ///Prepends the given directory onto each resource's filename
  void AddBaseDirectory(const std::string& dir);
  ///Changes the first directory in each resource's filename to dir.
  ///If the filename is not in a subdirectory, dir is prepended.
  ///If dir is empty, this deletes a base directory from the filename
  void ChangeBaseDirectory(const std::string& dir);
  ///Saves all resources to their given fileName's
  bool SaveAll();
  ///Saves the named resource only
  bool SaveItem(const std::string& name);
  ///Loads all resources in a given path
  bool LoadAll(const std::string& dir);
  ///Loads all resources in a given path without loading external files
  bool LazyLoadAll(const std::string& dir);
  ///Loads a new resource from the given file and adds it to the library
  ResourcePtr LoadItem(const std::string& fn);
  ///Reloads all resources from their given filenames
  bool ReloadAll();

  //accessors
  template <class T>
  T* Get(const std::string& name,int index);
  ///Get by name 
  std::vector<ResourcePtr>& Get(const std::string& name);
  ///Get by type
  std::vector<ResourcePtr>& GetByType(const std::string& type);
  template <class T>
  std::vector<ResourcePtr>& GetByType();
  template <class T>
  std::vector<T*> GetPtrsByType();
  ///Count by name
  size_t Count(const std::string& name) const;
  ///Count by type
  size_t CountByType(const std::string& type) const;
  template <class T>
  size_t CountByType() const;

  //modifiers
  void Add(const ResourcePtr& resource);
  bool Erase(const ResourcePtr& resource);
  bool Erase(const std::string& name,int index=-1);

  //helpers
  std::string DefaultFileName(const ResourcePtr& r);

  typedef std::map<std::string,std::vector<ResourcePtr> > Map;
  Map knownTypes;
  Map loaders;
  Map itemsByName,itemsByType;
};

/** @brief A resource that contains a ResourceLibrary.  Useful for
 * hierarchical resource libraries.
 */
class ResourceLibraryResource : public ResourceBase
{
 public:
  virtual bool Load(const std::string& fn);
  virtual bool Save(const std::string& fn);
  virtual const char* Type() const { return "ResourceLibrary"; }
  virtual ResourceBase* Make() { return new ResourceLibraryResource; }

  ResourceLibrary library;
};

template <class T>
void ResourceLibrary::AddType()
{
  ResourcePtr res=new T;
  if(knownTypes.count(res->Type()) == 0) {
    knownTypes[res->Type()].push_back(res);
  }
  else {
    //potential conflict? check raw string
    if(knownTypes[res->Type()][0]->Type() != res->Type())
      fprintf(stderr,"ResourceLibrary: Potential conflict with type %s\n",res->Type());
    knownTypes[res->Type()][0] = res;
  }
}

template <class T>
void ResourceLibrary::AddLoader(const std::string& ext)
{
  AddType<T>();
  ResourcePtr res=new T;
  loaders[ext].push_back(res);
}

template <class T>
T* ResourceLibrary::Get(const std::string& name,int index)
{
  std::vector<ResourcePtr>& items=Get(name);
  if(index < 0 || index > (int)items.size()) return NULL;
  return dynamic_cast<T*>((ResourceBase*)items[index]);
}

template <class T>
std::vector<ResourcePtr>& ResourceLibrary::GetByType()
{
  T temp;
  return GetByType(temp.Type());
}

template <class T>
std::vector<T*> ResourceLibrary::GetPtrsByType()
{
  std::vector<ResourcePtr>& items = GetByType<T>();
  std::vector<T*> cast_items(items.size());
  for(size_t i=0;i<items.size();i++)
    cast_items[i] = dynamic_cast<T*>((ResourceBase*)items[i]);
  return cast_items;
}

template <class T>
size_t ResourceLibrary::CountByType() const
{
  T temp;
  return CountByType(temp.Type());
}

#endif

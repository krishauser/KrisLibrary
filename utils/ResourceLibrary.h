#ifndef RESOURCE_LIBRARY_H
#define RESOURCE_LIBRARY_H

#include <KrisLibrary/Logger.h>
#include "AnyCollection.h"
#include <memory>
#include <string>
#include <string.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <map>
#include <vector>

class TiXmlElement;

/**@brief A generic "resource" that can be saved/loaded to disk.
 *
 * The subclass must support saving through one or more of the
 * following methods
 * - to a file on disk: Load/Save(string filename)
 * - to iostreams: Load(istream)/Save(ostream)
 * - to xml nodes: Load/Save(TiXmlElement*)
 * - to AnyCollections: Load/Save(AnyCollection&)
 *   (these are used by ResourceLibrary's to save to JSON)
 * Note: subclass should not save the "name" or "fileName" members, nor
 * parse the "name" or "file" attributes of xml elements or collections.
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
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  ///A unique type string used for type indexing and xml output, only 
  ///alphanumeric characters allowed.
  virtual const char* Type() const { return typeid(*this).name(); }
  ///Make a ResourceBase of the same dynamic type as this.
  ///ResourceBase instances are their own factories.
  virtual ResourceBase* Make() { return new ResourceBase; }
  ///Make a ResourceBase that copies all the contents of this.
  ///(Copying the name and fileName are optional)
  virtual ResourceBase* Copy() { return new ResourceBase(name,fileName); }

  std::string name,fileName;
};

typedef std::shared_ptr<ResourceBase> ResourcePtr;


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
 * Inlined JSON files are structured
 * [
 *    {type:resource.Type(),
 *     name:resource.name,
 *     file:resource.fileName,
 *     data:resource.data
 *    }
 * ]
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
  ///Saves to a JSON stream
  bool SaveJSON(std::ostream& s);
  ///Loads from a JSON stream
  bool LoadJSON(std::istream& s);
  ///Loads from a JSON stream without loading external files
  bool LazyLoadJSON(std::istream& s);
  ///Saves to an AnyCollection
  bool Save(AnyCollection& c);
  ///Loads from an AnyCollection
  bool Load(AnyCollection& c);
  ///Loads from an AnyCollection without loading external files
  bool LazyLoad(AnyCollection& c);
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
  ///Return all
  std::vector<ResourcePtr> Enumerate() const;

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


///Extracts only the resources of type T
template <class T>
std::vector<T*> ResourcesByType(std::vector<ResourcePtr>& resources)
{
  std::vector<T*> rtype;
  for(size_t i=0;i<resources.size();i++) {
    T* ptr = dynamic_cast<T*>(resources[i].get());
    if(ptr) rtype.push_back(ptr);
  }
  return rtype;
}

///Extracts only the resources whose type string matches type
std::vector<ResourcePtr> ResourcesByType(std::vector<ResourcePtr>& resources,const std::string& type);

///Returns the set of type strings covered by the given resources
std::vector<std::string> ResourceTypes(const std::vector<ResourcePtr>& resources);



///Override this to give a Type() string for a basic resource
template <class T>
const char* BasicResourceTypeName() { return typeid(T).name(); }

/** @brief A basic data type.
 *
 * Assumes the data type has std::ostream << and std::istream >> overloads.
 *
 * The type T is by default represented by the string returned to by typeid(T).name().
 * This is typically a mangled string that is compiler-dependent.
 *
 * To override this, you must override the template function BasicResourceTypeName<T>().
 */
template <class T>
class BasicResource : public ResourceBase
{
 public:
  BasicResource() {}
  BasicResource(const T& val) : data(val) {}
  BasicResource(const T& val,const std::string& name) : ResourceBase(name),data(val) {}
  BasicResource(const T& val,const std::string& name,const std::string& fileName) : ResourceBase(name,fileName),data(val) {}
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
  virtual const char* Type() const { return BasicResourceTypeName<T>(); }
  virtual ResourceBase* Make() { return new BasicResource<T>; }
  virtual ResourceBase* Copy() { return new BasicResource<T>(data,name,fileName); }

  T data;
};

template <> const char* BasicResourceTypeName<int>();
template <> const char* BasicResourceTypeName<double>();
template <> const char* BasicResourceTypeName<std::string>();
typedef BasicResource<int> IntResource;
typedef BasicResource<double> FloatResource;
typedef BasicResource<std::string> StringResource;


/** @brief A resource that is composed of multiple sub-objects, which can
 * themselves be considered resources.
 *
 * Subclasses should overload casting, extracting, packing, and unpacking
 * methods to implement compound functionality.
 */
class CompoundResourceBase : public ResourceBase
{
 public:
  CompoundResourceBase() {}
  CompoundResourceBase(const std::string& name) : ResourceBase(name) {}
  CompoundResourceBase(const std::string& name,const std::string& fileName) : ResourceBase(name,fileName) {}
  virtual ~CompoundResourceBase() {}

  ///Returns a list of subtypes that can be used in Cast
  virtual std::vector<std::string> CastTypes() const { return std::vector<std::string>(); }
  ///Returns a list of subtypes that can be used in Extract
  virtual std::vector<std::string> ExtractTypes() const { return SubTypes(); }
  ///Returns a list of subtypes that will be produced in Unpack
  virtual std::vector<std::string> SubTypes() const { return std::vector<std::string>(); }

  /** Returns a copy of the resource cast to a resource of another type */
  virtual ResourcePtr Cast(const char* type) { return NULL; }

  /** Extracts all items of the given subtype.  Default uses Unpack to 
   find objects of the right type. */
  virtual bool Extract(const char* subtype,std::vector<ResourcePtr>& subobjects);

  /** Creates the object of out of the given resources, returning true if
   * successful.  If unsuccessful, the error message may be set to an
   * informative description of the error.
   */
  virtual bool Pack(std::vector<ResourcePtr>& subobjects,std::string* errorMessage=NULL) { if(errorMessage) *errorMessage = std::string(Type())+"::Pack not implemented"; return false; }

  /** Creates a list of sub-objects that describe the given resource.
   * If the decomposition succeeded, returns true
   * If the decomposition succeeded but is incomplete, the flag
   * incomplete is set to true (if non-null).
   * Here "incomplete" means that calling Pack on the return array may
   * NOT produce an exact copy of r.
   */
  virtual bool Unpack(std::vector<ResourcePtr>& subobjects,bool* incomplete=NULL) { return false; }
};

/** @brief A basic array data type.
 *
 * Assumes the data type has std::ostream << and std::istream >> overloads.
 */
template <class T>
class BasicArrayResource : public CompoundResourceBase
{
 public:
  typedef BasicResource<T> BaseType;
  BasicArrayResource() {}
  BasicArrayResource(const std::vector<T>& val) : data(val) {}
  BasicArrayResource(const std::vector<T>& val,const std::string& name) : CompoundResourceBase(name),data(val) {}
  BasicArrayResource(const std::vector<T>& val,const std::string& name,const std::string& fileName) : CompoundResourceBase(name,fileName),data(val) {}
  virtual ~BasicArrayResource() {}
  virtual bool Load(std::istream& in) {
    size_t n;
    in>>n;
    if(in.bad()) return false;
    data.resize(n);
    for(size_t i=0;i<n;i++) {
      in>>data[i];
      if(in.bad()) return false; 
    }
    return true;
  }
  virtual bool Save(std::ostream& out) {
    out<<data.size()<<'\t';
    for(size_t i=0;i<data.size();i++) {
      out<<data[i];
      if(i+1!=data.size()) out<<" ";
    }
    out<<std::endl;
    return true;
  }
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save(const std::string& fn) { return ResourceBase::Save(fn); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual bool Load(AnyCollection& c) { return c["data"].asvector(data); }
  virtual bool Save(AnyCollection& c) { c["data"] = data; return true;}
  virtual const char* Type() const { return BasicResourceTypeName<std::vector<T> >(); }
  virtual ResourceBase* Make() { return new BasicArrayResource<T>; }
  virtual ResourceBase* Copy() { return new BasicArrayResource<T>(data,name,fileName); }

  //CompoundResourceBase methods
  virtual std::vector<std::string> SubTypes() const { return std::vector<std::string>(1,BasicResourceTypeName<T>()); }
  virtual bool Extract(const char* subtype,std::vector<ResourcePtr>& subobjects) {
    if(0==strcmp(subtype,BasicResourceTypeName<T>()))
      return Unpack(subobjects);
    return false;
  }
  virtual bool Pack(std::vector<ResourcePtr>& subobjects,std::string* errorMessage=NULL) {
    for(size_t i=0;i<subobjects.size();i++)
      if(typeid(*subobjects[i]) != typeid(BaseType)) {
	if(errorMessage) *errorMessage = std::string("Subobject does not have type ")+std::string(BasicResourceTypeName<T>());
	return false;
      }
    data.resize(subobjects.size());
    for(size_t i=0;i<subobjects.size();i++)    
      data[i] = dynamic_cast<BaseType*>(&*subobjects[i])->data;
    return true;
  }
  virtual bool Unpack(std::vector<ResourcePtr>& subobjects,bool* incomplete=NULL) {
    subobjects.resize(data.size());
    for(size_t i=0;i<data.size();i++) {
      subobjects[i] = std::make_shared<BaseType>(data[i]);
      std::stringstream ss;
      ss<<"data["<<i<<"]";
      subobjects[i]->name = ss.str();
    }
    return true;
  }

  std::vector<T> data;
};

template <> const char* BasicResourceTypeName<std::vector<int> >();
template <> const char* BasicResourceTypeName<std::vector<double> >();
template <> const char* BasicResourceTypeName<std::vector<std::string> >();
typedef BasicArrayResource<int> IntArrayResource;
typedef BasicArrayResource<double> FloatArrayResource;
typedef BasicArrayResource<std::string> StringArrayResource;


/** @brief A resource that contains a ResourceLibrary.  Useful for
 * hierarchical resource libraries.
 */
class ResourceLibraryResource : public CompoundResourceBase
{
 public:
  virtual bool Load(const std::string& fn);
  virtual bool Save(const std::string& fn);
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual const char* Type() const { return "ResourceLibrary"; }
  virtual ResourceBase* Make() { return new ResourceLibraryResource; }
  virtual ResourceBase* Copy();
  virtual std::vector<std::string> SubTypes() const;
  virtual bool Extract(const char* subtype,std::vector<ResourcePtr>& subobjects);
  virtual bool Pack(std::vector<ResourcePtr>& subobjects,std::string* errorMessage=NULL);
  virtual bool Unpack(std::vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  ResourceLibrary library;
};

template <class T>
void ResourceLibrary::AddType()
{
  ResourcePtr res(new T);
  if(knownTypes.count(res->Type()) == 0) {
    knownTypes[res->Type()].push_back(res);
  }
  else {
    //potential conflict? check raw string
    if(knownTypes[res->Type()][0]->Type() != res->Type())
            LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary: Potential conflict with type "<<res->Type());
    knownTypes[res->Type()][0] = res;
  }
}

template <class T>
void ResourceLibrary::AddLoader(const std::string& ext)
{
  AddType<T>();
  ResourcePtr res(new T);
  loaders[ext].push_back(res);
}

template <class T>
T* ResourceLibrary::Get(const std::string& name,int index)
{
  std::vector<ResourcePtr>& items=Get(name);
  if(index < 0 || index > (int)items.size()) return NULL;
  return dynamic_cast<T*>(items[index].get());
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
    cast_items[i] = dynamic_cast<T*>(items[i].get());
  return cast_items;
}

template <class T>
size_t ResourceLibrary::CountByType() const
{
  T temp;
  return CountByType(temp.Type());
}

#endif

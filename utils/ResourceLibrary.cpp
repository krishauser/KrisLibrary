#include <KrisLibrary/Logger.h>
#include "ResourceLibrary.h"
#include "AnyCollection.h"
#include "stringutils.h"
#include "fileutils.h"
#include "ioutils.h"
#include <set>
#include <fstream>
#include <sstream>
#if HAVE_TINYXML
#include <tinyxml.h>
#else
class TiXmlElement {};
#endif
using namespace std;

template <> const char* BasicResourceTypeName<int>() { return "int"; }
template <> const char* BasicResourceTypeName<double>() { return "double"; }
template <> const char* BasicResourceTypeName<std::string>() { return "string"; }
template <> const char* BasicResourceTypeName<std::vector<int> >() { return "vector<int>"; }
template <> const char* BasicResourceTypeName<std::vector<double> >() { return "vector<double>"; }
template <> const char* BasicResourceTypeName<std::vector<std::string> >() { return "vector<string>"; }

template class BasicResource<int>;
template class BasicResource<double>;
template class BasicResource<std::string>;
template class BasicArrayResource<int>;
template class BasicArrayResource<double>;

//specialization to allow for whitespace in strings
template <>
bool BasicArrayResource<std::string>::Load(std::istream& in) {
  size_t n;
  in>>n;
  if(in.bad()) return false;
  data.resize(n);
  for(size_t i=0;i<n;i++) {
    if(!SafeInputString(in,data[i])) return false;
  }
  return true;
}
//specialization to allow for whitespace in strings
template <>
bool BasicArrayResource<std::string>::Save(std::ostream& out) {
  out<<data.size()<<'\t';
  for(size_t i=0;i<data.size();i++) {
    SafeOutputString(out,data[i]);
    if(i+1!=data.size()) out<<" ";
  }
  out<<std::endl;
  return true;
}
template class BasicArrayResource<std::string>;


vector<ResourcePtr> ResourcesByType(std::vector<ResourcePtr>& resources,const std::string& type)
{
  vector<ResourcePtr> res;
  for(size_t i=0;i<resources.size();i++)
    if(type == resources[i]->Type())
      res.push_back(resources[i]);
  return res;
}

vector<string> ResourceTypes(const vector<ResourcePtr>& resources)
{
  set<string> types;
  for(size_t i=0;i<resources.size();i++)
    types.insert(resources[i]->Type());
  return vector<string>(types.begin(),types.end());
}

ResourceBase::ResourceBase()
{}

ResourceBase::ResourceBase(const string& _name)
  :name(_name)
{}

ResourceBase::ResourceBase(const string& _name,const string& fn)
  :name(_name),fileName(fn)
{}

bool ResourceBase::Load(AnyCollection& c)
{
  string data;
  if(c["data"].as<string>(data)) {
    stringstream ss(data);
    return Load(ss);
  }
  return false;
}

bool ResourceBase::Save(AnyCollection& c)
{
  stringstream ss;
  if(!Save(ss)) return false;
  c["data"] = ss.str();
  return true;
}

bool ResourceBase::Load(TiXmlElement* in)
{
#if HAVE_TINYXML
  if(in->GetText()) {
    stringstream ss(in->GetText());
    return Load(ss);
  }
#endif
  return false;
}

bool ResourceBase::Save(TiXmlElement* out)
{
#if HAVE_TINYXML
  stringstream ss;
  if(!Save(ss)) return false;
  TiXmlText text(ss.str().c_str());
  text.SetCDATA(true);
  out->InsertEndChild(text);
  return true;
#else
  return false;
#endif //HAVE_TINYXML
}

bool ResourceBase::Load(const string& fn)
{
  ifstream in(fn.c_str(),ios::in);
  if(!in) return false; 
  if(!Load(in)) {
#if HAVE_TINYXML
    TiXmlDocument doc;
    if(doc.LoadFile(fn)) {
      if(Load(doc.RootElement())) {
	fileName = fn;
	return true;
      }
    }
#endif
    return false;
  }
  fileName = fn;
  return true;
}

bool ResourceBase::Load()
{
  return Load(fileName);
}

bool ResourceBase::Save(const string& fn)
{
  //create the path if it does not exist
  string path = GetFilePath(fn);
  if(!path.empty()) {
    if(!FileUtils::IsDirectory(path.c_str()))
      FileUtils::MakeDirectory(path.c_str());
  }
  //save
  ofstream out(fn.c_str(),ios::out);
  if(!out) return false; 
  if(Save(out)) return true;
  out.close();

#if HAVE_TINYXML
  TiXmlDocument doc;
  TiXmlElement* element = new TiXmlElement("ResourceBase");
  doc.LinkEndChild(element);
  if(Save(doc.RootElement())) {
    if(doc.SaveFile(fn)) {
      return true;
    }
  }
#endif
  return false;
}

bool ResourceBase::Save()
{
  return Save(fileName); 
}

bool CompoundResourceBase::Extract(const char* subtype,std::vector<ResourcePtr>& subobjects)
{
  std::vector<ResourcePtr> all;
  if(Unpack(all)) {
    subobjects = ResourcesByType(all,subtype);
    if(subobjects.empty()) {
      //no objects -- do we extract of this given type?
      vector<string> extractTypes = ExtractTypes();
      for(size_t i=0;i<extractTypes.size();i++)
	if(subtype == extractTypes[i]) return true;
      return false;
    }
    else
      return true;
  }
  return false;
}


void ResourceLibrary::Clear()
{
  itemsByName.clear();
  itemsByType.clear();
}

bool ResourceLibrary::SaveXml(const std::string& fn)
{
#if HAVE_TINYXML
  TiXmlDocument doc;
  TiXmlElement* element = new TiXmlElement("resource_library");
  doc.LinkEndChild(element);
  if(!Save(element)) return false;
  return doc.SaveFile(fn.c_str());
#else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::SaveXml(): tinyxml not defined\n");
  return false;
#endif
}

bool ResourceLibrary::Save(TiXmlElement* root)
{
#if HAVE_TINYXML
  root->SetValue("resource_library");
  Assert(root != NULL);
  bool res=true;
  for(Map::iterator i=itemsByType.begin();i!=itemsByType.end();i++) {
    for(size_t j=0;j<i->second.size();j++) {
      TiXmlElement* c = new TiXmlElement(i->first.c_str());
      c->SetAttribute("name",i->second[j]->name.c_str());
      if(!i->second[j]->fileName.empty())
	c->SetAttribute("file",i->second[j]->fileName.c_str());
      if(i->second[j]->Save(c)) {
	//ok, saved directly to TiXmlElement
	root->LinkEndChild(c);
      }
      else {
	if(!i->second[j]->Save()) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::SaveXml(): "<<i->second[j]->name<<" failed to save to "<<i->second[j]->fileName);
	  res=false;
	  delete c;
	}
	else {
	  root->LinkEndChild(c);
	}
      }
    }
  }
  return res;
#else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Save(): tinyxml not defined\n");
  return false;
#endif
}

bool ResourceLibrary::SaveJSON(std::ostream& s)
{
  AnyCollection c;
  if(!Save(c)) return false;
  c.write(s);
  return true;
}

bool ResourceLibrary::Save(AnyCollection& c)
{
  c.clear();
  c.resize(itemsByType.size());
  int k=0;
  for(Map::iterator i=itemsByType.begin();i!=itemsByType.end();i++,k++) {
    for(size_t j=0;j<i->second.size();j++) {
      if(i->second[j]->Save(c[k])) {
	//ok, saved directly to AnyCollection
	c[k]["type"] = string(i->second[j]->Type());
	c[k]["name"] = i->second[j]->name;
      }
      else if(!i->second[j]->fileName.empty()) {
	c[k]["type"] = string(i->second[j]->Type());
	c[k]["name"] = i->second[j]->name;
	c[k]["file"] = i->second[j]->fileName;
	if(!i->second[j]->Save()) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Save(): "<<i->second[j]->name<<" failed to save to "<<i->second[j]->fileName);
	  return false;
	}
      }
      else {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Save(): "<<i->second[j]->name<<" failed to save to AnyCollection or file");
	return false;
      }
    }
  }
  return true;
}

bool ResourceLibrary::LoadXml(const std::string& fn)
{
#if HAVE_TINYXML
  TiXmlDocument doc;
  //whitespace needs to be preserved for some items to load properly
  TiXmlBase::SetCondenseWhiteSpace(false);

  if(!doc.LoadFile(fn.c_str()))
     return false;
  return Load(doc.RootElement());
#else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): tinyxml not defined\n");
  return false;
#endif
}

bool ResourceLibrary::Load(TiXmlElement* root)
{
#if HAVE_TINYXML
  if(0 != strcmp(root->Value(),"resource_library")) return false;
  TiXmlElement* element = root->FirstChildElement();
  bool res=true;
  while(element != NULL) {
    if(knownTypes.count(element->Value()) == 0) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): do not know how to load objects of type "<<element->Value());
      res = false;
    }
    else {
      shared_ptr<ResourceBase> resource(knownTypes[element->Value()][0]->Make());
      if(element->Attribute("name")!=NULL)
	resource->name = element->Attribute("name");
      if(element->Attribute("file")!=NULL) {
	resource->fileName = element->Attribute("file");
	if(!resource->Load()) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): error loading element of type "<<element->Value());
	  res = false;
	}
	else {
	  Add(resource);
	}
      }
      else if(!resource->Load(element)) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): error loading element of type "<<element->Value());
	res = false;
      }
      else {
	Add(resource);
      }
    }
    element = element->NextSiblingElement();
  }
  return res;
#else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Load(): tinyxml not defined\n");
  return false;
#endif
}


bool ResourceLibrary::LoadJSON(istream& s)
{
  AnyCollection c;
  if(!c.read(s)) return false;
  return Load(c);
}

bool ResourceLibrary::Load(AnyCollection& c)
{
  vector<shared_ptr<AnyCollection> > subitems;
  c.enumerate(subitems);
  for(size_t i=0;i<subitems.size();i++) {
    AnyCollection& c=(*subitems[i]);
    string type;
    if(!c["type"].as<string>(type)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ResourceLibrary::Load: Item "<<i);
      return false;
    }
    if(knownTypes.count(type) == 0) {    
      LOG4CXX_INFO(KrisLibrary::logger(),"ResourceLibrary::Load: Item "<<i<<" type "<<type.c_str());
      return false;
    }
    shared_ptr<ResourceBase> res(knownTypes[type][0]->Make());
    c["name"].as<string>(res->name);
    if(c["file"].as<string>(res->fileName)) {
      if(!res->Load()) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Load: Error reading item "<<i);
	return false;
      }
    }
    else if(!res->Load(c)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::Load: Error reading item "<<i);
      return false; 
    }
    Add(res);
  }
  return true;
}

bool ResourceLibrary::LazyLoadXml(const std::string& fn)
{
#if HAVE_TINYXML
  TiXmlDocument doc;
  if(!doc.LoadFile(fn.c_str()))
     return false;
  TiXmlElement* element = doc.RootElement()->FirstChildElement();
  bool res=true;
  while(element != NULL) {
    if(knownTypes.count(element->Value()) == 0) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): do not know how to load objects of type "<<element->Value());
      res = false;
    }
    else {
      ResourceBase* resource=knownTypes[element->Value()][0]->Make();
      if(element->Attribute("name") != NULL)
	resource->name = element->Attribute("name");
      if(element->Attribute("file") != NULL) {
	resource->fileName = element->Attribute("file");
	Add(ResourcePtr(resource));
      }
      else {
	if(!resource->Load(element)) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LoadXml(): error loading element of type "<<element->Value());
	  delete resource;
	  res = false;
	}
	else 
	  Add(ResourcePtr(resource));
      }
    }
    element = element->NextSiblingElement();
  }
  return res;
#else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LazyLoadXml(): tinyxml not defined\n");
  return false;
#endif
}


bool ResourceLibrary::LazyLoadJSON(std::istream& s)
{
  AnyCollection c;
  if(!c.read(s)) return false;
  return LazyLoad(c);
}

bool ResourceLibrary::LazyLoad(AnyCollection& c)
{
  vector<shared_ptr<AnyCollection> > subitems;
  c.enumerate(subitems);
  for(size_t i=0;i<subitems.size();i++) {
    AnyCollection& c=*subitems[i];
    string type;
    if(!c["type"].as<string>(type)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ResourceLibrary::LazyLoad: Item "<<i);
      return false;
    }
    if(knownTypes.count(type) == 0) {    
      LOG4CXX_INFO(KrisLibrary::logger(),"ResourceLibrary::LazyLoad: Item "<<i<<" type "<<type.c_str());
      return false;
    }
    shared_ptr<ResourceBase> res(knownTypes[type][0]->Make());
    c["name"].as<string>(res->name);
    if(c["file"].as<string>(res->fileName)) {
      //lazy load
    }
    else if(!res->Load(c)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ResourceLibrary::LazyLoad: Error reading item "<<i);
      return false;
    }
    Add(res);
  }
  return true;
}

ResourcePtr ResourceLibrary::LoadItem(const string& fn)
{
  string ext=FileExtension(fn);
  Map::iterator i=loaders.find(ext);
  if(i == loaders.end()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"No known loaders for type "<<ext.c_str());
    return NULL;
  }
  for(size_t j=0;j<i->second.size();j++) {
    ResourcePtr r(i->second[j]->Make());
    if(r->Load(fn)) {
      r->name = GetFileName(fn);
      StripExtension(r->name);
      Add(r);
      return r;
    }
  }
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load "<<fn.c_str());
  for(size_t j=0;j<i->second.size();j++) 
        LOG4CXX_ERROR(KrisLibrary::logger()," "<<i->second[j]->Type());
    LOG4CXX_ERROR(KrisLibrary::logger(),"\n");
  return NULL;
}

bool ResourceLibrary::ReloadAll()
{
  bool res=true;
  for(Map::iterator i=itemsByName.begin();i!=itemsByName.end();i++) {
    for(size_t j=0;j<i->second.size();j++)
      if(!i->second[j]->Load()) res=false;
  }
  return res;
}

bool ResourceLibrary::SaveItem(const string& name)
{
  Map::iterator i=itemsByName.find(name);
  if(i==itemsByName.end()) return false;
  bool res=true;
  for(size_t j=0;j<i->second.size();j++)
    if(!i->second[j]->Save()) res=false;
  return res;
}

bool ResourceLibrary::SaveAll()
{
  bool res=true;
  for(Map::iterator i=itemsByName.begin();i!=itemsByName.end();i++) {
    for(size_t j=0;j<i->second.size();j++)
      if(!i->second[j]->Save()) res=false;
  }
  return res;
}

bool ResourceLibrary::LoadAll(const string& dir)
{
  vector<string> files;
  FileUtils::ListDirectory(dir.c_str(),files);
  bool res=true;
  vector<string> path(2);
  path[0] = dir;
  for(size_t i=0;i<files.size();i++) {
    if(files[i] == ".") continue;
    if(files[i] == "..") continue;
    path[1] = files[i];
    string filepath = JoinPath(path);
    if(FileUtils::IsDirectory(filepath.c_str())) {
      ResourceLibrary sublib;
      swap(knownTypes,sublib.knownTypes);
      swap(loaders,sublib.loaders);
      if(!sublib.LoadAll(filepath)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load sub-directory "<<filepath);
	res = false;
      }
      for(Map::iterator j=sublib.itemsByName.begin();j!=sublib.itemsByName.end();j++) {
	for(size_t k=0;k<j->second.size();k++) {
	  j->second[k]->name = files[i] + "/" + j->second[k]->name;
	  Add(j->second[k]);
	}
      }
      swap(knownTypes,sublib.knownTypes);
      swap(loaders,sublib.loaders);
    }
    else {
      if(LoadItem(filepath) == NULL) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load file "<<filepath);
	res = false;
      }
    }
  }
  return res;
}

bool ResourceLibrary::LazyLoadAll(const string& dir)
{
  vector<string> files;
  FileUtils::ListDirectory(dir.c_str(),files);
  bool res=true;
  vector<string> path(2);
  path[0] = dir;
  for(size_t i=0;i<files.size();i++) {
    if(files[i] == ".") continue;
    if(files[i] == "..") continue;
    path[1] = files[i];
    string fn = JoinPath(path);
    string ext=FileExtension(fn);
    Map::iterator it=loaders.find(ext);
    if(it == loaders.end()) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load file "<<fn);
      res = false;
      continue;
    }
    for(size_t j=0;j<it->second.size();j++) {
      ResourcePtr r(it->second[j]->Make());
      r->name = GetFileName(fn);
      StripExtension(r->name);
      Add(r);
      return true;
    }
  }
  return res;
}


void ResourceLibrary::AddBaseDirectory(const string& dir)
{
  if(dir.empty()) return;
  for(Map::iterator i=itemsByName.begin();i!=itemsByName.end();i++) {
    for(size_t j=0;j<i->second.size();j++) {
      i->second[j]->fileName=JoinPath(dir,i->second[j]->fileName);
    }
  }
}

void ResourceLibrary::ChangeBaseDirectory(const string& dir)
{
  vector<string> oldpath,newpath;
  for(Map::iterator i=itemsByName.begin();i!=itemsByName.end();i++) {
    for(size_t j=0;j<i->second.size();j++) {
      SplitPath(i->second[j]->fileName,oldpath);
      if(dir.empty()) {
	if(oldpath.size()==1) newpath=oldpath;
	else {
	  newpath.resize(oldpath.size()-1);
	  copy(oldpath.begin()+1,oldpath.end(),newpath.begin());
	}
      }
      else {
	newpath.resize(oldpath.size()+1);
	newpath[0] = dir;
	copy(oldpath.begin(),oldpath.end(),newpath.begin()+1);
      }
      i->second[j]->fileName=JoinPath(newpath);
    }
  }
}

std::vector<ResourcePtr >& ResourceLibrary::Get(const string& name)
{
  return itemsByName[name];
}

std::vector<ResourcePtr >& ResourceLibrary::GetByType(const std::string& type)
{
  return itemsByType[type];
}

size_t ResourceLibrary::Count(const std::string& name) const
{
  Map::const_iterator i=itemsByName.find(name);
  if(i==itemsByName.end()) return 0;
  return i->second.size();
}

size_t ResourceLibrary::CountByType(const std::string& name) const
{
  Map::const_iterator i=itemsByType.find(name);
  if(i==itemsByType.end()) return 0;
  return i->second.size();
}

std::vector<ResourcePtr> ResourceLibrary::Enumerate() const
{
  std::vector<ResourcePtr> res;
  for(Map::const_iterator i=itemsByName.begin();i!=itemsByName.end();i++) 
    res.insert(res.end(),i->second.begin(),i->second.end());
  return res;
}

void ResourceLibrary::Add(const ResourcePtr& r)
{
  itemsByName[r->name].push_back(r);
  itemsByType[r->Type()].push_back(r);
}

bool ResourceLibrary::Erase(const ResourcePtr& resource)
{
  Map::iterator i=itemsByName.find(resource->name);
  if(i==itemsByName.end()) return false;
  Map::iterator it=itemsByType.find(resource->Type());
  Assert(it != itemsByType.end());
  for(size_t j=0;j<it->second.size();j++) {
    if(it->second[j] == resource) {
      it->second.erase(it->second.begin()+j);
      break;
    }
  }
  for(size_t j=0;j<i->second.size();j++) {
    if(i->second[j] == resource) {
      i->second.erase(i->second.begin()+j);
      return true;
    }
  }
  return false;
}

bool ResourceLibrary::Erase(const std::string& name,int index)
{
  Map::iterator i=itemsByName.find(name);
  if(i==itemsByName.end()) return false;
  if(index < 0 || index >= (int)i->second.size()) return false;
  ResourcePtr r=i->second[index];
  Map::iterator it=itemsByType.find(r->Type());
  Assert(it != itemsByType.end());
  for(size_t j=0;j<it->second.size();j++) {
    if(it->second[j] == r) {
      it->second.erase(it->second.begin()+j);
      break;
    }
  }  
  i->second.erase(i->second.begin()+index);
  return true;
}

bool ResourceLibraryResource::Load(AnyCollection& c)
{
  return library.Load(c["data"]);
}
bool ResourceLibraryResource::Save(AnyCollection& c)
{
  return library.Save(c["data"]);
}

ResourceBase* ResourceLibraryResource::Copy()
{
  ResourceLibraryResource* res = new ResourceLibraryResource;
  res->library = library;
  return res;
}

std::vector<std::string> ResourceLibraryResource::SubTypes() const
{ 
  std::vector<std::string> res;
  for(ResourceLibrary::Map::const_iterator i=library.itemsByType.begin();i!=library.itemsByType.end();i++)
    res.push_back(i->first);
  return res;
}

bool ResourceLibraryResource::Extract(const char* subtype,std::vector<ResourcePtr>& res)
{
  if(library.itemsByType.count(subtype)==0) return false;
  res = library.itemsByType.find(subtype)->second;
  return true;
}

bool ResourceLibraryResource::Pack(std::vector<ResourcePtr>& subobjects,std::string* errorMessage)
{
  library.Clear();
  for(size_t i=0;i<subobjects.size();i++)
    library.Add(subobjects[i]);
  return true;
}

bool ResourceLibraryResource::Unpack(std::vector<ResourcePtr>& subobjects,bool* incomplete)
{
  subobjects = library.Enumerate();
  return true;
}

bool ResourceLibraryResource::Load(const std::string& fn)
{
  //if fn is a directory, loads from a directory
  if(FileUtils::IsDirectory(fn.c_str())) 
    return library.LoadAll(fn);
  else 
    return library.LoadXml(fn);
}

bool ResourceLibraryResource::Save(const std::string& fn)
{
  //if fn is a directory, saves to a directory  
  if(FileUtils::IsDirectory(fn.c_str()) || fn[fn.length()-1]=='/' || fn[fn.length()-1]=='\\') {
    library.ChangeBaseDirectory(fn);
    return library.SaveAll();
  }
  else {
    return library.SaveXml(fn);
  }
}

std::string ResourceLibrary::DefaultFileName(const ResourcePtr& r)
{
  std::string ext = "txt";
  for(Map::const_iterator i=loaders.begin();i!=loaders.end();i++) {
    for(size_t j=0;j<i->second.size();j++) {
      if(string(i->second[j]->Type())==string(r->Type())) {
	ext = i->first;
	break;
      }
    }
  }
  return r->name+"."+ext;
}

#include "ResourceLibrary.h"
#include "stringutils.h"
#include "fileutils.h"
#include "ioutils.h"
#include <fstream>
#include <sstream>
#if HAVE_TINYXML
#include <tinyxml.h>
#else
class TiXmlElement {};
#endif
using namespace std;

template class BasicResource<int>;
template class BasicResource<double>;
template class BasicResource<std::string>;
template <> const char* BasicResource<int>::className = "int";
template <> const char* BasicResource<double>::className = "double";
template <> const char* BasicResource<std::string>::className = "string";

ResourceBase::ResourceBase()
{}

ResourceBase::ResourceBase(const string& _name)
  :name(_name)
{}

ResourceBase::ResourceBase(const string& _name,const string& fn)
  :name(_name),fileName(fn)
{}

bool ResourceBase::Load(TiXmlElement* in)
{
#if HAVE_TINYXML
  if(0 != strcmp(in->Value(),Type())) {
    printf("ResourceBase::Load: Element %s doesn't have type %s\n",in->Value(),Type());
    return false;
  }
  if(in->Attribute("name")!=NULL)
    name = in->Attribute("name");
  if(in->Attribute("file")!=NULL)
    fileName = in->Attribute("file");
  if(in->GetText()) {
    stringstream ss(in->GetText());
    return Load(ss);
  }
  else if(!fileName.empty()) {
    return Load(fileName);
  }
#endif
  return false;
}

bool ResourceBase::Save(TiXmlElement* out)
{
#if HAVE_TINYXML
  out->SetValue(Type());
  if(!name.empty())
    out->SetAttribute("name",name);
  if(!fileName.empty())
    out->SetAttribute("file",fileName);
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
      FileUtils::CreateDirectory(path.c_str());
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
  fprintf(stderr,"ResourceLibrary::SaveXml(): tinyxml not defined\n");
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
	//ok
	root->LinkEndChild(c);
      }
      else {
	if(!i->second[j]->Save()) {
	  cerr<<"ResourceLibrary::SaveXml(): "<<i->second[j]->name<<" failed to save to "<<i->second[j]->fileName<<endl;
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
  fprintf(stderr,"ResourceLibrary::Save(): tinyxml not defined\n");
  return false;
#endif

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
  fprintf(stderr,"ResourceLibrary::LoadXml(): tinyxml not defined\n");
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
      fprintf(stderr,"ResourceLibrary::LoadXml(): do not know how to load objects of type %s\n",element->Value());
      res = false;
    }
    else {
      ResourceBase* resource=knownTypes[element->Value()][0]->Make();
      if(!resource->Load(element)) {
	fprintf(stderr,"ResourceLibrary::LoadXml(): error loading element of type %s\n",element->Value());
	delete resource;
	res = false;
      }
      else {
	//handle loading of name and filename if the resource doesnt do it already
	if(resource->name.empty()) {
	  if(element->Attribute("name") != NULL)
	    resource->name = element->Attribute("name");
	}
	if(resource->fileName.empty()) {
	  if(element->Attribute("file") != NULL)
	    resource->fileName = element->Attribute("file");
	}
	Add(resource);
      }
    }
    element = element->NextSiblingElement();
  }
  return res;
#else
  fprintf(stderr,"ResourceLibrary::Load(): tinyxml not defined\n");
  return false;
#endif
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
      fprintf(stderr,"ResourceLibrary::LoadXml(): do not know how to load objects of type %s\n",element->Value());
      res = false;
    }
    else {
      ResourceBase* resource=knownTypes[element->Value()][0]->Make();
      if(element->Attribute("name") != NULL)
	resource->name = element->Attribute("name");
      if(element->Attribute("file") != NULL) {
	resource->fileName = element->Attribute("file");
	Add(resource);
      }
      else {
	if(!resource->Load(element)) {
	  fprintf(stderr,"ResourceLibrary::LoadXml(): error loading element of type %s\n",element->Value());
	  delete resource;
	  res = false;
	}
	else 
	  Add(resource);
      }
    }
    element = element->NextSiblingElement();
  }
  return res;
#else
  fprintf(stderr,"ResourceLibrary::LazyLoadXml(): tinyxml not defined\n");
  return false;
#endif
}


ResourcePtr ResourceLibrary::LoadItem(const string& fn)
{
  string ext=FileExtension(fn);
  Map::iterator i=loaders.find(ext);
  if(i == loaders.end()) {
    fprintf(stderr,"No known loaders for type %s\n",ext.c_str());
    return NULL;
  }
  for(size_t j=0;j<i->second.size();j++) {
    ResourcePtr r = i->second[j]->Make();
    if(r->Load(fn)) {
      r->name = GetFileName(fn);
      StripExtension(r->name);
      Add(r);
      return r;
    }
  }
  fprintf(stderr,"Unable to load %s as types:",fn.c_str());
  for(size_t j=0;j<i->second.size();j++) 
    fprintf(stderr," %s",i->second[j]->Type());
  fprintf(stderr,"\n");
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
	cerr<<"Unable to load sub-directory "<<filepath<<endl;
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
	cerr<<"Unable to load file "<<filepath<<endl;
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
      cerr<<"Unable to load file "<<fn<<endl;
      res = false;
      continue;
    }
    for(size_t j=0;j<it->second.size();j++) {
      ResourcePtr r = it->second[j]->Make();
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

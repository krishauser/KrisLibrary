#include <KrisLibrary/Logger.h>
#include "SimpleFile.h"
#include "stringutils.h"
#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

SimpleFile::SimpleFile(const char* fn)
{
  Load(fn);
}

SimpleFile::SimpleFile(istream& in)
{
  Load(in);
}

SimpleFile::SimpleFile()
{
  loaded = false;
}

void SimpleFile::AllowItem(const std::string& str,bool caseSensitive)
{
  if(!caseSensitive) {
    string s = str;
    Lowercase(s);
    validItems[s] = caseSensitive;
  }
  else
    validItems[str] = caseSensitive;
}

bool SimpleFile::Load(const char* fn)
{
  ifstream in(fn);
  if(!in) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile::Load(): Unable to open file "<<fn);
    loaded = false;
    return false;
  }
  return Load(in);
}

bool SimpleFile::Load(istream& in)
{
  string name;
  string line;
  char buf[1025];
  int lineno=1;
  while(in >> name) {
    //read the rest of the line
    lineno++;
    buf[1024]=0;
    line.clear();
    bool foundEndline=false,comment=false;
    while(!foundEndline) {
      for(int i=0;i<1024;i++) {
	int c=in.get();
	if(c == '\n' || c == EOF) {
	  buf[i] = 0;
	  foundEndline = true;
	  break;
	}
	else if(c == '#' || comment) {
	  c = 0;
	  comment = true;
	}
	buf[i] = c;
      }
      line += buf;
    }
    if(name[0]=='#') continue;  //ignore comment up to end of line
    if(!validItems.empty()) {
      if(validItems.count(name) == 0) {
	string temp=name;
	Lowercase(temp);
	if(validItems.count(temp) != 0) {
	  if(validItems[temp] == true) { //case sensitive
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile::Load: unsupported item "<<name.c_str()<<" on line "<<lineno);
	    return false;
	  }
	}
	else {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile::Load: unsupported item "<<name.c_str()<<" on line "<<lineno);
	  return false;
	}
      }
    }

    stringstream ss;
    ss.str(line);

    vector<PrimitiveValue>& items=entries[name];
    items.clear();
    PrimitiveValue value;
    while(ss>>value) {
      items.push_back(value);
    }
  }
  loaded=true;
  if(in.bad()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile::Load() Bad value on line "<<lineno);
    loaded=false;
  }
  return !in.bad();
}

bool SimpleFile::Save(const char* fn)
{
  ofstream out(fn);
  if(!out) return false;
  return Save(out);
}

bool SimpleFile::Save(ostream& out)
{
  for(map<string,vector<PrimitiveValue> >::const_iterator i=entries.begin();i!=entries.end();i++) {
    out<<i->first<<"\t";
    for(size_t j=0;j<i->second.size();j++)
      out<<i->second[j]<<" ";
    out<<endl;
  }
  return true;
}

bool SimpleFile::CheckSize(const string& name,int size,const char* errorString)
{
  const char* title=errorString;
  if(errorString==NULL) title = "Untitled file";
  if(entries.count(name) == 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile("<<title<<") Wrong number of items in "<< name.c_str()<<", entry not present\n");
        return false;
  }
  if((int)entries[name].size() != size) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile("<<title<<"): Wrong number of items in "<<name.c_str()<<".  Need "<<size<<", have "<<entries[name].size());
    return false;
  }
  return true;
}

bool SimpleFile::CheckType(const string& name,int type,const char* errorString)
{
  const char* title=errorString;
  if(errorString==NULL) title = "Untitled file";
  if(entries.count(name) == 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile("<<title<<"): Wrong entry type in "<<name.c_str() <<" entry not present\n");
          return false;
  }
  vector<PrimitiveValue>& items = entries[name];
  for(size_t i=0;i<items.size();i++) {
    if(!items[i].CanCast(type)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleFile("<<title<<"): Wrong entry type in "<<name.c_str()<<".  Need "<<type<<", have "<<items[i].type);
      LOG4CXX_ERROR(KrisLibrary::logger(),"   Item "<<i<<": "<<items[i]);
      return false;
    }
  }
  return true;
}

vector<int> SimpleFile::AsInteger(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<int> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsInteger();
  return res;
}

vector<double> SimpleFile::AsDouble(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<double> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsDouble();
  return res;
}

vector<string> SimpleFile::AsString(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<string> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsString();
  return res;
}
